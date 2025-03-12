import numpy as np
from scipy.spatial.transform import Rotation as R

from motion_planning.panda_moveit_controller import PandaMoveItController
from motion_planning.summitxl_movebase_controller import SummitXLMovebaseController
        
import os
import message_filters
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import tf.transformations as tft
import json

target_found = False
target_name = None
target_semantic_value = None
semantic_labels_list = []
target_points = set()

tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)

bridge = CvBridge()


def save_points_to_npy(points, filename):
    # 将点集转换为NumPy数组
    points_array = np.array(list(points))
    np.save(filename, points_array)

def save_knowledge_base_of_items(scene_name, room_name, carrier_name, target_name,
                                semantic_labels_list, target_path=None, target_point=None):
    """
    Save item information to a JSON file.

    Args:
        scene_name: Name of the scene
        room_name: Name of the room
        carrier_name: Name of the carrier
        target_name: Name of the target
        semantic_labels_list: List of semantic labels
        target_path: Optional path to target
        target_point: Optional target point
    """
    # 创建输出目录
    output_dir = os.path.expanduser('~/ws_thesis_lzq/src/find_target/scripts/scene_level/previous_knowledge_base')
    os.makedirs(output_dir, exist_ok=True)

    # 构建输出文件路径
    output_file = os.path.join(output_dir, 'saved_items_results.json')

    try:
        # 读取已存在的数据
        if os.path.exists(output_file):
            with open(output_file, 'r') as f:
                existing_data = json.load(f)
        else:
            existing_data = {}

        # 处理目标数据
        if target_point is not None:
            target_data = {
                'scene_name': scene_name,
                'room_name': room_name,
                'carrier_name': carrier_name,
                'target_path': target_path,
                'target_point': target_point
            }
            existing_data[target_name] = target_data

        # 处理语义标签
        # 移除已存在的carrier_name
        updated_semantic_labels = [label for label in semantic_labels_list if label != carrier_name]
        
        # 更新载体的语义标签
        if 'semantic_labels' not in existing_data[carrier_name]:
            existing_data[carrier_name]['semantic_labels'] = []
        
        # 添加新的唯一语义标签
        for label in updated_semantic_labels:
            if label not in existing_data[carrier_name]['semantic_labels']:
                existing_data[carrier_name]['semantic_labels'].append(label)

        # 写入JSON文件
        with open(output_file, 'w') as f:
            json.dump(existing_data, f, indent=4)

        print(f":white_check_mark: Successfully saved item information to {output_file}")
    except Exception as e:
        raise IOError(f"Failed to write knowledge base: {e}")

def semantic_labels_callback(msg):
    """处理语义标签信息，更新标签字典和列表"""
    global target_found, target_name, target_semantic_value, semantic_labels_list
    target_semantic_value = None
    try:
        data = json.loads(msg.data)
        for key, value in data.items():
            if key.isdigit():
                class_label = value.get("class", "")
                if class_label not in ["BACKGROUND", "UNLABELLED"]:
                    # 更新语义标签列表
                    if class_label not in semantic_labels_list:
                        semantic_labels_list.append(class_label)
                    
                    # 检查是否是目标物体
                    if class_label == target_name:
                        target_found = True
                        target_semantic_value = int(key)           
    except json.JSONDecodeError:
        rospy.logwarn("Failed to decode JSON message")

def depth_semantic_callback(depth_msg, semantic_msg):
    """
    处理深度和语义图像，提取载体的点云
    更高效的点云处理和坐标变换
    """
    global target_points, target_semantic_value

    try:
        # 使用更高效的转换方法
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        semantic_image = bridge.imgmsg_to_cv2(semantic_msg, desired_encoding="32SC1")
        
        # 预先获取变换以减少重复查询
        try:
            transform = tf_buffer.lookup_transform(
                'map',          # 目标框架
                'Hand_Camera',   # 源框架
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return

        carrier_mask = (semantic_image == target_semantic_value)
            
        # 快速过滤有效点
        valid_mask = (depth_image > 0) & (depth_image < 10) & carrier_mask
            
        # 使用NumPy快速获取坐标
        v, u = np.where(valid_mask)
        depths = depth_image[valid_mask]

        # 相机内参
        fx, fy = 1108.5125, 1108.5125
        cx, cy = 640, 360
            
        # 批量坐标转换
        x = (u - cx) * depths / fx
        y = (v - cy) * depths / fy
        z = depths
            
        # 创建点云数据
        points_3d_camera = np.column_stack([x, y, z]).astype(np.float32)
            
        # 批量坐标变换
        points_3d_map = transform_points(points_3d_camera, transform)
        
        # 使用NumPy数组替代集合
        new_points = {tuple(point) for point in points_3d_map}
            
        # 合并点云
        if target_points:
            target_points = target_points.union(new_points)
        else:
            target_points = new_points
            
            rospy.loginfo(f"Extracted point cloud for {target_name}: {len(new_points)} points")

    except Exception as e:
        rospy.logerr(f"Error in depth_semantic_callback: {e}")

def transform_points(points, transform):
    """
    将3D点从一个坐标系转换到另一个坐标系
    :param points: N x 3 的数组，3D点
    :param transform: geometry_msgs/TransformStamped 变换
    :return: N x 3 的数组，转换后的3D点
    """
    # 将TransformStamped转换为4x4矩阵
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    # 使用四元数转换为旋转矩阵
    
    rotation_matrix = tft.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    translation_matrix = np.eye(4)
    translation_matrix[0:3, 3] = [translation.x, translation.y, translation.z]
    transform_matrix = np.dot(translation_matrix, rotation_matrix)

    # 将点云转换为齐次坐标
    num_points = points.shape[0]
    points_homogeneous = np.hstack((points, np.ones((num_points, 1))))
    # 应用变换
    points_transformed = np.dot(transform_matrix, points_homogeneous.T).T
    # 去除齐次坐标
    points_3d_odom = points_transformed[:, :3].astype(np.float32)
    return points_3d_odom

def interpolate_arc(center_position, close_door_ee, open_door_ee, num_points):
    # 计算圆弧的半径
    radius = np.linalg.norm(np.array(close_door_ee[:2]) - np.array(center_position))

    # 计算圆心到开门和关门的角度差
    angle_start = np.arctan2(close_door_ee[1] - center_position[1], close_door_ee[0] - center_position[0])
    angle_end = np.arctan2(open_door_ee[1] - center_position[1], open_door_ee[0] - center_position[0])

    # 确保角度差在正确的范围内
    if angle_end < angle_start:
        angle_end += 2 * np.pi

    # 创建角度的插值
    angles = np.linspace(angle_start, angle_end, num_points)

    # 计算圆弧上的点坐标
    arc_points = []
    for angle in angles:
        x = center_position[0] + radius * np.cos(angle)
        y = center_position[1] + radius * np.sin(angle)
        z = close_door_ee[2]  # 假设z轴保持不变
        yaw = angle  # 根据圆弧的角度设置yaw
        arc_points.append([x, y, z, 0, 0, yaw])  # 使用roll, pitch, yaw

    return arc_points

def inside_search_action(scene_name, room_name, carrier_name, target_name):
    panda_controller = PandaMoveItController()
    summit_controller = SummitXLMovebaseController()
    
    if carrier_name == 'cabinet':
        left_close_door_ee = [3.535, 2.83, 0.55, 0, 0, 0]
        left_open_door_ee = [3.795, 2.53, 0.55, 0, 0, 0]
        left_camera_pose = [3.665, 2.68, 0.55, 0, 0, 0]
        left_center_position = [3.515, 2.55]
        left_footprint_pose = [3.8, 2.68, 0]
        right_close_door_ee = [3.535, 3.69, 0.55, 0, 0, 0]
        right_open_door_ee = [3.795, 3.99, 0.55, 0, 0, 0]
        right_camera_pose = [3.8, 3.842, 0.55, 0, 0, 0]
        right_center_position = [3.515, 3.97]
        right_footprint_pose = [-3.522, 3.842, 0]
      
    elif carrier_name == 'fridge':
        left_close_door_ee = [-2.82, -5.16, 1, 0, 0, 0]
        left_open_door_ee = [-2.155, -4.387, 1, 0, 0, 0]
        left_camera_pose = [-2.378, -4.765, 1, 0, 0, 90]
        left_center_position = [-1.985, -5.21]
        left_footprint_pose = [-2.378, -4.2, 90]
        right_close_door_ee = [-2.98, -5.16, 1, 0, 0, 0]
        right_open_door_ee = [-3.865, -4.37, 1, 0, 0, 0]
        right_camera_pose = [-3.422, -4.765, 1, 0, 0, 90]
        right_center_position = [-3.815, -5.21]
        right_footprint_pose = [-3.422, -4.2, 90]


    # 插值并转换为RPY形式
    num_points = 10  # 设置插值点的数量
    left_arc_points = interpolate_arc(left_center_position, left_close_door_ee, left_open_door_ee, num_points)
    right_arc_points = interpolate_arc(right_center_position, right_close_door_ee, right_open_door_ee, num_points)

    print("Left Door Points:", left_arc_points)
    print("Right Door Points:", right_arc_points)
  
    # 先右
    # 第1步：移动底盘到达right_footprint_pose
    summit_controller.move_to_pose(right_footprint_pose)

    # 第2步：控制机械臂到达right_points的第一个位置，用夹爪夹住
    panda_controller.move_to_pose(right_arc_points[0])
    panda_controller.move_to_gripper(True)  # 完全关闭夹爪

    # 第3步：让机械臂依次到达right_points的所有位置，松开夹爪
    for pose in right_arc_points[1:]:
        panda_controller.move_to_pose(pose)  # 到达下一个位置
    panda_controller.move_to_gripper(False)  # 松开夹爪

    # 第4步：让机械臂到达right_camera_pose
    panda_controller.move_to_pose(right_camera_pose)

    # 第5步：订阅语义标签并判断是否有目标物体

    # 创建订阅者
    depth_sub = message_filters.Subscriber("/depth_hand", Image)
    semantic_sub = message_filters.Subscriber("/seg_sem_hand", Image)

    # 设置消息同步器
    sync = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, semantic_sub], 
        queue_size=10, 
        slop=0.1
    )

    # 获取和处理语义标签
    semantic_labels_subscriber = rospy.Subscriber('/semantic_labels_hand', 
                                    String, 
                                    semantic_labels_callback)
    
    if target_found:
        # 如果找到目标，处理图像数据以获取点云
        sync.registerCallback(depth_semantic_callback)
        
        if target_points:  # 如果成功获取到点云
            save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}/{room_name}/{carrier_name}')
            os.makedirs(save_dir, exist_ok=True) # 确保目录存在
            target_path = os.path.join(save_dir, f'{target_name}_point_cloud.npy')
            save_points_to_npy(target_points, target_path)
            sum_x = sum(point[0] for point in target_points)
            sum_y = sum(point[1] for point in target_points)
            sum_z = sum(point[2] for point in target_points)
            
            # 计算点的数量
            num_points = len(target_points)
            
            # 计算平均位置
            target_point = (
                sum_x / num_points,
                sum_y / num_points, 
                sum_z / num_points
            )
            save_knowledge_base_of_items(scene_name, room_name, carrier_name, target_name,
                                            semantic_labels_list, target_path, target_point)
            
            semantic_labels_subscriber.unregister()
            depth_sub.unregister()
            semantic_sub.unregister()
            
            return True

    # 确保在函数结束时注销所有订阅者
    semantic_labels_subscriber.unregister()
    depth_sub.unregister()
    semantic_sub.unregister()

    # 第6步：重新夹住物体并按逆序依次到达right_points的所有位置
    panda_controller.move_to_gripper(True)  # 夹爪完全关闭
    for pose in reversed(right_arc_points[:-1]):
        panda_controller.move_to_pose(pose)  # 逆序到达位置

    # 第7步：松开夹爪
    panda_controller.move_to_gripper(False)  # 松开夹爪

    # 后左
    # 重复上述右侧的所有步骤，只是替换为左侧的位置和姿态
    # 第1步：移动底盘到达left_footprint_pose
    summit_controller.move_to_pose(left_footprint_pose)

    # 第2步：控制机械臂到达left_points的第一个位置，用夹爪夹住
    panda_controller.move_to_pose(left_arc_points[0])
    panda_controller.move_to_gripper(True)  # 完全关闭夹爪

    # 第3步：让机械臂依次到达left_points的所有位置，松开夹爪
    for pose in left_arc_points[1:]:
        panda_controller.move_to_pose(pose)  # 到达下一个位置
    panda_controller.move_to_gripper(False)  # 松开夹爪

    # 第4步：让机械臂到达left_camera_pose
    panda_controller.move_to_pose(left_camera_pose)

    # 第5步：订阅语义标签并判断是否有目标物体
    # 创建订阅者
    depth_sub = message_filters.Subscriber("/depth_hand", Image)
    semantic_sub = message_filters.Subscriber("/seg_sem_hand", Image)

    # 设置消息同步器
    sync = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, semantic_sub], 
        queue_size=10, 
        slop=0.1
    )

    # 获取和处理语义标签
    semantic_labels_subscriber = rospy.Subscriber('/semantic_labels_hand', 
                                    String, 
                                    semantic_labels_callback)
    
    if target_found:
        # 如果找到目标，处理图像数据以获取点云
        sync.registerCallback(depth_semantic_callback)
        
        if target_points:  # 如果成功获取到点云
            save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}/{room_name}/{carrier_name}')
            os.makedirs(save_dir, exist_ok=True) # 确保目录存在
            target_path = os.path.join(save_dir, f'{target_name}_point_cloud.npy')
            save_points_to_npy(target_points, target_path)
            sum_x = sum(point[0] for point in target_points)
            sum_y = sum(point[1] for point in target_points)
            sum_z = sum(point[2] for point in target_points)
            
            # 计算点的数量
            num_points = len(target_points)
            
            # 计算平均位置
            target_point = (
                sum_x / num_points,
                sum_y / num_points, 
                sum_z / num_points
            )
            save_knowledge_base_of_items(scene_name, room_name, carrier_name, target_name,
                                            semantic_labels_list, target_path, target_point)
            
            semantic_labels_subscriber.unregister()
            depth_sub.unregister()
            semantic_sub.unregister()
            
            return True

    # 确保在函数结束时注销所有订阅者
    semantic_labels_subscriber.unregister()
    depth_sub.unregister()
    semantic_sub.unregister()
        
    # 第6步：重新夹住物体并按逆序依次到达right_points的所有位置
    panda_controller.move_to_gripper(True)  # 夹爪完全关闭
    for pose in reversed(left_arc_points[:-1]):
        panda_controller.move_to_pose(pose)  # 逆序到达位置

    # 第7步：松开夹爪
    panda_controller.move_to_gripper(False)  # 松开夹爪

    print(f"在{carrier_name}的inside没找到{target_name}")
    return False