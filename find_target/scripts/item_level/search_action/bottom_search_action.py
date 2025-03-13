import rospy
import numpy as np
import json
import os
import tf2_ros
from cv_bridge import CvBridge
import tf.transformations as tft
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image

from item_level.view_pose_prediction.view_pose_prediction import bottom_search

from item_level.search_action.pose_prediction.footprint_pose import footprint_pose
from item_level.search_action.pose_prediction.sorting_pose import sorting_pose

from motion_planning.panda_moveit_controller import PandaMoveItController
from motion_planning.summitxl_movebase_controller import SummitXLMovebaseController
from motion_planning.transform_to_base_frame import transform_to_base_frame

def bottom_search_action(scene_name, room_name, carrier_name, target_name,
                      carrier_area_occupancy, obstacles_area_occupancy, footprint_area_occupancy,
                      room_x_max, room_x_min, room_y_max, room_y_min,
                      carrier_height, horizontal_fov=60, vertical_fov=60*9/16,
                      grid_interval=0.05):
    """
    在载体顶部搜索目标物体，获取其点云数据，并保存所有语义标签信息
    """
    
    # Step 1: 获取EE的poses
    EE_poses = bottom_search(carrier_area_occupancy, obstacles_area_occupancy,
                          room_x_max, room_x_min, room_y_max, room_y_min,
                          carrier_height, horizontal_fov, vertical_fov, grid_interval)
    print('success 1')
    # Step 2: 计算并生成footprint字典
    footprint_dict = footprint_pose(EE_poses, footprint_area_occupancy, carrier_area_occupancy); print('success 2')
    sorted_footprint_dict = sorting_pose(footprint_dict); print('success 3')
    panda_controller = PandaMoveItController()
    summit_controller = SummitXLMovebaseController()
    
    # 初始化变换后的字典
    transform_sorted_footprint_dict = {}

    # 遍历 sorted_footprint_dict，并应用 transform_to_base_frame 进行变换
    for chassis_pose, ee_poses in sorted_footprint_dict.items():
        # 变换所有末端执行器位姿
        transformed_ee_poses = [
            transform_to_base_frame(list(chassis_pose), ee_pose) for ee_pose in ee_poses
        ]
        # 将变换后的末端执行器位姿保存在 transform_sorting_dict 中
        transform_sorted_footprint_dict[chassis_pose] = transformed_ee_poses
    
    # Step 3: 初始化变量
    target_found = False
    target_points = set()  # 存储目标物体的点云
    target_semantic_value = None  # 目标物体的语义标签值
    semantic_labels_list = []  # 存储所有语义标签
    bridge = CvBridge()
    
    # 创建TF缓冲器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


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
        nonlocal target_found, target_name, target_semantic_value, semantic_labels_list
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
        nonlocal target_points, target_semantic_value

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

    # 创建订阅者
    depth_sub = message_filters.Subscriber("/depth_hand", Image)
    semantic_sub = message_filters.Subscriber("/seg_sem_hand", Image)
    
    # 设置消息同步器
    sync = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, semantic_sub], 
        queue_size=1, 
        slop=1
    )
    
    try:
        # Step 4: 遍历footprint字典
        for key, values in transform_sorted_footprint_dict.items():
            # 移动底盘到目标位置
            summit_controller.move_to_pose(list(key))

            # 获取和处理语义标签
            semantic_labels_subscriber = rospy.Subscriber('/semantic_labels_hand', 
                                            String, 
                                            semantic_labels_callback)
            
            sync.registerCallback(depth_semantic_callback)

            # 遍历该位置的多个EE姿态
            for value in values:
                panda_controller.move_to_pose(value)

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

        # 如果遍历完毕没有找到目标，仍然返回收集到的语义标签信息
        semantic_labels_subscriber.unregister()
        depth_sub.unregister()
        semantic_sub.unregister()
        print(f"No target '{target_name}' found on bottom of {carrier_name}")
        save_knowledge_base_of_items(scene_name, room_name, carrier_name, target_name,
                                        semantic_labels_list)
        return False

    except rospy.ROSInterruptException:
        pass