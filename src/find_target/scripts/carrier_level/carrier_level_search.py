import rospy
import numpy as np
import json
import os
import tf2_ros
import tf.transformations as tft
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from motion_planning.panda_moveit_controller import PandaMoveItController
from language_model.create_prompt import create_prompt_classify_semantic_labels, create_prompt_sort_carrier_objects
from language_model.ollama_LLM import ollama_LLM 
from item_level.item_level_search import item_level_search

# ROS初始化和设置
rospy.init_node('depth_semantic_processor')

# 在全局位置定义
bridge = CvBridge()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 全局存储点云的字典
carrier_point_clouds = {}

def semantic_labels_callback(msg):
    """语义标签的回调函数，更新全局变量"""
    global semantic_labels_dict, semantic_labels_list
    try:
        data = json.loads(msg.data)
        for key, value in data.items():
            if key.isdigit():
                class_label = value.get("class", "")
                if class_label not in ["BACKGROUND", "UNLABELLED"]:
                    # 更新字典
                    if class_label not in semantic_labels_dict:
                        semantic_labels_dict[class_label] = int(key)
                    
                    # 更新列表
                    if class_label not in semantic_labels_list:
                        semantic_labels_list.append(class_label)
    except json.JSONDecodeError:
        rospy.logwarn("Failed to decode JSON message")

def depth_semantic_callback(depth_msg, semantic_msg):
    """
    处理深度和语义图像，提取载体的点云
    更高效的点云处理和坐标变换
    """
    global carrier_point_clouds, semantic_labels_dict, carrier_names_list

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

        # 使用NumPy加速处理
        for carrier_name in carrier_names_list:
            semantic_value = semantic_labels_dict[carrier_name]
            # 创建载体掩码
            carrier_mask = (semantic_image == semantic_value)
            
            # 快速过滤有效点
            valid_mask = (depth_image > 0) & (depth_image < 10) & carrier_mask
            
            # 使用NumPy快速获取坐标
            v, u = np.where(valid_mask)
            depths = depth_image[valid_mask]

            step = 100
            if len(v) > step:
                v = v[::step]
                u = u[::step]
                depths = depths[::step]

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
            if carrier_name in carrier_point_clouds:
                carrier_point_clouds[carrier_name] = carrier_point_clouds[carrier_name].union(new_points)
            else:
                carrier_point_clouds[carrier_name] = new_points
            
            # rospy.loginfo(f"Extracted point cloud for {carrier_name}: {len(new_points)} points")

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

def find_nearest_occupancy_point(target_point, occupancy_points_set):
    """
    在occupancy_points_set中找到距离目标点xy坐标最近的点
    
    Args:
        target_point: (x, y) 目标点的xy坐标
        occupancy_points_set: set of (x, y) 占用栅格点集
    
    Returns:
        (x, y): occupancy_points_set中最近的点
    """
    min_distance = float('inf')
    nearest_point = None
    
    for point in occupancy_points_set:
        distance = abs(target_point[0] - point[0]) + abs(target_point[1] - point[1])
        if distance < min_distance:
            min_distance = distance
            nearest_point = point
            
    return nearest_point

def get_connected_points(start_point, occupancy_points_set, resolution=0.05):
    """
    使用BFS找到所有与起始点相连的点
    
    Args:
        start_point: (x, y) 起始点
        occupancy_points_set: set of (x, y) 占用栅格点集
        resolution: float 栅格分辨率
    
    Returns:
        set: 与起始点相连的所有点集合
    """
    connected_points = set()
    queue = [start_point]
    visited = {start_point}

    # 四个方向的偏移（上下左右）
    directions = [
        (resolution, 0),
        (-resolution, 0),
        (0, resolution),
        (0, -resolution)
    ]

    while queue:
        current = queue.pop(0)
        connected_points.add(current)

        # 检查四个方向
        for dx, dy in directions:
            
            next_point = (
                round(current[0] + dx,2),
                round(current[1] + dy,2)
            )
            
            # 如果点在occupancy_set中且未访问过
            if next_point in occupancy_points_set and next_point not in visited:
                queue.append(next_point)    
                visited.add(next_point)            
        
    return connected_points

def process_carrier_points(carrier_point_clouds, occupancy_points_set):
    """
    处理每个carrier的点云，找到对应的占用区域
    
    Args:
        carrier_point_clouds: Dict[str, set of (x,y,z)]
        occupancy_points_set: set of (x,y)
    
    Returns:
        Dict[str, set]: 每个carrier对应的占用区域点集
    """
    carrier_areas = {}
    
    for carrier_name, points in carrier_point_clouds.items():
        # 计算xy平均值（忽略z坐标）
        points_xy = [(p[0], p[1]) for p in points]  # 提取xy坐标
        if points_xy:
            avg_x = sum(p[0] for p in points_xy) / len(points_xy)
            avg_y = sum(p[1] for p in points_xy) / len(points_xy)
        else:
            rospy.logwarn(f"No points found for {carrier_name}")
            continue
        
        # 在occupancy_points_set中找到最近的点
        nearest_point = find_nearest_occupancy_point((avg_x, avg_y), occupancy_points_set)
        
        if nearest_point is not None:
            # 获取所有连通的点
            connected_area = get_connected_points(nearest_point, occupancy_points_set)
            carrier_areas[carrier_name] = connected_area
        else:
            rospy.logwarn(f"No nearby occupancy point found for {carrier_name}")
    
    return carrier_areas

def save_points_to_npy(points, filename):
    # 将点集转换为NumPy数组
    points_array = np.array(list(points))
    np.save(filename, points_array)

def load_points_from_npy(filename):
    # 加载NumPy数组并转换为集合
    points_array = np.load(filename)
    return set(map(tuple, points_array))

# def save_knowledge_base_of_carrier(scene_name, room_name, carrier_name,
#                     carrier_areas_path):
#     """
#     Save carrier information to a JSON file.

#     Args:
#         scene_name: Name of the scene (e.g., 'flat', 'office')
#         room_name: Name of the room
#         carrier_name: Name of the carrier
#         carrier_points_path: Path to carrier points
#         carrier_areas_path: Path to carrier areas
#     """
#     # 创建输出目录
#     output_dir = os.path.expanduser('~/ws_thesis_lzq/src/find_target/scripts/scene_level/previous_knowledge_base')
#     os.makedirs(output_dir, exist_ok=True)

#     # 构建输出文件路径
#     output_file = os.path.join(output_dir, 'saved_carriers_results.json')

#     # 准备要保存的数据
#     carrier_data = {
#         'scene_name': scene_name,
#         'room_name': room_name,
#         'carrier_areas_path': carrier_areas_path
#     }

#     try:
#         # 读取已存在的数据
#         if os.path.exists(output_file):
#             with open(output_file, 'r') as f:
#                 existing_data = json.load(f)
#         else:
#             existing_data = {}
        
#         existing_data[carrier_name] = carrier_data

#         # 写入JSON文件
#         with open(output_file, 'w') as f:
#             json.dump(existing_data, f, indent=4)

#         print(f":white_check_mark: Successfully saved carrier information to {output_file}")
#     except Exception as e:
#         raise IOError(f"Failed to write knowledge base: {e}")

def save_knowledge_base_of_carrier(scene_name, room_name, carrier_name,
                    carrier_points_path, carrier_areas_path):
    """
    Save carrier information to a JSON file.

    Args:
        scene_name: Name of the scene (e.g., 'flat', 'office')
        room_name: Name of the room
        carrier_name: Name of the carrier
        carrier_points_path: Path to carrier points
        carrier_areas_path: Path to carrier areas
    """
    # 创建输出目录
    output_dir = os.path.expanduser('~/ws_thesis_lzq/src/find_target/scripts/scene_level/previous_knowledge_base')
    os.makedirs(output_dir, exist_ok=True)

    # 构建输出文件路径
    output_file = os.path.join(output_dir, 'saved_carriers_results.json')

    # 准备要保存的数据
    carrier_data = {
        'scene_name': scene_name,
        'room_name': room_name,
        'carrier_points_path': carrier_points_path,
        'carrier_areas_path': carrier_areas_path
    }

    try:
        # 读取已存在的数据
        if os.path.exists(output_file):
            with open(output_file, 'r') as f:
                existing_data = json.load(f)
        else:
            existing_data = {}
        
        existing_data[carrier_name] = carrier_data

        # 写入JSON文件
        with open(output_file, 'w') as f:
            json.dump(existing_data, f, indent=4)

        print(f":white_check_mark: Successfully saved carrier information to {output_file}")
    except Exception as e:
        raise IOError(f"Failed to write knowledge base: {e}")

def carrier_level_search(target_name, scene_name, room_name):
    global semantic_labels_dict, carrier_point_clouds, semantic_labels_list, carrier_names_list

    # 初始化全局变量
    semantic_labels_dict = {}
    carrier_point_clouds = {}
    semantic_labels_list = []
    carrier_names_list = []

    panda_controller = PandaMoveItController()

    # 第一个joint pose
    joint_goal_first = [120, -4, 120, -4, 0, 80, 45]
    panda_controller.move_to_joint(joint_goal_first, is_degrees=True)

    # 订阅语义分割结果的主题
    semantic_labels_subscriber = rospy.Subscriber('/semantic_labels_hand', 
                                                String, 
                                                semantic_labels_callback)

    # 第二个joint pose
    joint_goal_second = [120, -4, 0, -4, 0, 80, 45]
    panda_controller.move_to_joint(joint_goal_second, is_degrees=True)

    # 完成后停止订阅语义分割结果
    semantic_labels_subscriber.unregister()

    # 输出接收到的语义标签
    print(f"\U0001F4A1 Received semantic labels: {semantic_labels_list}")

    # 使用语言模型来分析语义标签
    prompt = create_prompt_classify_semantic_labels(semantic_labels_list)
    ollama_LLM()
    carrier_names = ollama_LLM(prompt)
    print(carrier_names)
    carrier_names_list = list(set([carrier.strip() for carrier in carrier_names.split(',') if carrier.strip() in semantic_labels_list]))
    if "floor" in carrier_names_list:
        carrier_names_list.remove("floor")
    if "wall" in carrier_names_list:
        carrier_names_list.remove("wall")
    print(f"\U0001F4A1 Classified carriers: {carrier_names_list}")

    # 同步深度和语义图像订阅器
    depth_sub = message_filters.Subscriber('/depth_hand', Image)
    semantic_sub = message_filters.Subscriber('/sem_seg_hand', Image)
    sync = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, semantic_sub], 1, 5
    )
    sync.registerCallback(depth_semantic_callback)

    # 第三个joint pose
    joint_goal_third = [120, -4, 120, -4, 0, 80, 45]
    panda_controller.move_to_joint(joint_goal_third, is_degrees=True)

    # 停止订阅
    # sync.clear()
    depth_sub.unregister()
    semantic_sub.unregister()

    # load points
    save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}')
    os.makedirs(save_dir, exist_ok=True) # 确保目录存在
    occupancy_points_path = os.path.join(save_dir, f'{room_name}_occupancy_area_points.npy')
    occupancy_points_set = load_points_from_npy(occupancy_points_path)
    free_points_path = os.path.join(save_dir, f'{room_name}_free_area_points.npy')
    free_points_set = load_points_from_npy(free_points_path)
    footprint_points_path = os.path.join(save_dir, f'{room_name}_footprint_area_points.npy')
    footprint_points_set = load_points_from_npy(footprint_points_path)

    # 处理carrier点云，获取每个carrier的区域
    carrier_areas = process_carrier_points(carrier_point_clouds, occupancy_points_set)

    # save points
    save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}/{room_name}')
    os.makedirs(save_dir, exist_ok=True) # 确保目录存在
    for carrier_name, carrier_points in carrier_point_clouds.items():
        
        # carrier_areas_path = os.path.join(save_dir, f'{carrier_name}_area.npy')
        # save_points_to_npy(carrier_areas[carrier_name], carrier_areas_path)
        # save_knowledge_base_of_carrier(scene_name, room_name, carrier_name,
        #                             carrier_areas_path)

        # 保存carrier的点云（3D点）
        carrier_points_path = os.path.join(save_dir, f'{carrier_name}_point_cloud.npy')
        save_points_to_npy(carrier_points, carrier_points_path)
        
        # 保存carrier的区域点（2D点）
        if carrier_name in carrier_areas:
            carrier_areas_path = os.path.join(save_dir, f'{carrier_name}_area.npy')
            save_points_to_npy(carrier_areas[carrier_name], carrier_areas_path)
            save_knowledge_base_of_carrier(scene_name, room_name, carrier_name,
                                        carrier_points_path, carrier_areas_path)
        else:
            rospy.logwarn(f"No area found for carrier {carrier_name}")

    # 使用语言模型排序
    prompt = create_prompt_sort_carrier_objects(target_name, carrier_names_list)
    ollama_LLM()
    sorted_carriers = ollama_LLM(prompt)
    print(sorted_carriers)
    sorted_carrier_list = list(set([carrier.strip() for carrier in sorted_carriers.split(',') if carrier.strip() in semantic_labels_list]))
    print(f"Sorted carriers: {sorted_carrier_list}")

    # 遍历每个排序后的carrier，进入房间并执行任务
    for carrier_name in sorted_carrier_list:
        result = item_level_search(target_name, scene_name, room_name, carrier_name,
                          occupancy_points_set, free_points_set, footprint_points_set)
        if result:
            return result

    # 搜索完毕后未找到目标
    print(f"Search for {target_name} in {room_name} of {scene_name} completed, target not found.")
    return False