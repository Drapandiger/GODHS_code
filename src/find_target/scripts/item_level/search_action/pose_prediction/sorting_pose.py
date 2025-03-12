import numpy as np
import math

def sorting_pose(footprint_dict, carrier_area_occupancy, room_x_max, room_x_min, room_y_max, room_y_min):
    """
    对footprint_dict进行复杂排序处理
    
    Args:
    - footprint_dict: 字典，key为footprint pose，value为EE pose列表
    - carrier_area_occupancy: 网格点集
    - room_x_max, room_x_min, room_y_max, room_y_min: 房间边界
    
    Returns:
    - sorting_dict: 排序后的字典
    """
    # 第一步：扩展carrier_area_occupancy并去除重复点
    def expand_points(points):
        expanded = []
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        for point in points:
            expanded.extend([
                (round(point[0] + 0.05 * dx,2), round(point[1] + 0.05 * dy,2)) 
                for dx, dy in directions
            ])
        return list(set(expanded) - set(points))
    
    edge_initial_points = expand_points(carrier_area_occupancy)
    
    edge_points = [
        point for point in edge_initial_points
        if room_x_min <= point[0] <= room_x_max and room_y_min <= point[1] <= room_y_max
    ]
    
    edge_removel = len(edge_points) < len(edge_initial_points)
  
    # 计算carrier_area_occupancy的平均xy
    avg_x = np.mean([p[0] for p in carrier_area_occupancy])
    avg_y = np.mean([p[1] for p in carrier_area_occupancy])
    
    if edge_removel:
        # 处理边界情况
        # 计算四个边界的绝对距离
        distances = [
            abs(room_x_max - avg_x),  # 第1个
            abs(room_x_min - avg_x),  # 第2个
            abs(room_y_max - avg_y),  # 第3个
            abs(room_y_min - avg_y)   # 第4个
        ]

        # 找最小距离的索引
        min_distance_idx = distances.index(min(distances))

        # 根据最小距离索引调整avg_x或avg_y
        if min_distance_idx == 0:
            avg_x = room_x_max  # 第1个
        elif min_distance_idx == 1:
            avg_x = room_x_min  # 第2个
        elif min_distance_idx == 2:
            avg_y = room_y_max  # 第3个
        elif min_distance_idx == 3:
            avg_y = room_y_min  # 第4个

 
    # 创建edge_dict：存储每个边界点到平均点的角度
    edge_dict = {}
    for point in edge_points:
        angle = math.atan2(point[1] - avg_y, point[0] - avg_x)
        if min_distance_idx == 0:
            if angle < 0:
                angle += 2 * math.pi
        edge_dict[point] = angle
   
    # 为每个footprint找最近的边界点并计算角度
    footprint_angles = {}
    for footprint_pose, _ in footprint_dict.items():
        distances = [
            (abs(point[0] - footprint_pose[0]) + abs(point[1] - footprint_pose[1]), point) 
            for point in edge_points
        ]
        _, nearest_point = min(distances)
        angle = edge_dict[nearest_point]
        
        footprint_angles[footprint_pose] = angle
    
    # 按角度降序排序footprint
    sorted_footprints = sorted(
        footprint_angles.items(), 
        key=lambda x: x[1], 
        reverse=True
    )
    
    # 创建排序后的字典
    sorting_dict = {}
    for footprint_pose, _ in sorted_footprints:
        # 对每个pose的value按照z, y, x, yaw, pitch, roll排序
        sorted_ee_poses = sorted(
            footprint_dict[footprint_pose],
            key=lambda pose: (pose[2], pose[1], pose[0], pose[5], pose[4], pose[3])
        )
        sorting_dict[footprint_pose] = sorted_ee_poses
    
    return sorting_dict
