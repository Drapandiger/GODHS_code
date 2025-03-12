import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def top_search(carrier_area_occupancy, obstacles_area_occupancy, 
                room_x_max, room_x_min, room_y_max, room_y_min, 
                carrier_height, 
                horizontal_fov = 60, vertical_fov = 60*9/16, grid_interval = 0.05):

    # 获取carrier_area_occupancy的xy坐标并扩展10个栅格
    initial_xy_positions = list(carrier_area_occupancy)  # 假设carrier_area_occupancy为占用位置的列表
    expanded_xy_positions = set()
    
    for pos in initial_xy_positions:
        x, y = pos
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                new_x = x + dx * grid_interval
                new_y = y + dy * grid_interval
                expanded_xy_positions.add((round(new_x,2), round(new_y,2)))

    # 转换集合为列表
    expanded_xy_positions = list(expanded_xy_positions)

    # 去除超出房间边界的xy坐标
    filtered_xy_positions = [
        (x, y) for (x, y) in expanded_xy_positions
        if room_x_min <= x <= room_x_max and room_y_min <= y <= room_y_max
    ]
    
    # 去除在障碍物区域内的位置，得到near_free_xy_positions
    obstacles_set = set(obstacles_area_occupancy)
    near_free_xy_positions = [
        (x, y) for (x, y) in filtered_xy_positions
        if (x, y) not in obstacles_set
    ]

    # 统计near_free_xy_positions在carrier_area_occupancy的上下左右四个方向上的数量
    direction_counts = {'up': 0, 'down': 0, 'left': 0, 'right': 0}
    carrier_set = set(carrier_area_occupancy)
    for x, y in near_free_xy_positions:
        # 统计上方向的数量
        for dy in range(1, 11):
            if (round(x,2), round(y + dy * grid_interval,2)) in carrier_set:
                direction_counts['up'] += 1
                break
        # 统计下方向的数量
        for dy in range(1, 11):
            if (round(x,2), round(y - dy * grid_interval,2)) in carrier_set:
                direction_counts['down'] += 1
                break
        # 统计左方向的数量
        for dx in range(1, 11):
            if (round(x - dx * grid_interval,2), round(y,2)) in carrier_set:
                direction_counts['left'] += 1
                break
        # 统计右方向的数量
        for dx in range(1, 11):
            if (round(x + dx * grid_interval,2), round(y,2)) in carrier_set:
                direction_counts['right'] += 1
                break

    # 计算沿z轴旋转的角度（取数量最多的方向）
    max_direction = max(direction_counts, key=direction_counts.get)
    if max_direction == 'up':
        yaw = 90
    elif max_direction == 'down':
        yaw = -90
    elif max_direction == 'left':
        yaw = 180
    elif max_direction == 'right':
        yaw = 0

    # set the pitch angle
    pitch_angles = [-120, -150]

    # 计算z的范围
    z_min_range = np.ceil(carrier_height / grid_interval) * grid_interval
    z_range = np.arange(z_min_range, z_min_range + 0.5 + grid_interval, grid_interval)
    z_range = z_range[(z_range >= 0.7) & (z_range <= 1.6)] 
    z_positions = np.round(z_range, 2)

    # 计算upper_surface
    upper_surface = [(x, y, carrier_height) for x, y in initial_xy_positions]
    print(carrier_height, z_min_range, np.arange(z_min_range, z_min_range + 0.5 + grid_interval, grid_interval), z_range)
    # 返回候选姿态列表（xy组合和z组合）
    candidate_poses = []
    for x, y in filtered_xy_positions:
        for z in z_positions:
            # default roll is 0
            candidate_poses.append((x, y, z, 0, -120, yaw))
            candidate_poses.append((x, y, z, 0, -150, yaw))
    print(f"共计计算了{len(candidate_poses)}种候选姿态")
    i = 0
    # 计算候选姿态的得分
    upper_surface_best_poses = []
    while upper_surface:
        i = i + 1
        print('top:', i)
        pose_scores = []
        for pose in candidate_poses:
            x, y, z, roll, pitch, yaw = pose

            # 计算朝向向量
            direction_vector = (math.cos(math.radians(pitch)) * math.cos(math.radians(yaw)), math.cos(math.radians(pitch)) * math.sin(math.radians(yaw)), math.sin(math.radians(pitch)))

            score = 0
            for ux, uy, uz in upper_surface:
                pointing_vector = (ux - x, uy - y, uz - z)
                pointing_vector_norm = np.linalg.norm(pointing_vector)
                direction_vector_norm = np.linalg.norm(direction_vector)
                if pointing_vector_norm == 0 or direction_vector_norm == 0:
                    continue

                # 计算横向夹角
                cos_theta_horizontal = (pointing_vector[0] * direction_vector[0] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                theta_horizontal = math.acos(np.clip(cos_theta_horizontal, -1.0, 1.0))

                # 计算纵向夹角
                cos_theta_vertical = (pointing_vector[1] * direction_vector[1] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                theta_vertical = math.acos(np.clip(cos_theta_vertical, -1.0, 1.0))

                if theta_horizontal < math.radians(horizontal_fov / 2) and theta_vertical < math.radians(vertical_fov / 2):
                    score += 1

            pose_scores.append((pose, score))

        # 找到得分最高的姿态
        best_pose, best_score = max(pose_scores, key=lambda x: x[1])
        if best_score == 0:
            break
        else:
            upper_surface_best_poses.append(list(best_pose))
            candidate_poses.remove(best_pose)

            # 删除对应的upper_surface中的栅格点
            x, y, z, roll, pitch, yaw = best_pose
            # 计算朝向向量
            direction_vector = (math.cos(math.radians(pitch)) * math.cos(math.radians(yaw)), math.cos(math.radians(pitch)) * math.sin(math.radians(yaw)), math.sin(math.radians(pitch)))

            for ux, uy, uz in upper_surface:
                pointing_vector = (ux - x, uy - y, uz - z)
                pointing_vector_norm = np.linalg.norm(pointing_vector)
                direction_vector_norm = np.linalg.norm(direction_vector)
                
                # 计算横向夹角
                cos_theta_horizontal = (pointing_vector[0] * direction_vector[0] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                theta_horizontal = math.acos(np.clip(cos_theta_horizontal, -1.0, 1.0))

                # 计算纵向夹角
                cos_theta_vertical = (pointing_vector[1] * direction_vector[1] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                theta_vertical = math.acos(np.clip(cos_theta_vertical, -1.0, 1.0))

                if theta_horizontal < math.radians(horizontal_fov / 2) and theta_vertical < math.radians(vertical_fov / 2):
                    upper_surface.remove((ux, uy, uz))
    print(len(upper_surface_best_poses))
    return upper_surface_best_poses

def sides_search(carrier_area_occupancy, obstacles_area_occupancy,
                    room_x_max, room_x_min, room_y_max, room_y_min, 
                    carrier_height,
                    horizontal_fov = 60, vertical_fov=60*9/16, grid_interval = 0.05):
    
    # 获取carrier_area_occupancy的xy坐标并扩展10个栅格
    initial_xy_positions = list(carrier_area_occupancy)  # 假设carrier_area_occupancy为占用位置的列表
    expanded_xy_positions = set()
    
    for pos in initial_xy_positions:
        x, y = pos
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                new_x = x + dx * grid_interval
                new_y = y + dy * grid_interval
                expanded_xy_positions.add((round(new_x,2), round(new_y,2)))

    # 转换集合为列表
    expanded_xy_positions = list(expanded_xy_positions)

    # 去除超出房间边界的xy坐标
    filtered_xy_positions = [
        (x, y) for (x, y) in expanded_xy_positions
        if room_x_min <= x <= room_x_max and room_y_min <= y <= room_y_max
    ]

    # 去除在障碍物区域内的位置，得到near_free_xy_positions
    obstacles_set = set(obstacles_area_occupancy)
    near_free_xy_positions = [
        (x, y) for (x, y) in filtered_xy_positions
        if (x, y) not in obstacles_set
    ]

    # 计算z的范围
    z_max_range = np.ceil(carrier_height / grid_interval) * grid_interval
    z_range = np.arange(0.5, z_max_range, grid_interval)
    z_positions = np.unique(z_range)
    z_positions = np.round(z_positions, 2)
    print(len(z_positions), len(near_free_xy_positions))
    # 定义side_surface_xy，寻找上下左右四个方向一个单位存在非占据的位置并加权向量
    side_surface_xy = []
    directions = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    for (x, y) in carrier_area_occupancy:
        direction_vector = (0, 0)
        for direction in directions:
            (dx, dy) = direction
            neighbor_pos = (round(x + dx * grid_interval, 2), round(y + dy * grid_interval, 2))
            if neighbor_pos not in carrier_area_occupancy:
                direction_vector = (direction_vector[0] - dx, direction_vector[1] - dy)
        if direction_vector != (0, 0):
            side_surface_xy.append((x, y, direction_vector))

    # 结合z_positions形成side_surface
    side_surface = []
    for (x, y, direction_vector) in side_surface_xy:
        for z in z_positions:
            side_surface.append((x, y, z, direction_vector))

    # 计算每个near_free_xy_positions的方向pose
    candidate_poses = []
    for (x, y) in near_free_xy_positions:
        direction_found = False
        directions = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0),
                        'up_left': (-1, 1), 'up_right': (1, 1), 'down_left': (-1, -1), 'down_right': (1, -1)}
        direction_angles = {
            'up': 90, 'down': -90, 'left': 180, 'right': 0,
            'up_left': 135, 'up_right': 45, 'down_left': -135, 'down_right': -45
        }

        # 判断上下左右四个方向
        for direction in ['up', 'down', 'left', 'right']:
            dx, dy = directions[direction]
            positions_in_direction = [
                (round(x + i * dx * grid_interval,2), round(y + i * dy * grid_interval,2))
                for i in range(1, 11)
            ]
            if any(pos in carrier_area_occupancy for pos in positions_in_direction):
                yaw = direction_angles[direction]
                direction_found = True
                break

        # 如果没有找到，再判断对角方向
        if not direction_found:
            for direction in ['up_left', 'up_right', 'down_left', 'down_right']:
                dx, dy = directions[direction]
                positions_in_direction = [
                    (round(x + i * dx * grid_interval,2), round(y + i * dy * grid_interval,2))
                    for i in range(1, 11)
                ]
                if any(pos in carrier_area_occupancy for pos in positions_in_direction):
                    yaw = direction_angles[direction]
                    direction_found = True
                    break

        # 如果找到方向，结合z_positions形成candidate_poses
        if direction_found:
            for z in z_positions:
                candidate_poses.append((x, y, z, 0, 0, yaw, (dx, dy, 0)))
    print(f"共计计算了{len(candidate_poses)}种候选姿态",near_free_xy_positions,z_positions)
    i = 0
    # 贪婪搜索，计算每个candidate_poses能够扫描多少个side_surface的位置
    side_surface_best_poses = []
    while side_surface:
        i = i + 1
        print('sides:', i)
        pose_scores = []
        for pose in candidate_poses:
            x, y, z, roll, yaw, pitch, direction = pose
            score = 0
            direction_vector = direction

            for sx, sy, sz, surface_direction in side_surface:
                if direction_vector[0] * surface_direction[0] + direction_vector[1] * surface_direction[1] > 0:
                    # 计算从候选点到side_surface的三维向量
                    pointing_vector = (sx - x, sy - y, sz - z)
                    pointing_vector_norm = np.linalg.norm(pointing_vector)
                    direction_vector_norm = np.linalg.norm(direction_vector)
                    if pointing_vector_norm == 0 or direction_vector_norm == 0:
                        continue

                    # 计算横向夹角
                    cos_theta_horizontal = (pointing_vector[0] * direction_vector[0] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                    theta_horizontal = math.acos(np.clip(cos_theta_horizontal, -1.0, 1.0))

                    # 计算纵向夹角
                    cos_theta_vertical = (pointing_vector[1] * direction_vector[1] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                    theta_vertical = math.acos(np.clip(cos_theta_vertical, -1.0, 1.0))

                    if theta_horizontal < math.radians(horizontal_fov / 2) and theta_vertical < math.radians(vertical_fov / 2):
                        score += 1

            pose_scores.append((pose, score))

        # 找到得分最高的姿态
        best_pose, best_score = max(pose_scores, key=lambda x: x[1])
        if best_score == 0:
            break
        else:
            side_surface_best_poses.append([best_pose[0], best_pose[1], best_pose[2], best_pose[3], best_pose[4], best_pose[5]])
            candidate_poses.remove(best_pose)

            # 删除对应的side_surface中的栅格点
            x, y, z, roll, pitch, yaw, direction = best_pose
            direction_vector = direction
            for sx, sy, sz, surface_direction in side_surface:
                if direction_vector[0] * surface_direction[0] + direction_vector[1] * surface_direction[1] > 0:
                    # 计算从候选点到side_surface的三维向量
                    pointing_vector = (sx - x, sy - y, sz - z)
                    pointing_vector_norm = np.linalg.norm(pointing_vector)
                    direction_vector_norm = np.linalg.norm(direction_vector)
                    if pointing_vector_norm == 0 or direction_vector_norm == 0:
                        continue

                    # 计算横向夹角
                    cos_theta_horizontal = (pointing_vector[0] * direction_vector[0] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                    theta_horizontal = math.acos(np.clip(cos_theta_horizontal, -1.0, 1.0))

                    # 计算纵向夹角
                    cos_theta_vertical = (pointing_vector[1] * direction_vector[1] + pointing_vector[2] * direction_vector[2]) / (pointing_vector_norm * direction_vector_norm)
                    theta_vertical = math.acos(np.clip(cos_theta_vertical, -1.0, 1.0))

                    if theta_horizontal < math.radians(horizontal_fov / 2) and theta_vertical < math.radians(vertical_fov / 2):
                        side_surface.remove((sx, sy, sz, surface_direction))
    print(len(side_surface_best_poses))
    return side_surface_best_poses

def bottom_search(carrier_area_occupancy, obstacles_area_occupancy,
                    room_x_max, room_x_min, room_y_max, room_y_min, 
                    carrier_height,
                    horizontal_fov = 60, vertical_fov = 60*9/16, grid_interval = 0.05
                    ):
    
    # 获取carrier_area_occupancy的xy坐标并扩展1个栅格
    initial_xy_positions = list(carrier_area_occupancy) 
    expanded_1_xy_positions = set()

    for pos in initial_xy_positions:
        x, y = pos
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                new_x = x + dx * grid_interval
                new_y = y + dy * grid_interval
                expanded_1_xy_positions.add((round(new_x,2), round(new_y,2)))

    # 转换集合为列表
    expanded_1_xy_positions = list(expanded_1_xy_positions)

    # 去除超出房间边界的xy坐标
    filtered_1_xy_positions = [
        (x, y) for (x, y) in expanded_1_xy_positions
        if room_x_min <= x <= room_x_max and room_y_min <= y <= room_y_max
    ]

    # 去除在障碍物区域内的位置，得到near_free_xy_positions
    obstacles_set = set(obstacles_area_occupancy)
    near_free_xy_positions = [
        (x, y) for (x, y) in filtered_1_xy_positions
        if (x, y) not in obstacles_set
    ]

    # 筛选出上下左右四个方向中连续19个栅格都不在障碍物区域内的位置，定义为candidate_positions
    candidate_positions = []
    directions = {
        'up': (0, 1),
        'down': (0, -1),
        'left': (-1, 0),
        'right': (1, 0)
    }

    for (x, y) in near_free_xy_positions:
        for direction, (dx, dy) in directions.items():
            is_candidate = True
            for step in range(1, 20):
                check_x = x + step * dx * grid_interval
                check_y = y + step * dy * grid_interval
                if (round(check_x,2), round(check_y,2)) in obstacles_set and not (room_x_min <= check_x <= room_x_max and room_y_min <= check_y <= room_y_max):
                    is_candidate = False
                    break
            if is_candidate:
                candidate_positions.append(((x, y), direction))
    
    # 选择尽可能少的candidate_positions以覆盖整个carrier_area_occupancy，定义为bottom_best_poses
    bottom_best_positions = []
    uncovered_positions = set(initial_xy_positions)

    while uncovered_positions:
        best_candidate = None
        best_covered = set()

        for candidate, direction in candidate_positions:
            x, y = candidate
            covered_positions = set()

            if direction == 'up':
                covered_positions = {(cx, cy) for (cx, cy) in uncovered_positions if cy < y}
            elif direction == 'down':
                covered_positions = {(cx, cy) for (cx, cy) in uncovered_positions if cy > y}
            elif direction == 'left':
                covered_positions = {(cx, cy) for (cx, cy) in uncovered_positions if cx > x}
            elif direction == 'right':
                covered_positions = {(cx, cy) for (cx, cy) in uncovered_positions if cx < x}

            if len(covered_positions) > len(best_covered):
                best_candidate = (candidate, direction)
                best_covered = covered_positions

        if best_candidate:
            bottom_best_positions.append(best_candidate)
            uncovered_positions -= best_covered
        else:
            break

    # 计算z的范围
    z_positions = 0.5
    z = np.round(z_positions, 2)

    # 组合bottom_best_positions和z_positions，并计算roll pitch yaw角度
    bottom_best_poses = []
    for (position, direction) in bottom_best_positions:
        x, y = position
        num_angles = int(np.ceil(180 / horizontal_fov))
        if direction == 'up':
            angles = np.linspace(-180 + horizontal_fov / 2, 0 - horizontal_fov / 2, num=num_angles)
        elif direction == 'down':
            angles = np.linspace(0 + horizontal_fov / 2, 180 - horizontal_fov / 2, num=num_angles)
        elif direction == 'left':
            angles = np.linspace(-90 + horizontal_fov / 2, 90 - horizontal_fov / 2, num=num_angles)
        elif direction == 'right':
            angles = np.linspace(90 + horizontal_fov / 2, 270 - horizontal_fov / 2, num=num_angles)

        for angle in angles:
            # 计算roll pitch yaw angle
            roll = 0
            pitch = 0
            yaw = np.deg2rad(angle)

            # 计算底部朝向
            bottom_best_poses.append([x, y, z, roll, pitch, yaw])
    print(len(bottom_best_poses))
    return bottom_best_poses
