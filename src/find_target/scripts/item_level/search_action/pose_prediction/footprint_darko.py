import math

def footprint_darko(EE_pose):
    radius_of_panda = 0.0825
    link1_of_panda = 0.333
    link2_of_panda = 0.316
    link3_of_panda = 0.384
    link4_of_panda = 0.088
    link5_of_panda = 0.107

    height_of_summit_xl_steel = 0.69
    bias_of_summit_xl_steel = 0.273

    def sample_points(radius, center_x, center_y, step=0.05):
        points = []
        x = -radius
        while x <= radius:
            y = -radius
            while y <= radius:
                if x ** 2 + y ** 2 <= radius ** 2:
                    points.append([center_x + x, center_y + y])
                y += step
            x += step
        return points

    def add_candidate_pose(center, point, angle_range, step=45):
        direction_vector = (point[0] - center[0], point[1] - center[1])
        angle = round(math.atan2(direction_vector[1], direction_vector[0]))
        candidate_poses = []
        
        for offset in range(-angle_range, angle_range + 1, 1):
            adjusted_angle = angle + math.radians(offset)
            if adjusted_angle % math.radians(45) == 0:
                # 创建包含位置和角度的候选姿态
                candidate_poses.append([point[0], point[1], adjusted_angle])
        return candidate_poses

    footprint_candidate_poses = set()

    # 根据原始代码逻辑保持不变，只修改元组为列表
    if height_of_summit_xl_steel + link1_of_panda - (link2_of_panda + link3_of_panda + link5_of_panda) * math.sin(math.radians(11)) < EE_pose[2] < height_of_summit_xl_steel + link1_of_panda + link2_of_panda + link3_of_panda + link4_of_panda:
        radius1 = math.sqrt((link2_of_panda + link3_of_panda + link4_of_panda) ** 2 - (EE_pose[2] - height_of_summit_xl_steel - link1_of_panda) ** 2)
        radius2 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2)
        radius3 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2 - 2 * bias_of_summit_xl_steel * radius1 * math.cos(math.radians(135)))
        radius4 = bias_of_summit_xl_steel + radius1

        center_x, center_y = EE_pose[0], EE_pose[1]
        for point in sample_points(radius4, center_x, center_y):
            distance = math.sqrt((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2)
            if distance <= radius1:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 180))
            elif radius1 < distance <= radius2:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 90))
            elif radius2 < distance <= radius3:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 45))
            elif radius3 < distance <= radius4:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 0))

    elif height_of_summit_xl_steel < EE_pose[2] < height_of_summit_xl_steel + link1_of_panda - (link2_of_panda + link3_of_panda + link5_of_panda) * math.sin(math.radians(11)):
        radius1 = link2_of_panda * math.cos(math.radians(11)) + math.sqrt((link3_of_panda + link5_of_panda) ** 2 - (height_of_summit_xl_steel + link1_of_panda + link2_of_panda * math.sin(math.radians(11)) - EE_pose[2]) ** 2)
        radius2 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2)
        radius3 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2 - 2 * bias_of_summit_xl_steel * radius1 * math.cos(math.radians(135)))
        radius4 = bias_of_summit_xl_steel + radius1

        center_x, center_y = EE_pose[0], EE_pose[1]
        for point in sample_points(radius4, center_x, center_y):
            distance = math.sqrt((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2)
            if distance <= radius1:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 180))
            elif radius1 < distance <= radius2:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 90))
            elif radius2 < distance <= radius3:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 45))
            elif radius3 < distance <= radius4:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 0))

    elif height_of_summit_xl_steel + link1_of_panda - radius_of_panda - link2_of_panda * math.sin(math.radians(11)) - link3_of_panda - link4_of_panda < EE_pose[2] < height_of_summit_xl_steel:
        radius1 = link2_of_panda * math.cos(math.radians(11)) + radius_of_panda + link5_of_panda
        radius2 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2)
        radius3 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius1 ** 2 - 2 * bias_of_summit_xl_steel * radius1 * math.cos(math.radians(135)))
        radius4 = bias_of_summit_xl_steel + radius1
        radius5 = link2_of_panda * math.cos(math.radians(11)) + math.sqrt((link3_of_panda + link5_of_panda) ** 2 - (height_of_summit_xl_steel + link1_of_panda + link2_of_panda * math.sin(math.radians(11)) - EE_pose[2]) ** 2)
        radius6 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius5 ** 2)
        radius7 = math.sqrt(bias_of_summit_xl_steel ** 2 + radius5 ** 2 - 2 * bias_of_summit_xl_steel * radius5 * math.cos(math.radians(135)))
        radius8 = bias_of_summit_xl_steel + radius5

        center_x, center_y = EE_pose[0], EE_pose[1]
        for point in sample_points(radius8, center_x, center_y):
            distance = math.sqrt((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2)
            direction_vector = (point[0] - center_x, point[1] - center_y)
            point_angle = math.atan2(direction_vector[1], direction_vector[0])
            
            # 使用EE_pose中的yaw角
            ee_angle = EE_pose[5]
            angle_diff = abs(point_angle - ee_angle)
            
            if angle_diff < math.radians(90) or angle_diff > math.radians(270):
                continue

            if distance == radius2 or distance == radius6:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 90))
            elif radius2 < distance <= radius3 or radius6 < distance <= radius7:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 45))
            elif radius3 < distance <= radius4 or radius7 < distance <= radius8:
                footprint_candidate_poses.update(add_candidate_pose([center_x, center_y], point, 0))
        
    else:
        return ValueError("EE pose is not valid")

    return list(footprint_candidate_poses)

# 示例用法
# EE_pose: [x, y, z, roll, pitch, yaw]
if __name__ == "__main__":
    example_ee_pose = [1.0, 2.0, 0.6, 0.1, 0.2, 0.5]
    result = footprint_darko(example_ee_pose)
