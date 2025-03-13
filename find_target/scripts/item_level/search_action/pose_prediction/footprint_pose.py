from item_level.search_action.pose_prediction.footprint_darko import footprint_darko
from item_level.search_action.pose_prediction.IK_validation import IK_Validation
import math

def footprint_pose(EE_poses,
                   footprint_area_occupancy, carrier_area_occupancy,
                   resolution = 0.05):
    """
    Parameters:
        EE_poses: List[List[float]] - 每个元素为 [x, y, z, roll, pitch, yaw]
        footprint_area_occupancy: List[List[float]] - 每个元素为 [x, y]
        carrier_area_occupancy: List[List[float]] - 每个元素为 [x, y]
        resolution: float - 分辨率
    Returns:
        candidate_footprint_ee_moving: Dict[List[float], List[List[float]]]
        - key: [x, y, yaw] 代表足部位姿
        - value: List[[x, y, z, roll, pitch, yaw]] 代表对应的末端执行器位姿列表
    """
    # Initialize dictionaries to store EE_pose to footprint_candidate_poses and footprint_candidate_pose to EE_poses
    ee_footprint_mapping = {}  # Dict[List[float], List[List[float]]]
    footprint_ee_mapping = {}  # Dict[List[float], List[List[float]]]
    full_valid_footprint_poses = set()  # Set[List[float]]

    # Calculate footprint occupancy for each EE_pose
    for EE_pose in EE_poses:
        footprint_candidate_poses = footprint_darko(EE_pose)  # Returns List[List[float]], each [x, y, yaw]
        # Filter candidate poses based on footprint_area_occupancy
        valid_footprint_candidate_poses = [
            pose for pose in footprint_candidate_poses
            if (pose[0], pose[1]) in footprint_area_occupancy
        ]
        ee_footprint_mapping[tuple(EE_pose)] = valid_footprint_candidate_poses
        # Add valid poses to full_valid_footprint_area
        for pose in valid_footprint_candidate_poses:
            full_valid_footprint_poses.add(tuple(pose))
    print("footprint_pose 1")
    # Convert full_valid_footprint_pose to a list of lists
    full_valid_footprint_poses = [list(pose) for pose in full_valid_footprint_poses]

    # Define directions: primary and diagonal
    primary_directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    diagonal_directions = [(1, 1), (-1, -1), (1, -1), (-1, 1)]
    direction_degrees = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): 270,
                        (1, 1): 45, (-1, -1): 225, (1, -1): 315, (-1, 1): 135}
    full_valid_footprint_normal_directions = {}  # Dict[List[float], Dict[str, Union[float, Tuple[float, float]]]]
    full_valid_footprint_tangential_directions = {}  # Dict[List[float], Dict[str, Union[float, Tuple[float, float]]]]
    print("footprint_pose 2")
    for pose in full_valid_footprint_poses:
        x, y = pose[0], pose[1]
        found_direction = False

        # Search in increasing concentric layers
        layer = 1
        while not found_direction:
            # First, search in primary directions
            for (dx, dy) in primary_directions:
                new_x = x + layer * dx * resolution
                new_y = y + layer * dy * resolution
                if [new_x, new_y] in carrier_area_occupancy:
                    normal_direction = direction_degrees[(dx, dy)]
                    full_valid_footprint_normal_directions[tuple(pose)] = {'degree': normal_direction, 'direction': (dx, dy)}
                    tangential_direction = (normal_direction + 90) % 360
                    tangential_dx, tangential_dy = -dy, dx
                    full_valid_footprint_tangential_directions[tuple(pose)] = {'degree': tangential_direction, 'direction': (tangential_dx, tangential_dy)}
                    found_direction = True
                    break

            # If no direction found, then search in diagonal directions
            if not found_direction:
                for (dx, dy) in diagonal_directions:
                    new_x = x + layer * dx * resolution
                    new_y = y + layer * dy * resolution
                    if [new_x, new_y] in carrier_area_occupancy:
                        normal_direction = direction_degrees[(dx, dy)]
                        full_valid_footprint_normal_directions[tuple(pose)] = {'degree': normal_direction, 'direction': (dx, dy)}
                        tangential_direction = (normal_direction + 90) % 360
                        tangential_dx, tangential_dy = -dy, dx
                        full_valid_footprint_tangential_directions[tuple(pose)] = {'degree': tangential_direction, 'direction': (tangential_dx, tangential_dy)}
                        found_direction = True
                        break

            layer += 1
    print("footprint_pose 3")
    # Build footprint_ee_mapping from ee_footprint_mapping
    for ee_pose, footprint_poses in ee_footprint_mapping.items():
        for footprint_pose in footprint_poses:
            footprint_pose_tuple = tuple(footprint_pose)
            if footprint_pose_tuple not in footprint_ee_mapping:
                footprint_ee_mapping[footprint_pose_tuple] = []
            if list(ee_pose) not in footprint_ee_mapping[footprint_pose_tuple]:
                footprint_ee_mapping[footprint_pose_tuple].append(list(ee_pose))
    print("footprint_pose 4")
    full_valid_footprint_delta_directions = {}  # Dict[List[float], float]

    for pose, tangential_info in full_valid_footprint_tangential_directions.items():
        # Use yaw angle directly from the pose
        pose_degree = math.degrees(pose[2]) % 360
        tangential_degree = tangential_info['degree']
        delta_degree = (tangential_degree - pose_degree) % 360
        full_valid_footprint_delta_directions[pose] = delta_degree

    # Find the pose with the minimum delta angle
    min_delta_angle = min(full_valid_footprint_delta_directions.values())
    min_delta_poses = [pose for pose, angle in full_valid_footprint_delta_directions.items() if angle == min_delta_angle]
    print("footprint_pose 5")
    # Filter footprint_ee_mapping to only include poses with the minimum delta angle
    filtered_footprint_ee_mapping = {
        pose: ee_poses
        for pose, ee_poses in footprint_ee_mapping.items()
        if pose in min_delta_poses
    }
    print("footprint_pose 6")
    # Find the footprint pose with the maximum number of ee poses
    max_ee_pose_footprint = max(filtered_footprint_ee_mapping, key=lambda pose: len(filtered_footprint_ee_mapping[pose]))
    candidate_footprint_ee_mapping = {
        max_ee_pose_footprint: filtered_footprint_ee_mapping[max_ee_pose_footprint]
    }
    print("footprint_pose 7")
    # Remove the selected footprint pose from filtered_footprint_ee_mapping
    filtered_footprint_ee_mapping.pop(max_ee_pose_footprint)

    candidate_footprint_ee_moving = {}  # Dict[List[float], List[List[float]]]
    solver = IK_Validation()
    print("footprint_pose 8")
    while True:
        # Step 1: Select the footprint poses with the smallest delta angle
        min_delta_angle = min(full_valid_footprint_delta_directions.values())
        min_delta_footprints = [footprint for footprint, delta in full_valid_footprint_delta_directions.items() if delta == min_delta_angle]

        # Step 2: Find the footprint pose with the maximum number of ee poses
        max_ee_footprint = None
        max_ee_count = -1
        for footprint in min_delta_footprints:
            ee_poses = footprint_ee_mapping.get(footprint, [])
            if len(ee_poses) > max_ee_count:
                max_ee_footprint = footprint
                max_ee_count = len(ee_poses)

        if max_ee_footprint is None:
            break

        # Step 3: Add the selected footprint pose to the candidate dictionary
        candidate_footprint_ee_moving[list(max_ee_footprint)] = []

        # Step 4: Remove the selected footprint pose from footprint_ee_mapping
        selected_ee_poses = {tuple(ee) for ee in footprint_ee_mapping.pop(max_ee_footprint, [])}
        for ee_pose in selected_ee_poses:
            if solver.validate_ik_solution(list(max_ee_footprint), list(ee_pose)):
                candidate_footprint_ee_moving[list(max_ee_footprint)].append(list(ee_pose))
            else:
                selected_ee_poses.remove(ee_pose)

        # Step 5: Remove ee poses that are repeated in other footprint poses
        footprints_to_remove = []
        for footprint, ee_poses in footprint_ee_mapping.items():
            footprint_ee_mapping[footprint] = [ee for ee in ee_poses if tuple(ee) not in selected_ee_poses]
            # Mark footprint for removal if its ee pose list is empty
            if not footprint_ee_mapping[footprint]:
                footprints_to_remove.append(footprint)

        # Step 6: Remove footprints with empty ee pose lists
        for footprint in footprints_to_remove:
            del footprint_ee_mapping[footprint]
            del full_valid_footprint_delta_directions[footprint]

        # Step 7: Remove the selected footprint pose from full_valid_footprint_delta_directions
        del full_valid_footprint_delta_directions[max_ee_footprint]

        # Stop if there are no remaining footprint poses with ee poses
        if not any(footprint_ee_mapping.values()):
            break
        
    return candidate_footprint_ee_moving