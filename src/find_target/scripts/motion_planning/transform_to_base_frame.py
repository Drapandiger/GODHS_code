import numpy as np
from math import radians, degrees

def transform_to_base_frame(chassis_world_pose, ee_world_pose):
    """
    将世界坐标系下的机械臂末端pose转换到机械臂底座坐标系下

    Args:
        chassis_world_pose: 底盘在世界坐标系下的位姿 [x, y, yaw]
        ee_world_pose: 末端在世界坐标系下的位姿 [x, y, z, roll, pitch, yaw]

    Returns:
        base_pose: 底座坐标系下末端的位姿 [x, y, z, roll, pitch, yaw]
    """
    # 1. 创建从世界坐标系到底盘坐标系的变换矩阵
    chassis_x, chassis_y, chassis_yaw = chassis_world_pose
    chassis_yaw_rad = radians(chassis_yaw)

    T_world_to_chassis = np.array([
        [np.cos(chassis_yaw_rad), -np.sin(chassis_yaw_rad), 0, chassis_x],
        [np.sin(chassis_yaw_rad), np.cos(chassis_yaw_rad), 0, chassis_y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # 2. 创建从底盘坐标系到机械臂底座坐标系的变换矩阵
    # 平移量: (-0.273, 0, 0.69)
    # 旋转量: (0, 0, 180)度
    base_rotation = radians(180)  # 转换为弧度
    T_chassis_to_base = np.array([
        [np.cos(base_rotation), -np.sin(base_rotation), 0, -0.273],
        [np.sin(base_rotation), np.cos(base_rotation), 0, 0],
        [0, 0, 1, 0.69],
        [0, 0, 0, 1]
    ])

    # 3. 创建末端在世界坐标系下的位姿矩阵
    ee_x, ee_y, ee_z, ee_roll, ee_pitch, ee_yaw = ee_world_pose
    ee_roll_rad = radians(ee_roll)
    ee_pitch_rad = radians(ee_pitch)
    ee_yaw_rad = radians(ee_yaw)

    # 按照roll-pitch-yaw顺序构建旋转矩阵
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(ee_roll_rad), -np.sin(ee_roll_rad)],
        [0, np.sin(ee_roll_rad), np.cos(ee_roll_rad)]
    ])

    Ry = np.array([
        [np.cos(ee_pitch_rad), 0, np.sin(ee_pitch_rad)],
        [0, 1, 0],
        [-np.sin(ee_pitch_rad), 0, np.cos(ee_pitch_rad)]
    ])

    Rz = np.array([
        [np.cos(ee_yaw_rad), -np.sin(ee_yaw_rad), 0],
        [np.sin(ee_yaw_rad), np.cos(ee_yaw_rad), 0],
        [0, 0, 1]
    ])

    R_world_ee = Rz @ Ry @ Rx

    T_world_ee = np.eye(4)
    T_world_ee[:3, :3] = R_world_ee
    T_world_ee[:3, 3] = [ee_x, ee_y, ee_z]

    # 4. 计算末端在底座坐标系下的位姿矩阵
    T_base_ee = np.linalg.inv(T_chassis_to_base) @ np.linalg.inv(T_world_to_chassis) @ T_world_ee

    # 5. 从变换矩阵中提取位置和欧拉角
    position = T_base_ee[:3, 3]

    # 从旋转矩阵提取欧拉角
    rotation_matrix = T_base_ee[:3, :3]
    pitch = degrees(np.arcsin(-rotation_matrix[2, 0]))

    if np.cos(pitch) != 0:
        roll = degrees(np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]))
        yaw = degrees(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
    else:
        # 万向锁情况的特殊处理
        roll = 0
        yaw = degrees(np.arctan2(-rotation_matrix[0, 1], rotation_matrix[1, 1]))

    return [position[0], position[1], position[2], roll, pitch, yaw]

# 使用示例
if __name__ == "__main__":
    # 底盘在世界坐标系下的位姿 [x, y, yaw]
    chassis_pose = [1.0, 2.0, 30.0]

    # 末端在世界坐标系下的位姿 [x, y, z, roll, pitch, yaw]
    ee_pose = [1.5, 2.5, 1.0, 10.0, 20.0, 30.0]

    # 计算底座坐标系下的末端位姿
    base_pose = transform_to_base_frame(chassis_pose, ee_pose)
    print("End-effector pose in base frame:", base_pose)