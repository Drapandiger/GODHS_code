#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import math
import tf
from geometry_msgs.msg import Pose

# 全局变量，确保只初始化一次
_move_group = None

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    
    :param roll: 绕x轴旋转角度（弧度）
    :param pitch: 绕y轴旋转角度（弧度）
    :param yaw: 绕z轴旋转角度（弧度）
    :return: 四元数 [x, y, z, w]
    """
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def initialize_moveit():
    """
    全局初始化 MoveIt 环境
    只在第一次调用时执行
    """
    global _move_group
    
    try:
        # 只在节点未初始化时进行初始化
        if not rospy.get_name() or rospy.get_name() == rospy.NAME_RESOLVER_NONE:
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node('moveit_pose_goal', anonymous=True)

        # 仅在第一次调用时创建 move_group
        if _move_group is None:
            robot = moveit_commander.RobotCommander()
            group_name = "panda_arm"  # 替换为你的运动组名称
            _move_group = moveit_commander.MoveGroupCommander(group_name)

            # 设置规划参数
            _move_group.set_planning_time(5.0)
            _move_group.set_max_velocity_scaling_factor(0.5)
            _move_group.set_max_acceleration_scaling_factor(0.5)
            
            rospy.loginfo("MoveIt 环境初始化成功")
        
        return _move_group
    
    except Exception as e:
        rospy.logerr(f"MoveIt 环境初始化失败: {e}")
        return None

def moveit_pose_goal(pose_goal, timeout=30.0):
    """
    移动到指定目标位置
    
    :param pose_goal: 目标位置 
      - 四元数输入: [x, y, z, qx, qy, qz, qw]
      - 欧拉角输入: [x, y, z, roll, pitch, yaw]
    :param timeout: 等待执行完成的超时时间（秒）
    :return: 是否成功到达目标
    """
    try:
        # 初始化 MoveIt 环境
        move_group = initialize_moveit()
        
        if move_group is None:
            rospy.logerr("初始化 MoveIt 失败")
            return False

        # 创建 Pose 消息
        goal = Pose()
        goal.position.x = pose_goal[0]
        goal.position.y = pose_goal[1]
        goal.position.z = pose_goal[2]

        # 处理朝向：支持四元数和欧拉角两种输入
        if len(pose_goal) == 7:  # 四元数输入 [x, y, z, qx, qy, qz, qw]
            goal.orientation.x = pose_goal[3]
            goal.orientation.y = pose_goal[4]
            goal.orientation.z = pose_goal[5]
            goal.orientation.w = pose_goal[6]
            rospy.loginfo(f"使用四元数朝向: [{pose_goal[3]:.4f}, {pose_goal[4]:.4f}, {pose_goal[5]:.4f}, {pose_goal[6]:.4f}]")
        
        elif len(pose_goal) == 6:  # 欧拉角输入 [x, y, z, roll, pitch, yaw]
            # 判断是角度还是弧度并转换
            roll, pitch, yaw = pose_goal[3:6]
            if abs(max(roll, pitch, yaw)) > 2 * math.pi:  # 认为是角度
                roll = math.radians(roll)
                pitch = math.radians(pitch)
                yaw = math.radians(yaw)
            
            # 转换为四元数
            quaternion = euler_to_quaternion(roll, pitch, yaw)
            goal.orientation.x = quaternion[0]
            goal.orientation.y = quaternion[1]
            goal.orientation.z = quaternion[2]
            goal.orientation.w = quaternion[3]
            rospy.loginfo(f"使用欧拉角: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
        
        else:
            rospy.logerr("无效的位姿输入")
            return False

        # 设置目标位姿
        rospy.loginfo(f"设置目标位置: x={goal.position.x:.4f}, y={goal.position.y:.4f}, z={goal.position.z:.4f}")
        move_group.set_pose_target(goal)

        # 进行路径规划
        plan = move_group.plan()

        # 检查规划是否成功
        if not plan[0]:
            rospy.logerr("路径规划失败")
            return False

        # 执行规划
        success = move_group.execute(plan[0], wait=True)

        if success:
            rospy.loginfo("成功到达目标位置")
        else:
            rospy.logerr("执行路径失败")

        # 清理目标
        move_group.stop()
        move_group.clear_pose_targets()

        return success

    except Exception as e:
        rospy.logerr(f"移动到目标位置时发生错误: {e}")
        return False

if __name__ == '__main__':
    try:
        # 使用四元数输入
        pose_goal1 = [0.4, 0.1, 0.4, 0.0, 0.0, 0.0, 1.0]  # [x, y, z, qx, qy, qz, qw]
        success1 = moveit_pose_goal(pose_goal1)

        # 使用欧拉角输入（角度）
        pose_goal2 = [0.5, 0.2, 0.5, 0, 0, 90]  # [x, y, z, roll, pitch, yaw]
        success2 = moveit_pose_goal(pose_goal2)

        # 使用欧拉角输入（弧度）
        pose_goal3 = [0.6, 0.3, 0.6, 0, 0, math.pi/2]  # [x, y, z, roll, pitch, yaw]
        success3 = moveit_pose_goal(pose_goal3)

        if success1 and success2 and success3:
            rospy.loginfo("成功移动到所有目标位置")
        else:
            rospy.logerr("无法移动到所有目标位置")
            
    except rospy.ROSInterruptException:
        rospy.logerr("程序被中断")