#!/usr/bin/env python

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

# 全局变量，确保只初始化一次客户端
_move_base_client = None

def euler_to_quaternion(yaw):
    """
    将偏航角转换为四元数
    
    :param yaw: 偏航角（弧度）
    :return: 四元数 [x, y, z, w]
    """
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return quaternion

def initialize_move_base_client():
    """
    全局初始化 move_base 客户端
    只在第一次调用时执行
    """
    global _move_base_client
    
    # 仅在节点未初始化时进行初始化
    if not rospy.get_name() or rospy.get_name() == rospy.NAME_RESOLVER_NONE:
        rospy.init_node('move_to_goal_node', anonymous=True)
    
    # 仅在第一次调用时创建客户端
    if _move_base_client is None:
        _move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 等待move_base的连接，增加超时处理
        rospy.loginfo("等待move_base连接...")
        if not _move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("无法连接到 move_base 服务器")
            return None
    
    return _move_base_client

def move_base_pose_goal(pose_goal, timeout=120.0):
    """
    移动到指定目标位置
    
    :param pose_goal: 目标位置 
      - 四元数输入: [x, y, qx, qy, qz, qw]
      - 偏航角输入: [x, y, yaw] (角度或弧度)
    :param timeout: 等待到达目标的最长时间（秒）
    :return: 是否成功到达目标
    """
    try:
        # 初始化 move_base 客户端
        client = initialize_move_base_client()
        
        if client is None:
            rospy.logerr("初始化 move_base 客户端失败")
            return False

        # 创建目标位置的消息
        goal = MoveBaseGoal()

        # 设置目标坐标系
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置 (x, y)
        goal.target_pose.pose.position.x = pose_goal[0]
        goal.target_pose.pose.position.y = pose_goal[1]

        # 处理朝向：支持四元数和偏航角两种输入
        if len(pose_goal) == 6:  # 四元数输入
            goal.target_pose.pose.orientation.x = pose_goal[2]
            goal.target_pose.pose.orientation.y = pose_goal[3]
            goal.target_pose.pose.orientation.z = pose_goal[4]
            goal.target_pose.pose.orientation.w = pose_goal[5]
            rospy.loginfo(f"使用四元数朝向: [{pose_goal[2]}, {pose_goal[3]}, {pose_goal[4]}, {pose_goal[5]}]")
        
        elif len(pose_goal) == 3:  # 偏航角输入（支持角度和弧度）
            # 判断是角度还是弧度
            yaw = pose_goal[2]
            if abs(yaw) > 2 * math.pi:  # 认为是角度
                yaw = math.radians(yaw)
            
            # 转换为四元数
            quaternion = euler_to_quaternion(yaw)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
            rospy.loginfo(f"使用偏航角: {math.degrees(yaw)}°")
        
        else:
            rospy.logerr("无效的位姿输入")
            return False

        # 发送目标
        rospy.loginfo(f"发送目标位置: x={pose_goal[0]}, y={pose_goal[1]}")
        client.send_goal(goal)

        # 等待机器人到达目标位置，带超时
        success = client.wait_for_result(rospy.Duration(timeout))

        if success and client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("目标位置到达成功！")
            return True
        else:
            rospy.logerr("目标位置到达失败！")
            return False

    except Exception as e:
        rospy.logerr(f"导航时发生错误: {e}")
        return False

if __name__ == '__main__':
    try:
        # 第一次导航：使用四元数输入
        pose_goal1 = [1.0, 2.0, 0.0, 0.0, 0.7071, 0.7071]  # 四元数输入
        success1 = move_base_pose_goal(pose_goal1)

        # 第二次导航：使用偏航角输入（角度）
        pose_goal2 = [3.0, 4.0, 45]  # 偏航角输入（角度）
        success2 = move_base_pose_goal(pose_goal2)

        # 第三次导航：使用偏航角输入（弧度）
        pose_goal3 = [5.0, 6.0, math.pi/4]  # 偏航角输入（弧度）
        success3 = move_base_pose_goal(pose_goal3)

        if success1 and success2 and success3:
            rospy.loginfo("成功导航到所有目标位置")
        else:
            rospy.logerr("无法导航到所有目标位置")

    except rospy.ROSInterruptException:
        rospy.loginfo("程序中断")