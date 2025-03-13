#!/usr/bin/env python

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SummitXLMovebaseController:
    def __init__(self, move_base_topic='move_base', frame_id='map'):
        """
        初始化SummitXL移动控制器
        
        :param move_base_topic: move_base动作服务器的话题名称
        :param frame_id: 目标位置的参考坐标系
        """
        try:
            # 初始化ROS节点
            if not rospy.core.is_initialized():
                rospy.init_node('move_goal', anonymous=True)

            # 创建move_base动作客户端
            self.move_base_client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)
            
            # 等待move_base服务器连接
            rospy.loginfo(f"等待 {move_base_topic} 连接...")
            if not self.move_base_client.wait_for_server(rospy.Duration(10000.0)):
                rospy.logerr(f"无法连接到 {move_base_topic} 服务器")
                raise Exception("Move base服务器连接失败")

            # 设置默认坐标系
            self.frame_id = frame_id

            rospy.loginfo("SummitXL移动控制器初始化成功")

        except Exception as e:
            rospy.logerr(f"SummitXL控制器初始化失败: {e}")
            raise

    def euler_to_quaternion(self, yaw):
        """
        将偏航角转换为四元数
        
        :param yaw: 偏航角（弧度）
        :return: 四元数 [x, y, z, w]
        """
        return tf.transformations.quaternion_from_euler(0, 0, yaw)

    def move_to_pose(self, pose_goal, timeout=120.0):
        """
        移动到指定目标位置
        
        :param pose_goal: 目标位置 
          - 四元数输入: [x, y, qx, qy, qz, qw]
          - 偏航角输入: [x, y, yaw] (角度或弧度)
        :param timeout: 等待到达目标的最长时间（秒）
        :return: 是否成功到达目标
        """
        try:
            # 创建目标位置的消息
            goal = MoveBaseGoal()

            # 设置目标坐标系
            goal.target_pose.header.frame_id = self.frame_id
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
                quaternion = self.euler_to_quaternion(yaw)
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
            self.move_base_client.send_goal(goal)

            # 等待机器人到达目标位置，带超时
            success = self.move_base_client.wait_for_result(rospy.Duration(timeout))
            
            if success and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("目标位置到达成功！")
                return True
            else:
                rospy.logerr("目标位置到达失败！")
                return False

        except Exception as e:
            rospy.logerr(f"导航时发生错误: {e}")
            return False

    def cancel_goal(self):
        """
        取消当前导航目标
        """
        try:
            self.move_base_client.cancel_goal()
            rospy.loginfo("已取消当前导航目标")
        except Exception as e:
            rospy.logerr(f"取消导航目标时发生错误: {e}")

    def shutdown(self):
        """
        关闭移动控制器
        """
        rospy.loginfo("关闭SummitXL移动控制器")
        self.cancel_goal()
        rospy.signal_shutdown("SummitXL Controller Shutdown")

def main():
    try:
        from motion_planning.summitxl_movebase_controller import SummitXLMovebaseController
        # 创建SummitXLMovebaseController实例
        summit_controller = SummitXLMovebaseController()

        # 第一次导航：使用四元数输入
        pose_goal1 = [1.0, 2.0, 0.0, 0.0, 0.7071, 0.7071]  # 四元数输入
        success1 = summit_controller.move_to_pose(pose_goal1)

        # 第二次导航：使用偏航角输入（角度）
        pose_goal2 = [3.0, 4.0, 45]  # 偏航角输入（角度）
        success2 = summit_controller.move_to_pose(pose_goal2)

        # 第三次导航：使用偏航角输入（弧度）
        pose_goal3 = [5.0, 6.0, math.pi/4]  # 偏航角输入（弧度）
        success3 = summit_controller.move_to_pose(pose_goal3)

        if success1 and success2 and success3:
            rospy.loginfo("成功导航到所有目标位置")
        else:
            rospy.logerr("无法导航到所有目标位置")

    except rospy.ROSInterruptException:
        rospy.loginfo("程序中断")
    finally:
        # 确保关闭控制器
        if 'summit_controller' in locals():
            summit_controller.shutdown()

if __name__ == '__main__':
    main()