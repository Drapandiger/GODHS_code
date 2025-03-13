#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import math
import tf
from geometry_msgs.msg import Pose

class PandaMoveItController:
    def __init__(self, move_group_name="panda_manipulator", gripper_group_name="panda_hand"):
        """
        初始化 MoveIt 控制器
        
        :param group_name: 机器人运动组名称
        """
        try:
            # 初始化 MoveIt 和 ROS 节点
            moveit_commander.roscpp_initialize(sys.argv)
            if not rospy.core.is_initialized():
                rospy.init_node('move_goal', anonymous=True)

            # 创建机器人和运动组对象
            self.robot = moveit_commander.RobotCommander()
            self.move_group = moveit_commander.MoveGroupCommander(move_group_name)
            self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

            # 配置运动参数
            # self.move_group.set_planning_time(5.0)
            # self.move_group.set_max_velocity_scaling_factor(0.5)
            # self.move_group.set_max_acceleration_scaling_factor(0.5)

            rospy.loginfo(f"MoveIt 控制器初始化成功，运动组: {move_group_name}, 抓取组: {gripper_group_name}")

        except Exception as e:
            rospy.logerr(f"MoveIt 控制器初始化失败: {e}")
            raise

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        
        :param roll: 绕x轴旋转角度（弧度）
        :param pitch: 绕y轴旋转角度（弧度）
        :param yaw: 绕z轴旋转角度（弧度）
        :return: 四元数 [x, y, z, w]
        """
        return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    def move_to_pose(self, pose_goal):
        """
        移动到指定目标位置
        
        :param pose_goal: 目标位置 
          - 四元数输入: [x, y, z, qx, qy, qz, qw]
          - 欧拉角输入: [x, y, z, roll, pitch, yaw]
        :return: 是否成功到达目标
        """
        try:
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
                roll, pitch, yaw = pose_goal[3:6]
                if abs(max(abs(roll), abs(pitch), abs(yaw))) > 2 * math.pi:  # 转换角度到弧度
                    roll = math.radians(roll)
                    pitch = math.radians(pitch)
                    yaw = math.radians(yaw)
                quaternion = self.euler_to_quaternion(roll, pitch, yaw)
                
                goal.orientation.x = quaternion[0]
                goal.orientation.y = quaternion[1]
                goal.orientation.z = quaternion[2]
                goal.orientation.w = quaternion[3]
                rospy.loginfo(f"使用欧拉角: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°,quaternion=[{quaternion[0]:.4f}, {quaternion[1]:.4f}, {quaternion[2]:.4f}, {quaternion[3]:.4f}]")
            
            else:
                rospy.logerr("无效的位姿输入")
                return False

            # 设置目标位姿
            self.move_group.set_pose_target(goal)
            rospy.loginfo(f"设置目标位置: x={goal.position.x:.4f}, y={goal.position.y:.4f}, z={goal.position.z:.4f}")

            # 执行路径规划和动作
            success = self.move_group.go(wait=True)
            if success:
                rospy.loginfo("成功到达目标位置")
            else:
                rospy.logerr("执行路径失败")

            # 停止后清除目标
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            return success

        except Exception as e:
            rospy.logerr(f"移动到目标位置时发生错误: {e}")
            return False

    def move_to_joint(self, joint_goal, is_degrees=False):
        """
        移动到指定关节角度
        
        :param joint_goal: 目标关节角度列表
        :param is_degrees: 是否为角度输入，默认为 False
        :return: 是否成功移动
        """
        try:
            # 如果是角度输入，将其转换为弧度
            if is_degrees:
                joint_goal = [math.radians(angle) for angle in joint_goal]
            
            # We get the joint values from the group and change some of the values:
            goal = self.move_group.get_current_joint_values()
            goal[0] = joint_goal[0]
            goal[1] = joint_goal[1]
            goal[2] = joint_goal[2]
            goal[3] = joint_goal[3]
            goal[4] = joint_goal[4]
            goal[5] = joint_goal[5]
            goal[6] = joint_goal[6]

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

            return True
        
        except Exception as e:
            rospy.logerr(f"移动到关节目标时发生错误: {e}")
            return False
        
    def move_to_gripper(self, gripper_goal):
        """
        控制夹爪移动到指定位置
        
        :param gripper_goal_position: 夹爪目标位置 
        :param timeout: 规划超时时间
        :return: 是否成功执行
        """
        try:

            if gripper_goal == False:
                gripper_action = "open"
            else:
                gripper_action = "close"
            
            self.gripper_group.set_named_target(gripper_action)
            
            self.gripper_group.go(wait=True)
            

            # Calling ``stop()`` ensures that there is no residual movement
            self.gripper_group.stop()

            return True
    
        except Exception as e:
            rospy.logerr(f"移动到夹爪目标时发生错误: {e}")
            return False
        
    def shutdown(self):
        """
        关闭 MoveIt commander
        """
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("MoveIt Controller Shutdown")

def main():
    try:
        from motion_planning.panda_moveit_controller import PandaMoveItController
        
        # 创建 PandaMoveItController 实例
        panda_controller = PandaMoveItController()

        # 使用四元数输入
        pose_goal1 = [0, -0.5, 0, 1.0, 0.0, 0.0, 0]  # [x, y, z, qx, qy, qz, qw]
        success1 = panda_controller.move_to_pose(pose_goal1)

        # 使用欧拉角输入（角度）
        pose_goal2 = [0.5, 0.2, 0.5, 0, 0, -90]  # [x, y, z, roll, pitch, yaw]
        success2 = panda_controller.move_to_pose(pose_goal2)

        # 使用欧拉角输入（弧度）
        pose_goal3 = [0.6, 0.3, 0.6, 0, 0, math.pi/2]  # [x, y, z, roll, pitch, yaw]
        success3 = panda_controller.move_to_pose(pose_goal3)

        if success1 and success2 and success3:
            rospy.loginfo("成功移动到所有目标位置")
        else:
            rospy.logerr("无法移动到所有目标位置")

    except rospy.ROSInterruptException:
        rospy.logerr("程序被中断")
    finally:
        # 确保关闭 MoveIt
        if 'panda_controller' in locals():
            panda_controller.shutdown()

if __name__ == '__main__':
    main()