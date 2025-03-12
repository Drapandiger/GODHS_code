import sys
import math
import rospy
import moveit_commander

# 全局变量，确保只初始化一次
_move_group = None

def initialize_moveit():
    """
    全局初始化 MoveIt 环境
    只在第一次调用时执行
    """
    global _move_group
    # 只在节点未初始化时进行初始化
    if not rospy.get_name() or rospy.get_name() == rospy.NAME_RESOLVER_NONE:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_joint_goal', anonymous=True)

    # 仅在第一次调用时创建 move_group
    if _move_group is None:
        robot = moveit_commander.RobotCommander()
        group_name = "panda_arm"  # 替换为你的运动组名称
        _move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # 设置规划参数
        _move_group.set_planning_time(5.0)
        _move_group.set_max_velocity_scaling_factor(0.5)
        _move_group.set_max_acceleration_scaling_factor(0.5)
    
    return _move_group

def moveit_joint_goal(joint_goal, is_degrees=False):
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
        
        # 初始化 MoveIt 环境
        move_group = initialize_moveit()

        # 设置关节目标
        move_group.go(joint_goal, wait=True)

        # 确保机械臂到达目标位置
        move_group.stop()

        # 清除目标
        move_group.clear_pose_targets()

        return True
    
    except Exception as e:
        rospy.logerr(f"移动到关节目标时发生错误: {e}")
        return False

if __name__ == '__main__':
    try:
        # 第一次调用：设置关节目标（例如：[joint1, joint2, joint3, joint4, joint5, joint6]）
        joint_goal1 = [0.0, -90, 45, -45, 30, 0.0]  # 使用角度输入作为例子
        success1 = moveit_joint_goal(joint_goal1, is_degrees=True)

        # 第二次调用：另一个关节目标
        joint_goal2 = [45, -60, 30, -30, 15, 90]  # 另一个示例关节目标
        success2 = moveit_joint_goal(joint_goal2, is_degrees=True)

        if success1 and success2:
            rospy.loginfo("成功移动到所有关节目标")
        else:
            rospy.logerr("无法移动到所有关节目标")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS节点被中断")