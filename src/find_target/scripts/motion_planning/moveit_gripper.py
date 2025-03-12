import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes

# 全局变量，确保只初始化一次
_gripper_group = None

def initialize_moveit():
    """
    全局初始化 MoveIt 环境
    只在第一次调用时执行
    """
    global _gripper_group
    # 只在节点未初始化时进行初始化
    if not rospy.get_name() or rospy.get_name() == rospy.NAME_RESOLVER_NONE:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_gripper_goal', anonymous=True)

    # 仅在第一次调用时创建 gripper_group
    if _gripper_group is None:
        # 只初始化 panda_hand 运动组
        _gripper_group = moveit_commander.MoveGroupCommander("panda_hand")
        
        # 设置规划参数
        _gripper_group.set_planning_time(5.0)
        _gripper_group.set_max_velocity_scaling_factor(0.5)
        _gripper_group.set_max_acceleration_scaling_factor(0.5)
    
    return _gripper_group

def move_to_gripper_goal(gripper_goal_position, timeout=5.0):
    """
    控制夹爪移动到指定位置
    
    :param gripper_goal_position: 夹爪目标位置 
    :param timeout: 规划超时时间
    :return: 是否成功执行
    """
    try:
        # 初始化 MoveIt 环境
        gripper_group = initialize_moveit()
        
        # 设置规划时间
        gripper_group.set_planning_time(timeout)
        
        # 对于Panda夹爪，通常使用具体的命名目标
        if gripper_goal_position == 0.0:
            gripper_group.set_named_target("close")
        elif gripper_goal_position == 1.0:
            gripper_group.set_named_target("open")
        else:
            # 如果需要精确控制，可以使用 set_joint_value_target
            current_joints = gripper_group.get_current_joint_values()
            target_joints = [gripper_goal_position] * len(current_joints)
            gripper_group.set_joint_value_target(target_joints)
        
        # 进行路径规划
        plan = gripper_group.plan()
        
        # 检查规划是否成功
        if not plan[0]:
            rospy.logerr("夹爪路径规划失败")
            return False
        
        # 执行规划
        success = gripper_group.execute(plan[0], wait=True)
        
        # 停止并清理目标
        gripper_group.stop()
        gripper_group.clear_pose_targets()
        
        if success:
            rospy.loginfo(f"成功执行夹爪目标: {gripper_goal_position}")
            return True
        else:
            rospy.logerr("执行夹爪动作失败")
            return False
    
    except Exception as e:
        rospy.logerr(f"夹爪控制时发生错误: {e}")
        return False

if __name__ == '__main__':
    try:
        # 第一次调用：关闭夹爪
        success1 = move_to_gripper_goal(0.0)  # 关闭夹爪

        # 第二次调用：打开夹爪
        success2 = move_to_gripper_goal(1.0)  # 打开夹爪

        if success1 and success2:
            rospy.loginfo("成功执行所有夹爪动作")
        else:
            rospy.logerr("无法执行所有夹爪动作")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS节点被中断")