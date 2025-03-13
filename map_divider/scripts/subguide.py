import yaml
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy

# 读取YAML文件
with open('submaps.yaml', 'r') as file:
    data = yaml.safe_load(file)

# 将区域角点转换为多边形的点列表
def convert_to_polygon(corners):
    return [(corner['x'], corner['y']) for corner in corners]

# 判断点是否在多边形内
def point_in_polygon(point, polygon):
    from matplotlib.path import Path
    path = Path(polygon)
    return path.contains_point(point)

# 检查机器人是否进入某个区域
def check_robot_in_area(robot_position, areas):
    for area in areas:
        polygon = convert_to_polygon(area['corners'])
        if point_in_polygon(robot_position, polygon):
            return area['name']
    return None

# 回调函数，当接收到/odom数据时更新机器人的位置
def odom_callback(data):
    global robot_position
    # 从 odom 消息中提取机器人的位置
    robot_position = (data.pose.pose.position.x, data.pose.pose.position.y)

# 获取机器人的 odom 数据
def get_robot_odom():
    global robot_position
    return robot_position

# ROS 主函数，订阅 /odom 话题
def ros_main():
    rospy.init_node('robot_odom_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # 循环等待回调函数处理 odom 数据
    rospy.spin()


# 当机器人进入某个区域时显示该区域的平面图并绘制机器人位置
def show_robot_in_area(robot_position, area_name):
    img_path = f"{area_name}.png"
    
    # 读取平面图
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法找到区域 {area_name} 的平面图: {img_path}")
        return

    # 转换为 RGB 格式，以便用 matplotlib 显示
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # 显示平面图
    plt.imshow(img_rgb)
    plt.title(f"Robot in {area_name}")

    # 在图上绘制机器人的位置
    plt.scatter(robot_position[0], robot_position[1], color='red', s=100, label='Robot')
    plt.legend()

    # 显示图像
    plt.show()

# 主函数，监听机器人位置并调出相应区域的子图
def main():
    
    try:
        ros_main()
    except rospy.ROSInterruptException:
        pass

    # 获取机器人当前的位置
    robot_position = get_robot_odom()

    # 检查机器人是否进入某个区域
    area_name = check_robot_in_area(robot_position, data['areas'])

    if area_name:
        print(f"机器人进入区域: {area_name}")
        show_robot_in_area(robot_position, area_name)
    else:
        print("机器人不在任何已知区域内")

# 运行主函数
if __name__ == "__main__":
    main()
