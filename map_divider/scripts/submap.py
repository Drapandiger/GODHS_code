import yaml
import cv2
import numpy as np

# 读取YAML文件
with open('submaps.yaml', 'r') as file:
    data = yaml.safe_load(file)

# 获取地图信息
map_image_path = data['map']['image']
resolution = data['map']['resolution']
origin = data['map']['origin']

# 加载地图图像
map_image = cv2.imread(map_image_path)
if map_image is None:
    raise ValueError(f"无法加载地图图像: {map_image_path}")

# 将实际坐标转换为像素坐标的函数
def convert_to_pixel_coords(x, y, resolution, origin):
    px = int((x - origin[0]) / resolution)
    py = int(map_image.shape[0] - (y - origin[1]) / resolution)
    return px, py

# 定义保存子图的函数
def save_sub_image(image, corners, output_path):
    # 将每个角点转换为像素坐标
    x_coords = [convert_to_pixel_coords(corner['x'], corner['y'], resolution, origin)[0] for corner in corners]
    y_coords = [convert_to_pixel_coords(corner['x'], corner['y'], resolution, origin)[1] for corner in corners]

    # 获取最小和最大x、y坐标
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    # 裁剪图像
    sub_image = image[min_y:max_y, min_x:max_x]
    
    # 保存裁剪的子图
    cv2.imwrite(output_path, sub_image)
    print(f"保存子地图: {output_path}")

# 遍历所有区域并保存对应的子图
for area in data['areas']:
    name = area['name']
    corners = [corner for corner in area['corners']]
    output_path = f"{name}.png"
    
    # 裁剪并保存子图
    save_sub_image(map_image, corners, output_path)
