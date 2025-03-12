import cv2
import numpy as np
import os
import importlib
import json

def crop_image(image, corners, origin, resolution):
    # 从corners字典列表中提取出x, y坐标
    x_coords = [corner[next(iter(corner))]['x'] for corner in corners]  # 提取x坐标
    y_coords = [corner[next(iter(corner))]['y'] for corner in corners]  # 提取y坐标

    min_x = min(x_coords)
    max_x = max(x_coords)
    min_y = min(y_coords)
    max_y = max(y_coords)

    # 计算像素范围
    x1, y1 = round((min_x - origin[0]) / resolution), round((-origin[1] - resolution - max_y) / resolution)
    x2, y2 = round((max_x - origin[0]) / resolution), round((-origin[1] - resolution - min_y) / resolution)

    return image[y1:y2+1, x1:x2+1]

def shrink_image(image):
    # 裁剪3个像素的边缘
    return image[3:-3, 3:-3]

def erode_image(image):
    # 创建一个 10x10 的方形核
    kernel = np.ones((10, 10), np.uint8)
    # 腐蚀操作，白色区域会收缩
    image = cv2.erode(image, kernel, iterations=1)
    # edge black
    image[:10, :] = 0
    image[-10:, :] = 0
    image[:, -10:] = 0
    image[:, :10] = 0
    return image

def extract_points(shrink_image, corners, resolution):
    # 找出灰度值为255的点
    white_points = np.argwhere(shrink_image == 255)

    # 找出灰度值非255的点
    non_white_points = np.argwhere(shrink_image != 255)

    # 将房间角点从地图坐标转换为像素坐标
    x_coords = [corner[next(iter(corner))]['x'] for corner in corners]  # 提取x坐标
    y_coords = [corner[next(iter(corner))]['y'] for corner in corners]  # 提取y坐标

    min_x = min(x_coords)
    max_y = max(y_coords)

    # 转换像素坐标到真实坐标系
    def pixel_to_real_coords(pixel_points):
        real_points = set()
        for point in pixel_points:
            # point[0]是y坐标(行)，point[1]是x坐标(列)
            real_x = min_x + point[1] * resolution + 3 * resolution
            real_y = max_y - point[0] * resolution - 3 * resolution
            real_points.add((round(real_x, 2), round(real_y, 2)))
        return real_points

    free_area_points = pixel_to_real_coords(white_points)
    occupancy_area_points = pixel_to_real_coords(non_white_points)

    return free_area_points, occupancy_area_points

def save_points_to_npy(points, filename):
    # 将点集转换为NumPy数组
    points_array = np.array(list(points))
    np.save(filename, points_array)

def load_points_from_npy(filename):
    # 加载NumPy数组并转换为集合
    points_array = np.load(filename)
    return set(map(tuple, points_array))

def save_image(image, file_path):
    cv2.imwrite(file_path, image)

def save_knowledge_base_of_room(scene_name, room_name, 
                    original_image_path, shrinked_image_path, eroded_image_path,
                    free_points_path, occupancy_points_path, footprint_points_path):
    """
    Save room information to a JSON file.

    Args:
        scene_name: Name of the scene (e.g., 'flat', 'office')
        room_name: Name of the room
        其他参数: 各种路径信息
    """
    # 创建输出目录
    output_dir = os.path.expanduser('~/ws_thesis_lzq/src/find_target/scripts/scene_level/previous_knowledge_base')
    os.makedirs(output_dir, exist_ok=True)

    # 构建输出文件路径
    output_file = os.path.join(output_dir, 'saved_previous_search_results.json')

    # 准备要保存的数据
    room_data = {
        'scene_name': scene_name,
        'original_image_path': original_image_path,
        'shrinked_image_path': shrinked_image_path,
        'eroded_image_path': eroded_image_path,
        'free_points_path': free_points_path,
        'occupancy_points_path': occupancy_points_path,
        'footprint_points_path': footprint_points_path
    }

    try:
        # 读取已存在的数据
        if os.path.exists(output_file):
            with open(output_file, 'r') as f:
                existing_data = json.load(f)
        else:
            existing_data = {}

        # 更新或添加新的房间数据
        existing_data[room_name] = room_data

        # 写入JSON文件
        with open(output_file, 'w') as f:
            json.dump(existing_data, f, indent=4)

        print(f"\U0001F4E3 Successfully saved room information to {output_file}")
    except Exception as e:
        raise IOError(f"Failed to write knowledge base: {e}")


def map_divider(scene_name):
    """
    根据传入的场景名称动态加载地图模块，并进行地图切割、裁剪、膨胀处理。
    """

    try:
        # 拼接模块名并加载
        module_name = f"map_info.{scene_name}_map_info"
        map_module = importlib.import_module(module_name)

        # 获取地图数据
        map_data = map_module.map_data
        rooms = map_module.rooms

        # 读取地图图像
        map_image = cv2.imread(os.path.expanduser(map_data['image']), cv2.IMREAD_GRAYSCALE)
        resolution = map_data['resolution']
        origin = map_data['origin']

        # check whether the map image is loaded
        if map_image is None:
            raise Exception("\u274C Failed to load map image")

        # 假设保存到某个特定目录
        save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}')
        os.makedirs(save_dir, exist_ok=True) # 确保目录存在

        # 遍历所有房间并处理
        for room in rooms:
            corners = room['corners']
            room_name = room['name']

            # 切割房间区域（原图）
            original_image = crop_image(map_image, corners, origin, resolution)

            original_image_path = os.path.join(save_dir, f'{room_name}_original.png')

            save_image(original_image, original_image_path)

            # 裁剪后的图像
            shrinked_image = shrink_image(original_image)
            shrinked_image_path = os.path.join(save_dir, f'{room_name}_shrink.png')
            save_image(shrinked_image, shrinked_image_path)

            free_area_points, occupancy_area_points = extract_points(shrinked_image, corners, resolution)

            # 构建完整路径
            free_points_path = os.path.join(save_dir, f'{room_name}_free_area_points.npy')
            occupancy_points_path = os.path.join(save_dir, f'{room_name}_occupancy_area_points.npy')
            # 保存
            save_points_to_npy(free_area_points, free_points_path)
            save_points_to_npy(occupancy_area_points, occupancy_points_path)

            # 收缩后的图像
            eroded_image = erode_image(shrinked_image)
            eroded_image_path = os.path.join(save_dir, f'{room_name}_eroded.png')
            save_image(eroded_image, eroded_image_path)

            footprint_area_points, _ = extract_points(eroded_image, corners, resolution)
            # 构建完整路径
            footprint_points_path = os.path.join(save_dir, f'{room_name}_footprint_area_points.npy')
            # 保存
            save_points_to_npy(footprint_area_points, footprint_points_path)

            save_knowledge_base_of_room(scene_name, room_name, 
                        original_image_path, shrinked_image_path, eroded_image_path,
                        free_points_path, occupancy_points_path, footprint_points_path)

    except Exception as e:
        print(f"\u274C Error occurred while extracting rooms: {e}")

# 示例使用
if __name__ == "__main__":
    scene_name = 'flat'  # 输入场景名称 'flat' 或 'office'
    map_divider(scene_name)