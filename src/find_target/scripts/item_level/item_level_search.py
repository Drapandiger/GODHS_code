import os
import numpy as np

from language_model.ollama_LLM import ollama_LLM
from language_model.create_prompt import create_prompt_infer_hidden_areas
from item_level.search_action.top_search_action import top_search_action
from item_level.search_action.bottom_search_action import bottom_search_action
from item_level.search_action.sides_search_action import sides_search_action
from item_level.search_action.inside_search_action import inside_search_action

def load_points_from_npy(filename):
    # 加载NumPy数组并转换为集合
    points_array = np.load(filename)
    return set(map(tuple, points_array))

def item_level_search(target_name, scene_name, room_name, carrier_name, 
                      occupancy_points_set, free_points_set, footprint_points_set):
    
    prompt = create_prompt_infer_hidden_areas(target_name, carrier_name)
    ollama_LLM()
    hidden_areas = ollama_LLM(prompt)
    print(hidden_areas)
    # 将hidden_areas按顺序拆分为列表
    hidden_areas_list = [hidden_area.strip() for hidden_area in hidden_areas.lower().split(',') if hidden_area.strip() in ['top', 'sides', 'bottom', 'inside']]
    print(f"Language model guess for {carrier_name} in the {room_name} hidden space: {hidden_areas_list}")

    all_points = occupancy_points_set.union(free_points_set)

    # 获取房间的边界
    room_x_max = max(point[0] for point in all_points)
    room_x_min = min(point[0] for point in all_points)
    room_y_max = max(point[1] for point in all_points)
    room_y_min = min(point[1] for point in all_points)

    save_dir = os.path.expanduser(f'~/ws_thesis_lzq/src/find_target/scripts/knowledge_base/{scene_name}/{room_name}')
    carrier_points_path = os.path.join(save_dir, f'{carrier_name}_point_cloud.npy')
    carrier_pcl_set = load_points_from_npy(carrier_points_path)
    carrier_height = max(point[2] for point in carrier_pcl_set)
    carrier_areas_path = os.path.join(save_dir, f'{carrier_name}_area.npy')
    carrier_points_set = load_points_from_npy(carrier_areas_path)
    
    # 按顺序依次搜索
    for area in hidden_areas_list:
        area = area.strip()
        if area == 'top':
            if carrier_height <= 1.6:
                result = top_search_action(scene_name, room_name, carrier_name, target_name,
                        carrier_points_set, occupancy_points_set, footprint_points_set,
                        room_x_max, room_x_min, room_y_max, room_y_min,
                        carrier_height, horizontal_fov=60, vertical_fov=60*9/16,
                        grid_interval=0.05)
                if result:
                    return result
            else:
                print("Top area can not be reached.")
        elif area == 'sides':
            result = sides_search_action(scene_name, room_name, carrier_name, target_name,
                      carrier_points_set, occupancy_points_set, footprint_points_set,
                      room_x_max, room_x_min, room_y_max, room_y_min,
                      carrier_height, horizontal_fov=60, vertical_fov=60*9/16,
                      grid_interval=0.05)
            if result:
                return result
        elif area == 'bottom':
            result = bottom_search_action(scene_name, room_name, carrier_name, target_name,
                      carrier_points_set, occupancy_points_set, footprint_points_set,
                      room_x_max, room_x_min, room_y_max, room_y_min,
                      carrier_height, horizontal_fov=60, vertical_fov=60*9/16,
                      grid_interval=0.05)
            if result:
                return result
        elif area == 'inside':
            if carrier_name in ['fridge', 'cabinet']:
                result = inside_search_action(scene_name, room_name, carrier_name, target_name)
                if result:
                    return result
            else:
                print("No inside area for this carrier.")

    # 搜索完毕后未找到目标
    print(f"Search for {target_name} by {carrier_name} in {room_name} of {scene_name} completed, target not found.")
    return False