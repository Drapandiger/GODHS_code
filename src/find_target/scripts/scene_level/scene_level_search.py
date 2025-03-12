import json
import os

from scene_level.previous_knowledge_base.direct_go_and_catch import direct_go_and_catch
from scene_level.map_divider.map_divider import map_divider
from room_level.room_level_search import room_level_search

def scene_level_search():

    print("\u2B50 System started.")

    scene_name = input("\U0001F4AC Enter the scene name (could be 'flat', 'office' or 'mall'): ")
    map_divider(scene_name)

    while True:
    
        target_name = input("\U0001F4AC Enter the target you want to find (or 'exit' to quit): ")
        if target_name.lower() == "exit":
            break

        print(f"\u2B50 The target you want to find: {target_name}")

        # 创建输出目录
        output_dir = os.path.expanduser('~/ws_thesis_lzq/src/find_target/scripts/scene_level/previous_knowledge_base')
        os.makedirs(output_dir, exist_ok=True)

        # 构建输出文件路径
        output_file = os.path.join(output_dir, 'saved_previous_search_results.json')

        # 读取JSON文件
        with open(output_file, 'r') as f:
            saved_previous_search_results = json.load(f)

        # 获取所有场景名称
        saved_target_list = list(saved_previous_search_results.keys())
        
        if target_name in saved_target_list:
            target_dict = getattr(saved_previous_search_results, target_name)
            if scene_name == target_dict["scene_name"]:
                print("\u2B50 Target found in previous search results, the system will directly go and catch it.")
                direct_go_and_catch(target_name, scene_name)
            else:
                result = room_level_search(target_name, scene_name)
                if result == True:
                    print("\u2B50 Target found!")
                else:
                    print("\u2B50 Target not found!")
        else:
            result = room_level_search(target_name, scene_name)
            if result:
                print("\u2B50 Target found!")
            else:
                print("\u2B50 Target not found!")

if __name__ == "__main__":
    scene_level_search()


