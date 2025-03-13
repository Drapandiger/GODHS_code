import sys
from map_info.rooms_infos import rooms_infos
from language_model.create_prompt import create_prompt_rooms_sorting
from language_model.ollama_LLM import ollama_LLM
from carrier_level.carrier_level_search import carrier_level_search
from motion_planning.summitxl_movebase_controller import SummitXLMovebaseController

def room_level_search(target_name, scene_name):
    
    # extract room names and entrance coordinates
    room_names, room_coordinates = rooms_infos(scene_name)
    print(f"Extracted room names: {room_names}")

    if room_names:
        room_names.remove("Corridor")
        # create prompt and sort the rooms using LLM
        prompt = create_prompt_rooms_sorting(target_name, room_names)
        ollama_LLM()
        sorted_rooms = ollama_LLM(prompt)
        print(sorted_rooms)
        sorted_room_list = [room.strip() for room in sorted_rooms.split(',') if room.strip() in room_names]
        print(f"Sorted rooms: {sorted_room_list}")

        # initialize entrance pose of the room
        room_entrance_pose = []

        # move to each room under the order
        for room in sorted_room_list:
            if room in room_coordinates:
                coords = room_coordinates[room]
                print(f"\U0001F4E3 Now moving to room: {room}, Coordinates: x={coords['x']}, y={coords['y']}, yaw={coords['yaw']}")

                # save the room entrance pose
                room_entrance_pose = [coords['x'], coords['y'], coords['yaw']]

                # move to the room
                summit_controller = SummitXLMovebaseController()
                
                success = summit_controller.move_to_pose(room_entrance_pose)

                if not success:            
                    print(f"\u274C Failed to move to room: {room}")
                    sys.exit(1)
                else:
                    print(f"\u2B50 Moved to room: {room}")
                    success = False

                # start carrier level search
                result = carrier_level_search(target_name, scene_name, room_name=room)

                # reset the room entrance pose
                room_entrance_pose = []

                if result:
                    return result

            else:
                print(f"\u274C Warning: No coordinates found for room '{room}'")
                sys.exit(1)

        # 搜索完毕后未找到目标
        print(f"Search for {target_name} in the scene {scene_name} completed, target not found.")
        return False        

    else:
        print("\u274C No rooms found.")
        sys.exit(1)

