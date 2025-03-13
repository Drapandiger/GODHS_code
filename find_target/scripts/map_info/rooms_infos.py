import importlib

def rooms_infos(scene_name):
    """
    Extract room names and entrance coordinates from flat_map_info.py.

    returns:
        room_names (list): list of extracted room names.
        room_coordinates (dict): dictionary of entrance position and yaw angle for each room.
    """
    try:
        # get the data of map and rooms from map_info.py
        module_name = f"map_info.{scene_name}_map_info"

        # load specified modules dynamically
        map_info = importlib.import_module(module_name)

        # get the data from map_data and rooms
        map_data = map_info.map_data
        rooms = map_info.rooms

        room_names = []
        room_coordinates = {}

        for room in rooms:
            name = room.get('name')
            entrance = room.get('entrance', {})
            position = entrance.get('position', {})
            yaw = entrance.get('direction', {}).get('yaw', 0)

            if name:
                room_names.append(name)
            
            if name and position:
                # record the entrance position and direction
                room_coordinates[name] = {
                    'x': position.get('x', 0),
                    'y': position.get('y', 0),
                    'yaw': yaw
                }

        return room_names, room_coordinates
    except Exception as e:
        print(f"\u274C Error occurred while extracting rooms: {e}")
        return [], {}

# example
if __name__ == '__main__':
    # input the scene name，could be 'flat' 或 'office'
    scene_name = 'flat'  # could be changed to 'office'
    room_names, room_coordinates = rooms_infos(scene_name)
    print(f"extracted room names: {room_names}")
    print(f"coordinates of entrance: {room_coordinates}")