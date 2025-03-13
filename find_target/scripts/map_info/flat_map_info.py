map_data = {
    'image': '~/ws_thesis_lzq/src/rbkairos_nav/map/flat_occupancy_map.png',
    'resolution': 0.05,
    'origin': [-6.1, -6.1, 0.0000],
    'negate': 0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196,
}

rooms = [
    {
        'name': 'Corridor',
        'level': 'high',
        'corners': [
            {'corner_1': {'x': 6, 'y': -3}},
            {'corner_2': {'x': 6, 'y': 0}},
            {'corner_3': {'x': 0, 'y': 0}},
            {'corner_4': {'x': 0, 'y': -3}}
        ],
        'has_door': False,
        'connections': [
            {'to': 'Livingroom'},
            {'to': 'Kitchen'},
            {'to': 'Bedroom'},
            {'to': 'Studyroom'},
            {'to': 'Bathroom'},
            {'to': 'Restroom'}
        ],
        'entrance': {
            'direction': {'yaw': -90},
            'position': {'x': 0, 'y': 0}
        }
    },
    {
        'name': 'Livingroom',
        'corners': [
            {'corner_1': {'x': 3, 'y': 0}},
            {'corner_2': {'x': 3, 'y': 6}},
            {'corner_3': {'x': -6, 'y': 6}},
            {'corner_4': {'x': -6, 'y': 0}}
        ],
        'has_door': False,
        'connections': [
            {'to': 'Corridor'},
            {'to': 'Kitchen'}
        ],
        'entrance': {
            'direction': {'yaw': 90},
            'position': {'x': 0, 'y': 0.273}
        }
    },
    {
        'name': 'Kitchen',
        'corners': [
            {'corner_1': {'x': 0, 'y': -6}},
            {'corner_2': {'x': 0, 'y': 0}},
            {'corner_3': {'x': -6, 'y': 0}},
            {'corner_4': {'x': -6, 'y': -6}}
        ],
        'has_door': False,
        'connections': [
            {'to': 'Corridor'},
            {'to': 'Livingroom'}
        ],
        'entrance': {
            'direction': {'yaw': -116.5},
            'position': {'x': -3, 'y': -1.5}
        }
    },
    {
        'name': 'Bedroom',
        'corners': [
            {'corner_1': {'x': 12, 'y': 0}},
            {'corner_2': {'x': 12, 'y': 6}},
            {'corner_3': {'x': 3, 'y': 6}},
            {'corner_4': {'x': 3, 'y': 0}}
        ],
        'has_door': True,
        'connections': [
            {'to': 'Corridor'}
        ],
        'entrance': {
            'direction': {'yaw': 90},
            'position': {'x': 4.5, 'y': 0.273}
        }
    },
    {
        'name': 'Studyroom',
        'corners': [
            {'corner_1': {'x': 12, 'y': -6}},
            {'corner_2': {'x': 12, 'y': 0}},
            {'corner_3': {'x': 6, 'y': 0}},
            {'corner_4': {'x': 6, 'y': -6}}
        ],
        'has_door': True,
        'connections': [
            {'to': 'Corridor'}
        ],
        'entrance': {
            'direction': {'yaw': 0},
            'position': {'x': 6.273, 'y': -1.5}
        }
    },
    {
        'name': 'Bathroom',
        'corners': [
            {'corner_1': {'x': 6, 'y': -6}},
            {'corner_2': {'x': 6, 'y': -3}},
            {'corner_3': {'x': 3, 'y': -3}},
            {'corner_4': {'x': 3, 'y': -6}}
        ],
        'has_door': True,
        'connections': [
            {'to': 'Corridor'}
        ],
        'entrance': {
            'direction': {'yaw': -90},
            'position': {'x': 4.5, 'y': -3.273}
        }
    },
    {
        'name': 'Restroom',
        'corners': [
            {'corner_1': {'x': 3, 'y': -6}},
            {'corner_2': {'x': 3, 'y': -3}},
            {'corner_3': {'x': 0, 'y': -3}},
            {'corner_4': {'x': 0, 'y': -6}}
        ],
        'has_door': True,
        'connections': [
            {'to': 'Corridor'}
        ],
        'entrance': {
            'direction': {'yaw': -90},
            'position': {'x': 1.5, 'y': -3.273}
        }
    }
]
