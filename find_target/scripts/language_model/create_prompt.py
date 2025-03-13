def create_prompt_rooms_sorting(input_target, room_names):
    area_list = ', '.join(room_names)
    prompt = f"""
    I have the following rooms: {area_list}. Where should I look for a '{input_target}'?
    Please sort the rooms in the order of likelihood where the '{input_target}' might be found.
    Please only output the room names separated by commas
    """
    return prompt

def create_prompt_classify_semantic_labels(semantic_labels_list):
    labels = ', '.join(semantic_labels_list)
    prompt = f"""
    I have a list of objects: {labels}.
    Your task is to classify each object into two categories:
    1. 'Carrier object' – An object that can carry or support other objects, such as table, shelf, or fridge.
    2. 'Non-carrier object' – Any object that does not carry or support other objects, or is already supported by other carrier objects, such as wall, lamp or cup.

    Example:
    - If given the objects: 'closet, book, wardrobe, bottle, bed, bowl, bathtub', the response is: closet, wardrobe, bed, bathtub

    Please identify which objects are 'carrier objects' and only output them separated by commas, without any additional text or explanations.
    """
    return prompt

def create_prompt_sort_carrier_objects(input_target, carrier_objects_list):
    carriers = ', '.join(carrier_objects_list)
    prompt = f"""
    Given the target '{input_target}' and the following carrier objects: {carriers},
    please sort the carrier objects in order of likelihood where the '{input_target}' might be found.
    Please only output the carrier object names separated by commas, without any additional text or explanations.
    """
    return prompt

def create_prompt_infer_hidden_areas(input_target, carrier_object):
    prompt = f"""
    For the carrier '{carrier_object}', consider the search target '{input_target}'. 
    Based on its typical structure and function, 
    only decide and sort the areas that worth searching for hidden spaces. 
    These areas might include 'top', 'bottom', 'sides', 'inside'. 
    'inside' refers to enclosed internal areas, while open internal areas (e.g., inside a bathtub) should be considered as 'top'.
    Please sort the areas in order of priority, from most likely to least likely.
    
    Example:
    - For a friger, the response is: inside, top
    - For a table, the response is: top, bottom
    - For a bathtub, the response is: top
    
    Please only output the area names separated by commas, without any additional text or explanations.
    """
    return prompt


if __name__ == '__main__':
    input_target = 'book'
    carrier_objects_list = ['shelf', 'table', 'desk']
    prompt = create_prompt_sort_carrier_objects(input_target, carrier_objects_list)
    print(prompt)

