from cat.node.detectron.run import analyze_image
from cat.node.Skeleton.process_skeleton import process_skeleton
import cv2


def get_object(image_array):

# image_path = "C:/Users/kakao/Desktop/Ajou_ComputerVisionAndRobotDesign/object_images/hand_objects_image_29.jpg"

    results = analyze_image(image_array)


    objects_height = {}

    # 반환된 결과에서 좌표 추출
    for idx, result in enumerate(results):
        class_name = result['class_name']
        bounding_box = result["bounding_box"]
        height = result['mask_height']

        objects_height[class_name] = height

        # print(f"Object {idx + 1}: {class_name}'s Bounding Box Coordinates")
        # print(f"  x1: {bounding_box['x1']:.2f}, y1: {bounding_box['y1']:.2f}")
        # print(f"  x2: {bounding_box['x2']:.2f}, y2: {bounding_box['y2']:.2f}")
        # print(f"  Center: ({bounding_box['center_x']:.2f}, {bounding_box['center_y']:.2f})")

    object_centers = [
        {
            "class_name": result["class_name"],
            "center_x": result["bounding_box"]["center_x"],
            "center_y": result["bounding_box"]["center_y"],
            "height": result['mask_height']
        }
        for result in results
    ]

    # print(objects_height)


    closest_object = process_skeleton(image_array, object_centers)

    if closest_object:
        # print(f"The closest object to the finger's extended line is: {closest_object}")
        print(f"And the {closest_object}'s height is {objects_height[closest_object]}")
    else:
        print("No closest object found.")
        return None

    # print(closest_object)

    target = closest_object
    target_height = objects_height[closest_object]

    return (target, target_height)

