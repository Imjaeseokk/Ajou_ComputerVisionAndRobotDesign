import os
import numpy as np
import cv2
import torch
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.utils.visualizer import Visualizer
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torch.functional")


os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# MetadataCatalog에서 클래스 이름 가져오기
# metadata = MetadataCatalog.get("images_val")  # "images_val"은 등록된 데이터셋 이름
class_names = ['Toothpaste', 'Can', 'Bottle', 'Paper', 'Rock']



def analyze_image(image_path):
    """
    이미지를 분석하고 객체 정보를 반환하는 함수.
    
    Parameters:
        image_path (str): 분석할 이미지 경로.

    Returns:
        list[dict]: 분석된 객체 정보 리스트.
    """
    # 모델 초기화
    cfg = get_cfg()
    cfg.merge_from_file(r"./mask_rcnn_R_50_FPN_3x.yaml")
    cfg.MODEL.WEIGHTS = "/home/defense/best_1207.pth"  # 가중치 파일 경로
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(class_names)  # 클래스 수
    cfg.MODEL.DEVICE = "cpu"  # CPU 환경
    predictor = DefaultPredictor(cfg)

    # 이미지 읽기 및 예측
    im = cv2.imread(image_path)
    outputs = predictor(im)
    instances = outputs["instances"]

    # 예측 결과 시각화 및 저장
    v = Visualizer(im[:, :, ::-1], scale=1.2)

    # 수동으로 클래스 이름 추가
    labels = [class_names[class_id.item()] for class_id in instances.pred_classes]
    out = v.overlay_instances(
        boxes=instances.pred_boxes if instances.has("pred_boxes") else None,
        masks=instances.pred_masks if instances.has("pred_masks") else None,
        labels=labels,
        assigned_colors=None,
    )
    result_image = out.get_image()[:, :, ::-1]

    # 저장 경로 설정 (_test.jpg 추가)
    save_path = image_path.replace(".jpg", "_test.jpg")
    cv2.imwrite(save_path, result_image)
    print(f"Saved visualization image at: {save_path}")

    # 결과 저장
    results = []
    for i in range(len(instances)):
        # 클래스 정보
        class_id = instances.pred_classes[i].item()
        class_name = class_names[class_id]
        confidence = instances.scores[i].item()

        # 바운딩 박스 정보
        box = instances.pred_boxes[i].tensor.cpu().numpy()[0]  # [x1, y1, x2, y2]
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        # 마스크 높이 계산
        mask = instances.pred_masks[i].cpu().numpy()  # 마스크를 NumPy 배열로 변환
        true_pixels = np.where(mask == True)
        mask_height = max(true_pixels[0]) - min(true_pixels[0]) if len(true_pixels[0]) > 0 else 0

        # 객체 정보 딕셔너리에 추가
        results.append({
            "class_id": class_id,
            "class_name": class_name,
            "confidence": confidence,
            "bounding_box": {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "center_x": center_x,
                "center_y": center_y
            },
            "mask_height": mask_height
        })

    print(results)

    return results

















# # 모델 설정 및 가중치 로드
# cfg = get_cfg()
# cfg.merge_from_file(r"C:\\Users\\kakao\\Desktop\\Ajou_ComputerVisionAndRobotDesign\\detectron2\\mask_rcnn_R_50_FPN_3x.yaml")
# cfg.MODEL.WEIGHTS = "best_1207.pth"  # 가중치 파일 경로
# cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
# cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5  # 데이터셋 클래스 수
# cfg.MODEL.DEVICE = "cpu"  # CPU 환경


# predictor = DefaultPredictor(cfg)



# # 예측 및 정보 추출
# def analyze_image(image_path):
#     im = cv2.imread(image_path)
#     outputs = predictor(im)
#     instances = outputs["instances"]

#     print(f"Processing image: {image_path}")
#     for i in range(len(instances)):
#         # 클래스 정보
#         class_id = instances.pred_classes[i].item()
#         class_name = class_names[class_id]
#         confidence = instances.scores[i].item()

#         # 바운딩 박스 정보
#         box = instances.pred_boxes[i].tensor.cpu().numpy()[0]  # [x1, y1, x2, y2]
#         x1, y1, x2, y2 = box
#         center_x = (x1 + x2) / 2
#         center_y = (y1 + y2) / 2

#         # 마스크 높이 계산
#         mask = instances.pred_masks[i].cpu().numpy()  # 마스크를 NumPy 배열로 변환
#         true_pixels = np.where(mask == True)
#         mask_height = max(true_pixels[0]) - min(true_pixels[0]) if len(true_pixels[0]) > 0 else 0

#         # 정보 출력
#         print(f"Object {i + 1}:")
#         print(f"  Class ID: {class_id}, Class Name: {class_name}")
#         print(f"  Confidence: {confidence:.2f}")
#         print(f"  Bounding Box: (x1={x1:.2f}, y1={y1:.2f}, x2={x2:.2f}, y2={y2:.2f}, center=({center_x:.2f}, {center_y:.2f}))")
#         print(f"  Mask Height: {mask_height}")

# # 분석할 이미지 경로 지정
# test_image_path = r"C:\\Users\\kakao\\Desktop\\Ajou_ComputerVisionAndRobotDesign\\imageset\\val\\object_image_46.jpg"  # 테스트할 이미지 경로
# analyze_image(test_image_path)
