from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.utils.visualizer import ColorMode
import cv2
import os
from datetime import datetime

class Gesture():
    def __init__(self):
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.DATASETS.TRAIN = ("mdata_116_train",)
        self.metadata_train = MetadataCatalog.get("mdata_116_train").set(thing_classes=["Toothpaste", "Can","Bottle","Paper","Rock"])
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  

        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
        cfg.MODEL.DEVICE = 'cpu'
        model_path = "/home/defense/best_1207.pth"
        cfg.MODEL.WEIGHTS = model_path  # path to the model we just trained
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7   # set a custom testing threshold
        self.predictor = DefaultPredictor(cfg)
        print("=================Finish Init========================")

    def detect_gesture(self, img, id):
        outputs = self.predictor(img)
        print('==========', id, '============')
        print(outputs["instances"].pred_classes)
        print(outputs["instances"].pred_boxes)

        class_index = None
        class_name = None
        confidence_score = None

        v = Visualizer(img[:, :, ::-1], metadata=self.metadata_train, scale=0.8, instance_mode=ColorMode.IMAGE_BW)
        out = v.draw_instance_predictions(outputs["instances"].to("cpu"))

        for idx, coordinates in enumerate(outputs["instances"].pred_boxes):
            class_index = outputs["instances"].pred_classes[idx]
            confidence_score = outputs["instances"].scores[idx].item()  # 신뢰도
            class_name = self.metadata_train.thing_classes[class_index]
            print(f"Detected: {class_name} (Index: {class_index}, Confidence: {confidence_score:.2f})")

        cv2.waitKey(1)  # 1ms 대기 (실시간 업데이트)

        return (class_name, class_index, confidence_score)
