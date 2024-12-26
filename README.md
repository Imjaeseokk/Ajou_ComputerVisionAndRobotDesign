# Ajou_ComputerVisionAndRobotDesign

## 👋 Introduction

This project presents a gesture-based control system for service robots, utilizing TurtleBot3 and Mediapipe for hand gesture recognition. By combining Detectron2 for object segmentation and real-time hand skeleton tracking, the system identifies user-pointed objects and enables the robot to navigate and interact with them. Experimental results show high accuracy, with gesture classification at 98.83% and Mask R-CNN at 96.58%, demonstrating the system's potential to improve human-robot interaction through advanced vision-based technologies.

## 



## 🎤 Presentation 

[![Slide1](./presentation/slides/slide1.JPG)](https://github.com/Imjaeseokk/Ajou_ComputerVisionAndRobotDesign/tree/main/presentation)

## 📁 Repository Structure

- `dataset/`: Contains training and validation images along with label information.
- `detectron/`: Source code for Detectron2-based object segmentation.
- `model/`: Pre-trained Detectron2 model parameters (trained on COCO dataset and fine-tuned with our custom image dataset).
- `presentation/`: Presentation slides.
- `skeleton/`: Source code for Mediapipe-based hand landmark detection.
  
## 👏 Thank You!

We are Team 2, consisting of the following members:
- Jaeseokk `me✋`
- [Hunseo](https://github.com/255is255)
- [Sangcheon](https://github.com/Park-Sangcheon)