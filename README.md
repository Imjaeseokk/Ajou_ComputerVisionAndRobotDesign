# Computer Vision and Robot Design in Ajou Univ

## 👋 Introduction

This project presents a gesture-based control system for service robots, utilizing TurtleBot3 and Mediapipe for hand gesture recognition. By combining Detectron2 for object segmentation and real-time hand skeleton tracking, the system identifies user-pointed objects and enables the robot to navigate and interact with them. Experimental results show high accuracy, with gesture classification at 98.83% and Mask R-CNN at 96.58%, demonstrating the system's potential to improve human-robot interaction through advanced vision-based technologies.

## ⚙️ Setup and Installation

### Turtlebot3
```bash
# Network
nmcli device wifi hotspot ssid turtlebot_abcde password 12345678 ifname wlan0
# -> After this, connect to the TurtleBot's Wi-Fi network from the ROS Linux system

# Bringup
bringup.sh

# PUBLISHER
cd ~/turtlebot3_ws/src
colcon build
source install/setup.bash
ros2 run cat camera_publisher
```


### Local 
```bash
# SUBSCRIBER
cd ~/turtlebot3_ws/src/cat
colcon build
source install/setup.bash
ros2 run cat camera_subscriber

# GO
cd ~/turtlebot3_ws/src/cat
colcon build
source install/setup.bash
ros2 run cat go

# Run from ROS
cd ~/turtlebot3_ws/CVROBOT
python3 main.py
```

## 🚀 How It Works

### Camera Publisher
- Captures image frames continuously using the robot's camera.
- If an object is detected in the frame, the image is published to the ROS topic for further processing.
- This module acts as the starting point for the system's data pipeline.

### Gesture Recognition
- Processes the published image to identify user hand gestures such as "rock" or "paper."
- A pre-trained Detectron2 model is used to analyze the image and classify gestures.
- If a "rock" gesture is recognized, it triggers a command to start robot navigation towards the user’s intended target.

### Object Detection
- Detects objects in the image using Detectron2 for semantic segmentation and bounding box detection.
- Simultaneously, Mediapipe calculates the direction of the user's index finger.
- Combines object coordinates with the pointing direction to identify the closest object being indicated.

### Robot Movement
- Receives the gesture and object detection results and translates them into navigation commands for TurtleBot3.
- Moves the robot towards the detected object and adjusts its position for interaction.
- Ensures smooth and accurate movement by integrating gesture and object detection data in real-time.


## 🔑 Key Components
- 🐢**TurtleBot3** Robotic hardware platform.
- 👈**Mediapipe** Hand gesture recognition.
- 📦**Detectron2** Object semantic segmantation and detection
- 🖍️ **Labelme**: Tool for creating annotated datasets for object detection and segmentation.

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
- [Jaeseokk](https://imjaeseokk.github.io/) `me✋`
- [Hunseo](https://github.com/255is255)
- [Sangcheon](https://github.com/Park-Sangcheon)