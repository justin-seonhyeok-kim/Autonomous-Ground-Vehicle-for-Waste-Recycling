# Autonomous-Ground-Vehicle-for-Waste-Recycling
Autonomous object recognition and robotic control logic for real-time material sorting and recycling

## 📌 Project Overview
This repository contains the full-stack implementation of an autonomous recycling robot developed for the **MECH 3433** project at **The University of Hong Kong**. The system integrates deep learning-based object detection with precise mechanical control to identify, collect, and sort different types of waste materials (Paper, Metal, Plastic) into designated zones.

## 🛠 Technical Architecture & Mechanism

### 1. Vision & Detection (Deep Learning)
* **Model:** Utilized **YOLOv5** object detection framework for real-time garbage classification.
* **Implementation:** Integrated `torch.hub` to load custom-trained weights (`realfinalbest.pt`) with a confidence threshold of 0.3 to ensure high-precision detection and prevent false positives in dynamic environments.
* **Real-time Processing:** Leveraged the RoboMaster SDK and OpenCV to stream and process video frames, extracting target coordinates for the navigation system.

### 2. Autonomous Navigation & Coordinate System
* **Global Mapping:** Designed a 2D coordinate system where the robot tracks its relative position `(x, y)` from a starting origin (0,0).
* **Relative Transformation:** Developed logic to calculate the necessary rotation and translation movements. For instance, the robot calculates the angle `θ` to the target and executes `chassis.move` commands based on real-time feedback.
* **Sorting Zones:** Pre-defined specific coordinates for different materials:
    * **Paper Zone:** (0, 2)
    * **Metal Zone:** (1, 2)
    * **Plastic Zone:** (-1, 2)

### 3. Robotic Control Logic
* **State Machine:** Implemented a robust logic flow: `Searching` → `Detection` → `Targeting` → `Grabbing` → `Returning to Zone`.
* **Mechanical Execution:**
    * **Gimbal & Blaster:** Precise aiming mechanism to center targets within the camera frame.
    * **Robotic Arm & Gripper:** Coded sequential commands to lower the arm, execute a `gripper.open(power=300)` command, and securely collect items before transporting them.
* **Error Handling:** Integrated `time.sleep` intervals and feedback loops to ensure mechanical stability during the grabbing and releasing phases.

## 💻 Tech Stack
* **Language:** Python
* **Frameworks:** YOLOv5 (PyTorch), OpenCV
* **Hardware:** DJI RoboMaster S1 (Chassis, Gimbal, Robotic Arm, Gripper)
* **API:** RoboMaster SDK
