# AR4 Autonomous Robotic Arm: Computer Vision & Sensor Fusion Control

An autonomous object acquisition system for the 6-axis AR4 robotic arm, integrating **YOLOv8** for real-time object detection and **VL53L1X (Time-of-Flight)** laser sensors for high-precision spatial positioning.

## 🚀 Overview
This project implements a sophisticated control pipeline that bridges the gap between high-level computer vision and low-level hardware actuation. The system uses a **Data Fusion** approach, combining bounding box (BBox) dimensions from YOLO with micro-distance data from a ToF sensor to achieve reliable autonomous "pick and place" operations.

### Key Technical Features
* **Target Tracking:** Real-time identification and centering using Ultralytics YOLOv8.
* **Sensor Fusion:** Hybrid localization merging visual data with laser-based distance measurements (ToF) to solve depth-perception issues in 2D camera streams.
* **Dynamic PID Control:** Implemented a Proportional-Integral-Derivative controller with **dynamic damping** based on object proximity to prevent oscillations during the approach phase.
* **Real-time Telemetry:** Live data broadcasting via UDP to **Teleplot**, monitoring error vectors ($e_x, e_y$), ToF distance, and BBox scaling.
* **Asynchronous Architecture:** Multithreaded I/O handling for non-blocking serial communication (115200 baud) between Linux, Arduino, and ESP32 nodes.

## 🛠️ Hardware Stack
* **Robot:** Annin Robotics AR4 (6-Axis).
* **Main Controller:** Linux PC (Ubuntu/Debian).
* **Actuator Node:** Arduino Uno (Serial-to-Servo Bridge).
* **Vision Node:** ESP32-CAM.
* **Distance Sensor:** VL53L1X Time-of-Flight (ToF) via ESP32.

## 💻 Software & Libraries
* **Language:** Python 3.10+
* **Vision:** OpenCV, Ultralytics (YOLOv8)
* **Control:** Custom PID implementation with state-machine logic.
* **Telemetry:** Teleplot (UDP visualization).
* **Communication:** PySerial.

## 📐 Mathematical Approach: PID Control
The system calculates the movement delta for the base and shoulder axes using a standard PID formula:

$$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$$

To improve stability as the arm nears the target, a **dynamic damping factor** $(\zeta)$ is applied to the gains $(K)$ based on the BBox width $(w)$, reducing sensitivity to visual noise at close range.

## 📂 Project Structure
```text
├── vision_brazo_yolo.py    # Main autonomous control loop (Python)
├── robotic_arm_log.csv     # Telemetry log for post-operation analysis
├── yolov8n.pt              # Pre-trained YOLOv8 weights
└── assets/                 # Images, demos, and architecture diagrams
