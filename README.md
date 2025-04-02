# Autonomous Object-Tracking/Following Drone

## Goal
Autonomous object-tracking/following drone

**Topics:** Neural Nets, CV, Sensor Fusion, Path Planning, Controls

## Hardware

| Component | Name | Cost | Purpose |
|-----------|------|------|---------|
| Computer | [Raspberry Pi 5 4GB](https://www.amazon.com/Raspberry-Pi-4GB-2023-Processor/dp/B0CK3L9WD3/) | | General computing |
| AI Coprocessor | [Coral TPU](https://coral.ai/products/accelerator) | | Run AI models (YOLO) |
| Flight Controller | [Pixhawk 2.4.8](https://www.amazon.com/Readytosky-Pixhawk-Controller-Autopilot-Splitter/dp/B07CHQ7SZ4/) | | Flight hardware control, IMU |
| Frame | [S550 Hexacopter](https://www.amazon.com/FPVDrone-Hexacopter-6-Axis-Unflodable-Landing/dp/B082V45KHH/) | | Frame |
| Motor | [ReadyToSky 2212 920KV](https://www.amazon.com/Readytosky-Brushless-Motors-Phantom-Quadcopter/dp/B075DD16LK/) | | Motors |
| ESC | [ReadyToSky 30A](https://www.amazon.com/Readytosky-Electronic-Controller-Helicopter-Quadcopter/dp/B09G5WFXSV/) | | ESCs |
| Propeller | [ReadyToSky 1045](https://www.amazon.com/Readytosky-6Pairs-Propeller-Quadcopter-Multirotor/dp/B0823NNTKD/) | | Propellers |
| Camera | [IMX219-83 Stereo](https://www.amazon.com/IMX219-83-Binocular-3280x2464-Developer-XYGStudy/dp/B088ZLMR9M/) | | Vision and relative positioning |
| Cam Wire | [Pi 5 Wire](https://www.amazon.com/IMX219-83-Binocular-3280x2464-Developer-XYGStudy/dp/B088ZLMR9M/) | | Camera interfacing |
| GPS | [ReadyToSky M8N](https://www.amazon.com/Readytosky-Compass-Protective-Standard-Controller/dp/B01KK9A8QG/) | | Global positioning |
| Battery | [4s 5000mAh (14.8V)](https://www.amazon.com/HRB-5000mAh-Connector-Airplane-Helicopter/dp/B06XK8WWX1/) | | Battery |
| Power Module | [Power Module (XT60)](https://www.amazon.com/Pixhawk-BEC-Helicopter-Quadcopters-Accessories/dp/B0BCJM3R5P/) | | Pixhawk power |
| PDB | [Acxico](https://www.amazon.com/Acxico-Drone-Power-Distribution-Output/dp/B0B2PF6YPQ/) | | Power distribution |
| Controller | [Xbox Bluetooth](https://www.amazon.com/Xbox-Wireless-Controller-Black-one/dp/B01LPZM7VI/) | | Manual override |

### Power Calculations
**Hover Power Estimate:** ~65A
- 6x Motors @ 12V, ~10A -> 60A
- Nano @ 5V, ~10W -> ~2A
- Pixhawk @ 12V, ~5W -> 0.4A

**Flight Time Calculation:** ~3:45min

Flight Time (min) = (Battery Capacity (mAh) * efficiency) / current (mA) * 60

Assuming 5000mAh LiPo, 0.8 efficiency:

(5,000 * 0.8) / (65,000) * 60 = ~3:45min

## Software

| Library | Purpose |
|---------|---------|
| ROS | Middleware (Jetson->PixHawk communication) using MAVRos package |
| OpenCV | Basic image processing |
| YOLO | Object detection |

**Software Flow**
1. RPi5 processes image from camera using OpenCV
2. RPi5 sends data to Coral TPU
3. Coral processes data with YOLO
4. Coral sends data to RPi5
5. RPi5 uses recognitions and camera depth info for relative position to target
6. Pixhawk sends IMU and GPS information to RPi5
7. RPi5 uses imu and GPS for global position estimation
8. RPi5 decides a path and generates commands to follow it
9. RPi5 sends commands to Pixhawk using MAVRos
10. Pixhawk sends control information to ESCs

### Resources
ROS + YOLO
  - https://docs.ultralytics.com/guides/ros-quickstart/#use-ultralytics-with-ros-sensor_msgsimage 

Coral + RPi5 
  - https://docs.ultralytics.com/guides/coral-edge-tpu-on-raspberry-pi/#installing-the-edge-tpu-runtime 
  - https://www.diyengineers.com/2024/05/18/setup-coral-tpu-usb-accelerator-on-raspberry-pi-5/ 
  - https://www.hackster.io/stefanw1/yolov8-on-raspberry-pi5-with-coral-tpu-8efd23 

RPi5 + Pixhawk 2.4.8 
  - https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html

Pixhawk wiring
  - https://ardupilot.org/copter/docs/connect-escs-and-motors.html

Person-Tracking Hexacopter 
  - https://m.youtube.com/watch?v=o3eywsXBpwA   