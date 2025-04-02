# Autonomous Object-Tracking/Following Drone

## Goal
Autonomous object-tracking/following drone

**Topics:** Neural Nets, CV, Sensor Fusion, Path Planning, Controls

## Hardware

| Component | Name | Cost | Purpose |
|-----------|------|------|---------|
| Computer | [Raspberry Pi 5 4GB](https://www.amazon.com/Raspberry-Pi-4GB-2023-Processor/dp/B0CK3L9WD3/) | | General computing |
| AI Accelerator | [Coral TPU](https://coral.ai/products/accelerator) | | Run AI models (YOLO) |
| Flight Controller | [Pixhawk 2.4.8](https://www.amazon.com/Readytosky-Pixhawk-Controller-Autopilot-Splitter/dp/B07CHQ7SZ4/) | | Flight hardware control, IMU |
| Frame | [S550 Hexacopter](https://www.amazon.com/FPVDrone-Hexacopter-6-Axis-Unflodable-Landing/dp/B082V45KHH/) | | Frame |
| Motor | [ReadyToSky 2212 920KV](https://www.amazon.com/Readytosky-Brushless-Motors-Phantom-Quadcopter/dp/B075DD16LK/) | | Motors |
| ESC | [ReadyToSky 30A](https://www.amazon.com/Readytosky-Electronic-Controller-Helicopter-Quadcopter/dp/B09G5WFXSV/) | | ESCs |
| Propeller | [ReadyToSky 1045](https://www.amazon.com/Readytosky-6Pairs-Propeller-Quadcopter-Multirotor/dp/B0823NNTKD/) | | Propellers |
| Camera | [8MP Stereo](https://www.amazon.com/Binocular-IMX219-83-8Megapixels-Supports-Developer/dp/B0CNH8P715/) | | Vision and relative positioning |
| GPS | [ReadyToSky M8N](https://www.amazon.com/Readytosky-Compass-Protective-Standard-Controller/dp/B01KK9A8QG/) | | Global positioning |
| Battery | [4s 5000mAh (14.8V)](https://www.amazon.com/HRB-5000mAh-Connector-Airplane-Helicopter/dp/B06XK8WWX1/) | | Battery |
| Controller | [Xbox Bluetooth](https://www.amazon.com/Xbox-Wireless-Controller-Black-one/dp/B01LPZM7VI/) | | Manual override |

### Power Calculations
**Hover Power Estimate:** ~65A
- 6x Motors @ 12V, ~10A -> 60A
- Nano @ 5V, ~10W -> ~2A
- Pixhawk @ 12V, ~5W -> 0.4A

**Flight Time Calculation:** ~3:45min
Flight Time (min) = (Battery Capacity (mAh) * efficiency) / current (mA) * 60

Assuming 4s 5000mAh LiPo, 0.8 efficiency:
(5,000 * 0.8) / (65,000) * 60 = ~3:45min

## Software

| Library | Purpose |
|---------|---------|
| ROS | Middleware (Jetson->PixHawk communication) using MAVRos package |
| OpenCV | Basic image processing |
| YOLO | Object detection |
| DepthAI | Allows YOLO to run on Luxonis |

**Software Flow**
1. ROS detects an object using DepthAI/YOLO
2. RP5 performs computations
3. ROS sends control information to PixHawk using MAVRos
4. PixHawk executes the command with flight logic

### Resources
Person-Tracking Hexacopter - https://m.youtube.com/watch?v=o3eywsXBpwA   