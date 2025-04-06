## Execution


## Setup
1. Make sure you RPi5 is running Ubuntu 22.04.5 Jammy Jellyfish
2. Set up SSH
3. Follow: https://docs.px4.io/main/en/ros2/user_guide.html
     - https://github.com/PX4/px4_ros_com/blob/main/launch/sensor_combined_listener.launch.py
     - https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/sensor_combined_listener.cpp
4. Understand: https://docs.px4.io/main/en/ros2/offboard_control.html 
     - https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp 

## Software
| Software | Purpose |
|----------|---------|
| PX4 1.14 | Flight controller |

| Library | Purpose |
|---------|---------|
| ROS 2 "Humble" | Middleware |
| MAVRos | MAVLink ROS package |
| OpenCV | Basic image processing |
| YOLO | Object detection |

## Design
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

**Hierarchy**
Drone/
├── README.md
├── ISSUES.md                           # TODO, bugs
├── .gitignore
├── CMakeLists.txt
├── src/
|   ├── sensor_node.cpp                 # Read sensors
│   └── control_node.cpp                # Issue commands
├── launch/
│   └── main.launch                     # Main system startup
├── models/
│   └── yolo/
│       ├── yolov8n_edgetpu.tflite      # Quantized model for Coral TPU
│       └── coco_labels.txt             # Object labels
└── docs/
    ├── hardware_info.md                # Hardware info
    └── software_info.md                # Software info