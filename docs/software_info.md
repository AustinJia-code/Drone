# Software
| Library | Purpose |
|---------|---------|
| ROS | Middleware (Jetson->PixHawk communication) using MAVRos package |
| OpenCV | Basic image processing |
| libcoral | Coral TPU interfacing |
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

**Hierarchy**
Drone/
├── README.md
├── ISSUES.md                           # TODO, bugs
├── .gitignore
├── CMakeLists.txt
├── scripts/                            # Helper scripts
│   └── calibration_tool.py             # Camera calibration tool
├── config/                             # Config files
│   ├── camera_params.yaml              # Camera calibration parameters
│   ├── detector_config.yaml            # YOLO model configuration
│   ├── flight_params.yaml              # Flight controller parameters
│   └── ros_params.yaml                 # ROS node configurations
├── include/
│   ├── drone_tracker/
│   │   ├── perception/
│   │   │   ├── detector.h              # Object detection interface
│   │   │   ├── tracker.h               # Object tracking algorithms
│   │   │   └── depth.h                 # Stereo depth processing
│   │   ├── localization/
│   │   │   ├── fusion.h                # Sensor fusion interface
│   │   │   ├── position.h              # Positioning system
│   │   │   └── state.h                 # State estimation
│   │   ├── planning/
│   │   │   ├── path_planner.h          # Path planning interface
│   │   │   ├── obstacle_avoid.h        # Obstacle avoidance
│   │   │   └── mission.h               # Mission manager
│   │   ├── control/
│   │   │   ├── commander.h             # Flight command interface
│   │   │   ├── mavlink_bridge.h        # MAVLink communication
│   │   │   └── tele_op.h               # Controller input handling
│   │   └── utils/
│   │       └─── math_funcs.h           # Misc math functions
├── src/
│   ├── perception/
│   │   ├── CMakeLists.txt
│   │   ├── detector.cpp
│   │   ├── tracker.cpp
│   │   └── depth.cpp
│   ├── localization/
│   │   ├── CMakeLists.txt
│   │   ├── fusion.cpp
│   │   ├── position.cpp
│   │   └── state.cpp
│   ├── planning/
│   │   ├── CMakeLists.txt
│   │   ├── path_planner.cpp
│   │   ├── obstacle_avoid.cpp
│   │   └── mission.cpp
│   ├── control/
│   │   ├── CMakeLists.txt
│   │   ├── commander.cpp
│   │   ├── mavlink_bridge.cpp
│   │   └── tele_op.cpp
│   ├── utils/
│   │   ├── CMakeLists.txt
│   │   └── math_funcs.h
│   └── nodes/                          # ROS nodes
│       ├── detector_node.cpp           # Object detection node
│       ├── tracker_node.cpp            # Object tracking node
│       ├── planner_node.cpp            # Path planning node
│       └── control_node.cpp            # Flight control node
├── launch/                             # ROS launch files
│   └── main.launch                     # Main system startup
├── models/
│   ├── yolo/                           # YOLO model files
│   │   ├── yolov8n_edgetpu.tflite      # Quantized model for Coral TPU
│   │   └── coco_labels.txt             # Object labels
├── third_party/
│   ├── CMakeLists.txt                  # Third-party build config
│   └── README.md                       # Third-party libraries info
├── tests/
│   ├── CMakeLists.txt
│   ├── perception_tests.cpp
│   ├── planning_tests.cpp
│   └── control_tests.cpp
└── docs/
    ├── hardware_info.md                # Hardware info
    ├── software_info.md                # Software info
    └── setup_guide.md                  # Holistic setup guide