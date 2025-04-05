## Execution


## Setup
**Teleop**
On RPi5:
1. Edit /boot/config.txt
   ```
   enable_uart=1
   ```
2. Disable serial console
   ```
   sudo raspi-config
    # -> Interfacing Options -> Serial
    # -> Disable login shell over serial, but enable hardware serial port
   ``` 
3. Reboot
   ```
   sudo reboot
   ```
4. Find serial port of Pixhawk, UART on the Pi usually shows up as /dev/serial0 or /dev/ttyAMA0.
   ```
   ls -l /dev/serial*
   ```
5. Launch MAVRos
   ```
   roslaunch mavros apm.launch fcu_url:=/dev/serial0:57600
   ```
> Use apm.launch (not px4.launch) - Pixhawk 2.4.8 runs ArduPilot, not PX4
> Ensure baud rate matches the Pixhawk TELEM2 port (default is usually 57600).

On Pixhawk:
1. Configure ```TELEM2``` via Mission Planner
   ```
   SERIAL2_PROTOCOL → 1 (MAVLink)
   SERIAL2_BAUD → 57 (for 57600 baud)
   ```

On RPi5:
1. Verify connection
   ```rostopic echo /mavros/state```


## Software
| Software | Purpose |
|----------|---------|
| PX4 1.14 | Flight controller |

| Library | Purpose |
|---------|---------|
| ROS | Middleware |
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
├── scripts/                            # Helper scripts
│   └── calibration_tool.py             # Camera calibration tool
├── config/                             # Config files
│   ├── camera_params.yaml              # Camera calibration parameters
│   ├── detector_config.yaml            # YOLO model configuration
│   ├── flight_params.yaml              # Flight controller parameters
│   └── ros_params.yaml                 # ROS node configurations
├── include/
│   ├── perception/
│   │   ├── detector.h                  # Object detection interface
│   │   └── depth.h                     # Stereo depth processing
│   ├── localization/
│   │   ├── fusion.h                    # Sensor fusion interface
│   │   ├── gps.h                       # Positioning system
│   │   └── imu.h                       # State estimation
│   ├── planning/
│   │   ├── path_planner.h              # Path planning interface
│   │   └── command_builder.h           # Mission manager
│   ├── control/
│   │   ├── commander.h                 # Flight command interface
│   │   ├── mavlink_bridge.h            # MAVLink communication
│   │   └── tele_op.h                   # Controller input handling
│   └── util/
|       ├── state.hpp                   # Pose, velocity, state info
│       └── math_funcs.hpp              # Misc math functions
├── src/
│   ├── perception/
│   │   ├── CMakeLists.txt
│   │   ├── detector.cpp
│   │   └── depth.cpp
│   ├── localization/
│   │   ├── CMakeLists.txt
│   │   ├── fusion.cpp
│   │   ├── gps.cpp
│   │   └── imu.cpp
│   ├── planning/
│   │   ├── CMakeLists.txt
│   │   ├── path_planner.cpp
│   │   └── command_builder.cpp
│   ├── control/
│   │   ├── CMakeLists.txt
│   │   ├── commander.cpp
│   │   ├── mavlink_bridge.cpp
│   │   └── tele_op.cpp
│   └── nodes/
│       ├── detector_node.cpp           # Raw image -> IDs and relative poses
│       ├── localizer_node.cpp          # Sensor data -> global position
│       ├── planner_node.cpp            # Position + target -> commands
│       └── control_node.cpp            # Commands -> pixhawk
├── launch/
│   └── main.launch                     # Main system startup
├── models/
│   ├── yolo/
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