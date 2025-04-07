## Execution
1. cd src
2. git clone https://github.com/PX4/px4_msgs.git
3. cd ..
1. colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
2. source install/setup.bash
3. launch!
     - ros2 launch sensor sensor.launch.py
     - ros2 launch control control.launch.py
     - ros2 launch system_launch main.launch.py
     - ros2 launch test test.launch.py

## Setup
1. Install PX4 on Pixhawk
2. Install Ubuntu 22.04.5 Jammy Jellyfish on RPi5 
     - Set up SSH
3. Follow: https://docs.px4.io/main/en/ros2/offboard_control.html 

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
2. RPi5 sends data to Coprocessor
3. Coprocessor processes data with YOLO
4. Coprocessor sends data to RPi5
5. RPi5 uses recognitions and camera depth info for relative position to target
6. Pixhawk sends IMU and GPS information to RPi5
7. RPi5 uses imu and GPS for global position estimation
8. RPi5 decides a path and generates commands to follow it
9. RPi5 sends commands to Pixhawk using MAVRos
10. Pixhawk sends control information to ESCs