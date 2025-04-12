## Setup
1. Configure Pixhawk with QGroundControl
2. Boot RPi5 with Ubuntu 24.04.2 , 64 bit
    - 22.xx.x is technically better supported by the libraries... but RPi5 doesn't like it!
3. With only RPi5 plugged in, try following [Execution](#Execution) flow and install packages as necessary 
    - Some things you may need: git, colcon, cmake, ROS2 Jazzy
    - Again, Jazzy technically isn't recommended by PX4, but compatability :)
4. Test basic pub/sub: 
    ```
    source install/setup.bash
    ros2 launch test test.launch.py
    ```
5. Open config: ```sudo nano /boot/firmware/config.txt```
6. Enable UART, disable bluetooth:
    ```
    enable_uart=1
    dtoverlay=disable-bt
    ```
7. Reboot: ```sudo reboot```
8. Check that ```ls -l /dev/serial0``` points to ```/dev/ttyAMA0```
9. Verify wiring between RPi5 and Pixhawk

## Execution
1. cd src
2. git clone https://github.com/PX4/px4_msgs.git
3. cd ..
4. colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
5. source install/setup.bash
6. launch!
     - ros2 launch sensor sensor.launch.py
     - ros2 launch control control.launch.py
     - ros2 launch system_launch main.launch.py
     - ros2 launch test test.launch.py

## Software
| Software | Purpose |
|----------|---------|
| PX4 1.14 | Flight controller |

| Library | Purpose |
|---------|---------|
| ROS 2 "Jazzy" | Middleware |
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
