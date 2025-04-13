## Software
| Software   |Version           | Purpose             |
|------------|------------------|---------------------|
| Ubuntu     | 24.04.2, Desktop | Companion OS        |
| PX4        | 1.14             | Flight controller   |

| Library       | Purpose                 | Link                                                                      |
|---------------|-------------------------|---------------------------------------------------------------------------|
| ROS 2 "Jazzy" | Middleware              | (Docs)[https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html] |
| XRCE-DDS      | Companion comm. agent   | (GitHub)[https://github.com/eProsima/Micro-XRCE-DDS-Agent]                |
| OpenCV        | Basic image processing  |
| YOLO          | Object detection        |

## Setup
1. Configure Pixhawk with QGroundControl
    |Parameter|	Value|	Description|
    |---|---|---|
    |MAV_1_CONFIG|	0	|Disable MAVLink on TELEM2|
    |SER_TEL2_BAUD|	921600|	Set baud rate to match RPi|
    |UXRCE_DDS_CFG|	telem2|	Use TELEM2 for the RTPS client|
2. Boot RPi5 with Ubuntu 24.04.2 , 64 bit
    - 22.xx.x is technically better supported by the libraries... but RPi5 doesn't like it!
    - I used the Desktop version due to networking issues with server
2. Install ROS2 Jazzy Jalisco: https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html
    - Jazzy technically isn't recommended by PX4 (Humble supported), but compatability :)
    - Follow to "Build ROS 2", then: 
      ```
      sudo apt update && sudo apt install ros-jazzy-desktop
      echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      ```
3. With only RPi5 plugged in, try following [Execution](#Execution) flow and install packages as necessary 
    - Test basic pub/sub launcher: ```ros2 launch test pubsub.launch.py```
4. Install micrortps_agent:
    ```
    cd ~
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    git submodule update --init --recursive
    mkdir build && cd build
    cmake ..
    make
    ```
5. Test micrortps: ```./MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600```
    - if nothing connected, should say something like "1 port not found", working!
6. Search this repository for "# TODO:" and take care of business
7. Open config: ```sudo nano /boot/firmware/config.txt```
8. Enable UART, disable bluetooth:
    ```
    enable_uart=1
    dtoverlay=disable-bt
    ```
9. Reboot: ```sudo reboot```
10. Check that ```ls -l /dev/serial0``` points to ```/dev/ttyAMA0```
11. Verify wiring between RPi5 and Pixhawk

## Execution
1. cd src
2. git clone https://github.com/PX4/px4_msgs.git
3. cd ..
4. colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
5. Set source:  ```echo "source ~/Projects/Drone/install/setup.bash" >> ~/.bashrc```
    - You may need to modify setup.bash path
6. Source: ```source ~/.bashrc```
7. launch!
     - ros2 launch sensor sensor.launch.py
     - ros2 launch control control.launch.py
     - ros2 launch system_launch main.launch.py

## Design
**Software Flow**
1. RPi5 processes image from camera using OpenCV
2. RPi5 sends data to Coprocessor
3. Coprocessor processes data with YOLO
4. Coprocessor sends data to RPi5
5. RPi5 uses recognitions and camera depth info for relative position to target
6. Pixhawk sends IMU and GPS information to RPi5
7. RPi5 uses IMU and GPS for global position estimation
8. RPi5 decides a path/direction and generates commands to follow it
9. RPi5 sends commands to Pixhawk using MAVRos
10. Pixhawk sends control information to ESCs

## Misc
RPi temp check (try to stay under 85c):
```
awk '{print $1/1000 "°C"}' /sys/class/thermal/thermal_zone0/temp
```