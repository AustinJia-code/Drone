## Software
| Software   |Version           | Purpose             |
|------------|------------------|---------------------|
| Ubuntu     | 24.04.2, Desktop | Companion OS        |
| PX4        | 1.14             | Flight controller   |

| Library       | Purpose                 | Link                                                                      |
|---------------|-------------------------|---------------------------------------------------------------------------|
| ROS 2 "Jazzy" | Middleware              | [Docs](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html) |
| XRCE-DDS      | Companion comm. agent   | [GitHub](https://github.com/eProsima/Micro-XRCE-DDS-Agent)                |
| OpenCV        | Basic image processing  |
| YOLO          | Object detection        |

## Setup
1. Configure Pixhawk with QGroundControl

  |Parameter      |	Value   |	Description                     |
  |-----          |-----    |-----                            |
  | MAV_1_CONFIG  |	0	      | Disable MAVLink on TELEM2       |
  | SER_TEL2_BAUD |	921600  |	Set baud rate to match RPi      |
  | UXRCE_DDS_CFG |	telem2  |	Use TELEM2 for the RTPS client  |

2. Boot RPi5 with Ubuntu 24.04.2 , 64 bit
    - 22.xx.x is technically better supported by the libraries... but RPi5 doesn't like it!
    - I used the Desktop version due to networking issues with server
2. Install ROS2 Jazzy Jalisco: https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html
    - Jazzy technically isn't recommended by PX4 (Humble supported), but compatability :)
    - Follow, then stop when you reach "Build ROS 2", then: 
      ```
      sudo apt update && sudo apt install ros-jazzy-desktop
      echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      sudo apt install ros-jazzy-robot-localization
      ```
3. With only RPi5 plugged in, try following [Execution](#Execution) flow and install packages as necessary 
    - Test basic pub/sub launcher: ```ros2 launch test pubsub.launch.py```
4. Install micrortps_agent for flight computer communication:
    ```
    cd ..
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    git submodule update --init --recursive
    mkdir build && cd build
    cmake ..
    make
    ```
5. Test micrortps: ```./MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600```
    - if nothing is connected, it should say something like "1 port not found", working!
6. Open config: ```sudo nano /boot/firmware/config.txt```
7. Enable UART: ```enable_uart=1```
8. Reboot: ```sudo reboot```
9. Check that ```ls -l /dev/serial0``` points to ```/dev/ttyAMA0```
10. Verify wiring between RPi5 and Pixhawk

## Execution
Do once:
1. Set source:  ```echo "source ~/Drone/install/setup.bash" >> ~/.bashrc```
  - You may need to modify setup.bash path
2. cd src
3. git clone https://github.com/PX4/px4_msgs.git
4. cd ..

Build and Run:
1. Source: ```source ~/.bashrc```
2. colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
3. launch!
     - ros2 launch localization localization.launch.py
     - ros2 launch control control.launch.py
     - ros2 launch system_launch main.launch.py

## Misc
RPi temp check (try to stay under 85c):
```
awk '{print $1/1000 "°C"}' /sys/class/thermal/thermal_zone0/temp
```