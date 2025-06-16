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
| LibCamera     | Camera "driver "        | [Wiki](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera#Working_with_Raspberry_Pi_5_.28libcamera.29)  |

## Setup
1. Disable RC controller parameter in QGroundControl
    ```
    COM_RC_IN_MODE = 4
    ```
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
      ```
3. Clone PX4
    ```
    cd src
    git clone https://github.com/PX4/px4_msgs.git
    ```
4. With only RPi5 plugged in, try following [Execution](#Execution) flow and install packages as necessary 
    - Test basic pub/sub launcher: ```ros2 launch test pubsub.launch.py```
5. Install micrortps_agent for flight computer communication:
    ```
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    git submodule update --init --recursive
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    ```
6. Test micrortps: ```./MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600```
    - if nothing is connected, it should say something like "1 port not found", working!
7. Change QGroundControl parameters, then reboot:
    ```
    MAV_1_CONFIG = TELEM2
    UXRCE_DDS_CFG = 0 (Disabled)
    SER_TEL2_BAUD = 57600
    ```
8. Verify wiring between RPi5 and Pixhawk
9. Install MAVProxy:
    ```
    sudo apt install pipx
    pipx install MAVProxy
    sudo apt remove modemmanager
    ```
10. Run MAVProxy:
    ```
    mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600
    ```
    - if ttyUSB0 is busy, run ```sudo lsof /dev/ttyUSB0``` and kill the active PIDs
    - You should now see a heartbeat :)
11. Change QGroundControl parameters, then reboot:
    ```
    MAV_1_CONFIG = 0 (Disabled)
    UXRCE_DDS_CFG = 102 (TELEM2)
    SER_TEL2_BAUD = 921600
    ```
12. At this point, basic setup should be complete!
    ```
    ros2 launch test sensor.launch.py
    ```
    - If still issues, read through the original docs [here](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html) and [here](https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html)

## Execution
Do once:
1. Set source:  ```echo "source ~/Drone/install/setup.bash" >> ~/.bashrc```
  - You may need to modify setup.bash path
2. Source: ```source ~/.bashrc```
3. colcon build --packages-select px4_msgs

Build and Run:
1. colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
2. launch!
     - ros2 launch test sensor.launch.py
     - ros2 launch system_launch main.launch.py

## Misc
RPi temp check (try to stay under 85c):
```
awk '{print $1/1000 "°C"}' /sys/class/thermal/thermal_zone0/temp
```
