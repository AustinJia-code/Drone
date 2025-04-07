"""
system launch
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description ():
    #Sensor Nodes
    sensor_launch = IncludeLaunchDescription (
        PythonLaunchDescriptionSource (
            os.path.join(
                get_package_share_directory('sensor'),
                'launch',
                'sensor.launch.py'
            )
        )
    )

    # Control Node
    control_node = Node (
        package='control',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    return LaunchDescription([
        # Start PX4 agent
        ExecuteProcess(
            cmd=['micrortps_agent', '-t', 'UDP', '-p', '2020'],
            shell=True,
            output='screen'
        ),
        
        sensor_launch,
        control_node
    ])
