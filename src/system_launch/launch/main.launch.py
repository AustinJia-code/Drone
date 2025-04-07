"""
system launch
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Optional: Start PX4 agent (adjust to your setup)
        ExecuteProcess(
            cmd=['micrortps_agent', '-t', 'UDP', '-p', '2020'],
            shell=True,
            output='screen'
        ),

        # Control Node (from `control` package)
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen'
        ),

        # Sensor Node (from `sensor` package)
        Node(
            package='sensor',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        )
    ])
