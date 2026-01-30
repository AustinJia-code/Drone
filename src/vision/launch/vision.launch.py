from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description ():
    return LaunchDescription ([
        Node (
            package='vision',
            executable='red_detector_node',
            name='red_detector_node',
            output='screen'
        )
    ])
