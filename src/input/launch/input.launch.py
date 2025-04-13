"""
input nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
  return LaunchDescription([
    # Control Node
    Node(
      package='input',
      executable='gamepad_bridge',
      name='gamepad_bridge',
      output='screen'
    )
  ])
