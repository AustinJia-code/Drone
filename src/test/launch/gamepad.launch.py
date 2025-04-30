"""
gamepad nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description ():
  return LaunchDescription ([
    # Joystick Publisher Node
    Node (
      package='joy',
      executable='joy_node',
      name='joy_node',
      output='screen',

      # Joystick location
      parameters = [{
        'dev': '/dev/input/js0'
      }]
    ),

    # Custom Joy Listener Node
    Node (
      package='test',
      executable='test_joy',
      name='test_joy',
      output='screen'
    )
  ])
