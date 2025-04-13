"""
control nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description ():
  return LaunchDescription ([
    # Publisher Node
    Node (
      package='test',
      executable='test_subscriber',
      name='test_subscriber',
      output='screen'
    ),

    # Subscriber Node
    Node (
      package='test',
      executable='test_publisher',
      name='test_publisher',
      output='screen'
    )
  ])
