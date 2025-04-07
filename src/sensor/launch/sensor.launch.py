"""
sensor nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
  return LaunchDescription([
    # Start the PX4 micrortps_agent
    ExecuteProcess(
      cmd=['micrortps_agent', '-t', 'UDP', '-p', '2020'],
      shell=True,
      output='screen'
    ),

    # Sensor Node
    Node(
      package='sensor',
      executable='sensor_node',
      name='sensor_node',
      output='screen'
    )
  ])
