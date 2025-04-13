"""
sensor nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
  # TODO: Replace this with your path!
  agent_path = os.path.expanduser('../Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')
  
  return LaunchDescription ([
    # Start the PX4 micrortps_agent
    ExecuteProcess (
      cmd=[agent_path,'serial','--dev','/dev/ttyAMA0','-b','921600'],
      output='screen'
    ),

    # IMU Node
    Node (
      package='sensor',
      executable='imu_bridge',
      name='imu_bridge',
      output='screen'
    ),

    # GPS Node
    Node(
      package='sensor',
      executable='gps_bridge',
      name='gps_bridge',
      output='screen'
    )
  ])
