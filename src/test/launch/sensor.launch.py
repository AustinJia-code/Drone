"""
sensor test nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
  # TODO: Replace this with your path!
  agent_path = os.path.expanduser('Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')
  
  return LaunchDescription ([
    # Start the PX4 micrortps_agent
    ExecuteProcess (
      cmd=[agent_path,'serial','--dev','/dev/ttyUSB0','-b','921600'],
      output='screen'
    ),

    # IMU Node
    Node (
      package='test',
      executable='test_imu',
      name='test_imu',
      output='screen'
    )
  ])