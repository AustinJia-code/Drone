"""
control nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description ():
  # TODO: Fix this path!
  agent_path = os.path.expanduser('~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')

  return LaunchDescription ([
    # Start the PX4 micrortps_agent
    ExecuteProcess (
      cmd=[agent_path,'serial','--dev','/dev/ttyAMA0','-b','921600'],
      output='screen'
    ),

    # Control Node
    Node (
      package = 'control',
      executable = 'control_node',
      name = 'control_node',
      output = 'screen'
    )
  ])
