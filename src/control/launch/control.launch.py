"""
control nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description ():
  # TODO: Fix this path!
  agent_path = os.path.expanduser('../Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')

  return LaunchDescription ([
    # Start the PX4 micrortps_agent
    ExecuteProcess (
      cmd=[agent_path,'serial','--dev','/dev/ttyAMA0','-b','921600'],
      output='screen'
    ),

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
    
    # Control Node
    Node (
      package = 'control',
      executable = 'drive_node',
      name = 'drive_node',
      output = 'screen'
    )
  ])
