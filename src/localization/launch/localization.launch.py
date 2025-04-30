"""
localization nodes
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
      package='localization',
      executable='imu_filter',
      name='imu_filter',
      output='screen'
    ),

    # GPS Node
    Node(
      package='localization',
      executable='gps_filter',
      name='gps_filter',
      output='screen'
    ),

    # ROS2 EKF node
    Node (
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=['config/ekf.yaml']
    ),

    # ROS2 GPS transform node
    Node (
      package='robot_localization',
      executable='navsat_transform_node',
      name='navsat_transform_node',
      output='screen',
      parameters=['config/navsat_config.yaml'],
      remappings=[
        ('imu', '/imu/data'),
        ('gps/fix', '/fix'),
        ('gps/vel', '/gps/vel'),
        ('odometry/gps', '/gps/filtered')
      ]
    )
  ])
