### LIKELY UNNEEDED, PX4 HAS ITS OWN LOCALIZATION ###

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  pkg_localization = FindPackageShare('localization')

  ekf_path = PathJoinSubstitution([pkg_localization, 'launch', 'config', 'ekf.yaml'])
  navsat_path = PathJoinSubstitution([pkg_localization, 'launch', 'config', 'navsat_config.yaml'])

  agent_path = 'Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'

  return LaunchDescription([
    ExecuteProcess(
      cmd=[agent_path, 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
      output='screen'
    ),

    Node(
      package='localization',
      executable='imu_filter',
      name='imu_filter',
      output='screen'
    ),

    Node(
      package='localization',
      executable='gps_filter',
      name='gps_filter',
      output='screen'
    ),

    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[ekf_path]
    ),

    Node(
      package='robot_localization',
      executable='navsat_transform_node',
      name='navsat_transform_node',
      output='screen',
      parameters=[navsat_path],
      remappings=[
          ('imu', '/imu/data'),
          ('gps/fix', '/fix'),
          ('gps/vel', '/gps/vel'),
          ('odometry/gps', '/gps/filtered')
      ]
    )
])
