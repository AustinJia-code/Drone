### LIKELY UNNEEDED, PX4 HAS ITS OWN LOCALIZATION ###

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description ():
    Node (
      package='vision',
      executable='camera_listener',
      name='camera_listener',
      output='screen'
    )
