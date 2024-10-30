import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
      get_package_share_directory('camera_feed'),
      'config',
      'params.yaml'
      )
    
    node = Node(
        package='camera_feed',
        name='publisher',
        executable='publisher',
        parameters=[config]
    )

    ld.add_action(node)
    return ld
