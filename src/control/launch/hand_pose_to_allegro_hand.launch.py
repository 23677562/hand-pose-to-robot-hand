import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'params.yaml'
        )
        
    hand_pose_to_allegro_hand_node=Node(
        package = 'control',
        executable = 'hand_pose_to_allegro_hand',
        name = 'hand_pose_to_allegro_hand',
        parameters = [config]
    )

    ld.add_action(hand_pose_to_allegro_hand_node)

    return ld