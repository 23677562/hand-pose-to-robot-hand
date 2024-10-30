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
 
    allegro_hand_gesture_control_node=Node(
        package = 'control',
        executable = 'allegro_hand_gesture_control'
    )

    ld.add_action(allegro_hand_gesture_control_node)

    return ld