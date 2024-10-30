import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('vision_processing'),
        'config',
        'params.yaml'
        )
        
    hand_pose_estimation_node=Node(
        package = 'vision_processing',
        executable = 'hand_pose_estimation',
        name = 'hand_pose_estimation',
        parameters = [config]
    )

    ld.add_action(hand_pose_estimation_node)

    return ld