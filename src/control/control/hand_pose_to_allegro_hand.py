import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_interfaces.msg import PoseKeypoints
import numpy as np

class HandPoseToAllegroHand(Node):

    def __init__(self):
        super().__init__('hand_pose_to_allegro_hand')

        self.declare_parameter('robot_config_path', '')

        robot_config_path = self.get_parameter('robot_config_path').get_parameter_value().string_value

        self.get_logger().info(f'Robot config path: {robot_config_path}')

        with open(robot_config_path, 'r') as config_file:
            self.joint_config = json.load(config_file)

        self.subscription = self.create_subscription(
            PoseKeypoints,
            '/hand_landmarks',
            self.listener_callback,
            10
        )
        
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info('Hand Pose to Joint State node initialized')

    def listener_callback(self, msg):
        landmarks = msg.keypoints
        landmarks_dict = {landmark.landmark_name: landmark.keypoint for landmark in landmarks}

        joint_angles, joint_names = self.calculate_joint_angles(landmarks_dict)

        self.publish_joint_states(joint_angles, joint_names)

    def calculate_angle(self, v1, v2):
        cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        return angle

    def calculate_joint_angles(self, landmarks):
        joint_angles = []
        joint_names = []

        for joint in self.joint_config['joints']:
            try:
                base_point = np.array([landmarks[joint['landmarks'][0]].x, landmarks[joint['landmarks'][0]].y, landmarks[joint['landmarks'][0]].z])
                self.get_logger().info(f'Base point: {base_point}')
                middle_point = np.array([landmarks[joint['landmarks'][1]].x, landmarks[joint['landmarks'][1]].y, landmarks[joint['landmarks'][1]].z])
                self.get_logger().info(f'Middle point: {middle_point}')
                tip_point = np.array([landmarks[joint['landmarks'][2]].x, landmarks[joint['landmarks'][2]].y, landmarks[joint['landmarks'][2]].z])
                self.get_logger().info(f'Tip point: {tip_point}')
  
                vector1 = middle_point - base_point
                self.get_logger().info(f'Vector 1: {vector1}')
                vector2 = tip_point - middle_point
                self.get_logger().info(f'Vector 2: {vector2}')

                angle = self.calculate_angle(vector1, vector2)

                clipped_angle = np.clip(angle, joint['min_angle'], joint['max_angle'])

                joint_angles.append(clipped_angle)
                joint_names.append(joint['joint_name'])

                self.get_logger().debug(f'Joint {joint["joint_name"]}: Calculated angle = {angle}, Clipped angle = {clipped_angle}')
            except KeyError as e:
                self.get_logger().error(f"Error: Landmark '{e.args[0]}' not found in landmarks dictionary.")
                continue

        return joint_angles, joint_names


    def publish_joint_states(self, joint_angles, joint_names):
        joint_state_msg = JointState()
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_angles

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {joint_state_msg.position}')

def main(args=None):
    rclpy.init(args=args)
    hand_pose_to_allegro_hand = HandPoseToAllegroHand()
    try:
        rclpy.spin(hand_pose_to_allegro_hand)
    except KeyboardInterrupt:
        pass
    finally:
        hand_pose_to_allegro_hand.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()