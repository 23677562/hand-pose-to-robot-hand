import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
from custom_interfaces.msg import PoseKeypoints, PoseKeypoint
from geometry_msgs.msg import Point

class HandPoseEstimation(Node):
    
    def __init__(self):
        super().__init__('hand_pose_estimation')
        self.subscription = self.create_subscription(
            Image,
            '/camera_feed',
            self.listener_callback,
            10)
        
        self.declare_parameter('log_enabled', None)

        self.log_enabled = self.get_parameter('log_enabled').get_parameter_value().bool_value
        self.hand_landmarks_publisher = self.create_publisher(PoseKeypoints, 'hand_landmarks', 10)
        
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils

        self.landmark_names = [
            "WRIST", "THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP", "INDEX_FINGER_MCP", "INDEX_FINGER_PIP", "INDEX_FINGER_DIP",
            "INDEX_FINGER_TIP", "MIDDLE_FINGER_MCP", "MIDDLE_FINGER_PIP", "MIDDLE_FINGER_DIP", "MIDDLE_FINGER_TIP", "RING_FINGER_MCP",
            "RING_FINGER_PIP", "RING_FINGER_DIP", "RING_FINGER_TIP", "PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP"
        ]

        self.get_logger().info('Hand Pose Estimation node initialized')

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                landmarks_msg = PoseKeypoints()
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    keypoint_msg = PoseKeypoint()
                    keypoint_msg.keypoint = Point()
                    keypoint_msg.keypoint.x = landmark.x
                    keypoint_msg.keypoint.y = landmark.y
                    keypoint_msg.keypoint.z = landmark.z
                    keypoint_msg.visibility = landmark.visibility
                    keypoint_msg.landmark_name = self.landmark_names[idx]
                    landmarks_msg.keypoints.append(keypoint_msg)

                self.hand_landmarks_publisher.publish(landmarks_msg)
            
                if self.log_enabled:
                    self.get_logger().info(f"Hand Pose Estimation: {landmarks_msg}")

        cv2.imshow('Hand Pose Estimation', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    hand_pose_estimation = HandPoseEstimation()
    try:
        rclpy.spin(hand_pose_estimation)
    except KeyboardInterrupt:
        pass
    finally:
        hand_pose_estimation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()