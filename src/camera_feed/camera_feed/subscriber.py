import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraFeedSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_feed',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

    def listener_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_feed_subscriber = CameraFeedSubscriber()
    rclpy.spin(camera_feed_subscriber)
    camera_feed_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()