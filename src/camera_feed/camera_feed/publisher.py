import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import requests
from cv_bridge import CvBridge, CvBridgeError

class CameraFeedPublisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self.publisher_ = self.create_publisher(Image, '/camera_feed', 10)
        self.bridge = CvBridge()
        
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('camera_url', "xxxx")
        self.declare_parameter('webcam_device_id', 1)

        self.url = self.get_parameter('camera_url').get_parameter_value().string_value
        self.get_logger().info(f"Camera URL: {self.url}")

        
        self.use_webcam = self.get_parameter('use_webcam').get_parameter_value().bool_value
        
        if not self.use_webcam:
            self.url = self.get_parameter('camera_url').get_parameter_value().string_value
            self.stream = requests.get(self.url, stream=True)
            if self.stream.status_code != 200:
                self.get_logger().error("Error: Unable to connect to the video feed.")
            self.bytes_data = bytes()
        else:
            self.device_id = self.get_parameter('webcam_device_id').get_parameter_value().integer_value
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                self.get_logger().error(f"Error: Unable to open webcam (device ID: {self.device_id}).")
        
        self.timer = self.create_timer(0.1, self.fetch_and_publish_video_feed)


    def fetch_and_publish_video_feed(self):
        if self.use_webcam:
            ret, frame = self.cap.read()
            if ret:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.publisher_.publish(img_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f"CvBridge Error: {e}")
            else:
                self.get_logger().error("Failed to capture frame from webcam.")
        
        else:
            for chunk in self.stream.iter_content(chunk_size=1024):
                self.bytes_data += chunk
                a = self.bytes_data.find(b'\xff\xd8')
                b = self.bytes_data.find(b'\xff\xd9')

                if a != -1 and b != -1:
                    jpg = self.bytes_data[a:b+2]
                    self.bytes_data = self.bytes_data[b+2:]

                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        try:
                            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                            self.publisher_.publish(img_msg)
                        except CvBridgeError as e:
                            self.get_logger().error(f"CvBridge Error: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.use_webcam:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_feed_publisher = CameraFeedPublisher()
    rclpy.spin(camera_feed_publisher)
    camera_feed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()