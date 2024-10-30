import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__('gesture_recognition')
        self.subscription = self.create_subscription(
            Image,
            '/camera_feed',
            self.listener_callback,
            10)
        
        self.declare_parameter('log_enabled', None)

        self.log_enabled = self.get_parameter('log_enabled').get_parameter_value().bool_value
        self.gesture_publisher = self.create_publisher(String, 'recognized_gesture', 10)
        self.bridge = CvBridge()
        self.mp_drawing = mp.solutions.drawing_utils

        base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
        running_mode = vision.GestureRecognizerOptions.running_mode.LIVE_STREAM

        def result_callback(result, image, timestamp_ms):
            self.recognition_result = result
            self.process_recognition_result(image.numpy_view())

        options = vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=running_mode,
            result_callback=result_callback
        )
        self.recognizer = vision.GestureRecognizer.create_from_options(options)
        self.timestamp_counter = 0
        self.get_logger().info('Gesture Recognition node initialized')

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
        self.timestamp_counter += 1
        self.recognizer.recognize_async(image_mp, self.timestamp_counter)

    def process_recognition_result(self, image_rgb):
        gesture_recogniser = self.recognition_result

        if gesture_recogniser.gestures:
            for gesture in gesture_recogniser.gestures:
                gesture_string = f"{gesture[0].category_name}"
                if gesture_string != 'None': 
                    gesture_msg = String() 
                    gesture_msg.data = gesture_string
                    self.gesture_publisher.publish(gesture_msg)
                    if self.log_enabled:
                        self.get_logger().info(f"Recognized gesture: {gesture_string}")
        else:
            if self.log_enabled:
                self.get_logger().warn('No hand landmarks detected')
                
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    gesture_recognition_node = GestureRecognitionNode()
    try:
        rclpy.spin(gesture_recognition_node)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_recognition_node.shutdown()

if __name__ == '__main__':
    main()