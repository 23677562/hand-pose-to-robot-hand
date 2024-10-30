import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

class AllegroHandGestureControl(Node):
    def __init__(self):
        super().__init__('allegro_hand_gesture_control')
        self.subscription = self.create_subscription(
            String,
            'recognized_gesture',
            self.hand_gesture_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/joint_command', QoSProfile(depth=10))
        self.closed_hand_command = JointState()
        self.closed_hand_command.name = ['index_joint_0', 'middle_joint_0', 'ring_joint_0', 'thumb_joint_0', 'index_joint_1', 'middle_joint_1', 'ring_joint_1', 'thumb_joint_1', 'index_joint_2', 'middle_joint_2', 'ring_joint_2', 'thumb_joint_2', 'index_joint_3', 'middle_joint_3', 'ring_joint_3', 'thumb_joint_3']
        self.closed_hand_command.position = [0.5585, 0.5585, 0.5585, 1.5707, 1.7278, 1.7278, 1.7278, 1.1519, 1.7278, 1.7278, 1.7278, 1.7278, 1.7278, 1.7278, 1.7278, 1.7627]
        
        self.open_hand_command = JointState()
        self.open_hand_command.name = ['index_joint_0', 'middle_joint_0', 'ring_joint_0', 'thumb_joint_0', 'index_joint_1', 'middle_joint_1', 'ring_joint_1', 'thumb_joint_1', 'index_joint_2', 'middle_joint_2', 'ring_joint_2', 'thumb_joint_2', 'index_joint_3', 'middle_joint_3', 'ring_joint_3', 'thumb_joint_3']
        self.open_hand_command.position = [-0.6585, -0.6585, -0.6585, 0.1793, -0.37950000000000006, -0.3789, -0.3789, -0.4316, -0.37950000000000006, -0.37870000000000004, -0.3788, -0.37950000000000006, -0.3792, -0.3792, -0.3792, -0.3792]

    def hand_gesture_callback(self, msg):
        self.gesture = msg.data
        if self.gesture == 'Closed_Fist':
            self.close_hand()
        
        elif self.gesture == 'Open_Palm':
            self.open_hand()

    def close_hand(self):
        self.publisher.publish(self.closed_hand_command)
        self.get_logger().info('Publishing new JointState message with updated timestamp')
        self.get_logger().info('Hand closed')

    def open_hand(self):
        self.publisher.publish(self.open_hand_command)
        self.get_logger().info('Publishing new JointState message with updated timestamp')
        self.get_logger().info('Hand opened')

def main(args=None):
    rclpy.init(args=args)
    hand_gesture_subscriber = AllegroHandGestureControl()
    rclpy.spin(hand_gesture_subscriber)
    hand_gesture_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()