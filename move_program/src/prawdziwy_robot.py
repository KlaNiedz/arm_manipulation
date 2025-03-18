#!/usr/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic_based_joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_state = JointState()

        self.subscription = self.create_subscription(
            JointState,
            'topic_based_joint_commands',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        self.joint_state = msg

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = 'base_link'
        self.publisher_.publish(self.joint_state)


      

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()


    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
