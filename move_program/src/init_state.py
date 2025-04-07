#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def main():
    rclpy.init()
    node = Node("initial_joint_publisher")
    pub = node.create_publisher(JointState, "/joint_states", 10)
    
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_gripper_left', 'joint_gripper_right']
    msg.position = [0.0, -0.2881, 1.8, 0.0, 0.0086, 0.0, 0.0, 0.0]
    
    # Publikuj przez kilka sekund, aby RViz odebrał wiadomość
    for _ in range(10):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()