#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math

class DualControl(Node):
    def __init__(self):
        super().__init__('dual_control')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_state = JointState()
        self.joint_state.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.joint_state.position = [0.0] * 6

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        # Пример: используем оси 0 и 1 для вращения первого и второго сустава
        self.joint_state.position[0] += 0.05 * axes[0]  # левый стик по горизонтали
        self.joint_state.position[1] += 0.05 * axes[1]  # левый стик по вертикали
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

