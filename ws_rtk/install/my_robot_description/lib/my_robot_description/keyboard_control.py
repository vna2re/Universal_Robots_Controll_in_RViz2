#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
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
        self.active_joint = 0

        self.get_logger().info("Use arrow keys: ←/→ to switch joint, ↑/↓ to move. Press 'q' to quit.")
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x1b':  # Arrow keys start with ESC
                    key += sys.stdin.read(2)  # Read the next two chars
                return key
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    break
                elif key == '\x1b[C':  # →
                    self.active_joint = (self.active_joint + 1) % 6
                    self.get_logger().info(f"Selected joint {self.active_joint}: {self.joint_state.name[self.active_joint]}")
                elif key == '\x1b[D':  # ←
                    self.active_joint = (self.active_joint - 1) % 6
                    self.get_logger().info(f"Selected joint {self.active_joint}: {self.joint_state.name[self.active_joint]}")
                elif key == '\x1b[A':  # ↑
                    self.joint_state.position[self.active_joint] += 0.05
                    self.get_logger().info(f"{self.joint_state.name[self.active_joint]}: {self.joint_state.position[self.active_joint]:.2f}")
                elif key == '\x1b[B':  # ↓
                    self.joint_state.position[self.active_joint] -= 0.05
                    self.get_logger().info(f"{self.joint_state.name[self.active_joint]}: {self.joint_state.position[self.active_joint]:.2f}")

                self.joint_state.header.stamp = self.get_clock().now().to_msg()
                self.joint_pub.publish(self.joint_state)

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            self.get_logger().info("Shutting down node")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.destroy_node()

if __name__ == '__main__':
    main()

