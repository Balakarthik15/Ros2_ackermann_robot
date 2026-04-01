#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Changed import
import sys
import termios
import tty

class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')
        self.pub = self.create_publisher(
            Twist,                   # Changed message type
            '/cmd_vel',
            10
        )

        self.speed = 0.0
        self.steering = 0.0

        self.get_logger().info(
            "Ackermann Teleop Started (Using Twist)\n"
            "W/S : Forward / Backward\n"
            "A/D : Left / Right\n"
            "Space : Stop\n"
            "Ctrl+C to quit"
        )

        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_cmd(self):
        # Note: This get_key() approach inside a timer might feel "stuttery"
        # because it waits for a key press every 0.1s.
        key = self.get_key()

        if key == 'w':
            self.speed += 0.2
        elif key == 's':
            self.speed -= 0.2
        elif key == 'a':
            self.steering += 0.1
        elif key == 'd':
            self.steering -= 0.1
        elif key == ' ':
            self.speed = 0.0
            self.steering = 0.0
        elif key == '\x03': # Handles Ctrl+C properly
            rclpy.shutdown()
            sys.exit()

        # Create and fill the Twist message
        msg = Twist()
        msg.linear.x = self.speed      # Map speed to linear x
        msg.angular.z = self.steering   # Map steering to angular z
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = AckermannTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()