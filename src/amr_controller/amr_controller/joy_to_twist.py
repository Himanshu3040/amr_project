#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('deadman_button', 0)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)

        self.linear_axis    = self.get_parameter('linear_axis').value
        self.angular_axis   = self.get_parameter('angular_axis').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.linear_scale   = self.get_parameter('linear_scale').value
        self.angular_scale  = self.get_parameter('angular_scale').value

        self.pub = self.create_publisher(Twist, 'joy_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.get_logger().info('joy_to_twist node started')

    def joy_cb(self, msg: Joy):
        if msg.buttons[self.deadman_button] != 1:
            return
        twist = Twist()
        twist.linear.x  = msg.axes[self.linear_axis]  * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
