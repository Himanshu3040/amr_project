#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from twist_mux_msgs.action import JoyTurbo
from rclpy.action import ActionServer


class TurboScaler(Node):
    def __init__(self):
        super().__init__('turbo_scaler')
        
        self.declare_parameter('linear_min',  rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('linear_max',  rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('angular_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('angular_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('steps',       rclpy.Parameter.Type.INTEGER)

        self.linear_min  = self.get_parameter('linear_min').value
        self.linear_max  = self.get_parameter('linear_max').value
        self.angular_min = self.get_parameter('angular_min').value
        self.angular_max = self.get_parameter('angular_max').value
        self.steps       = self.get_parameter('steps').value
        self.current_step = self.steps  # start at max speed

        self.sub = self.create_subscription(
            Twist, 'cmd_vel_combined', self.cmd_callback, 10
        )
        self.pub = self.create_publisher(
            Twist, 'amr_controller/cmd_vel_unstamped', 10
        )

        self._decrease_server = ActionServer(
            self, JoyTurbo, 'joy_turbo_decrease', self.decrease_callback
        )
        self._increase_server = ActionServer(
            self, JoyTurbo, 'joy_turbo_increase', self.increase_callback
        )

        self.get_logger().info(
            f'TurboScaler ready | steps={self.steps} | '
            f'linear={self.linear_min}~{self.linear_max} | '
            f'angular={self.angular_min}~{self.angular_max}'
        )

    def _current_linear(self):
        t = (self.current_step - 1) / max(self.steps - 1, 1)
        return self.linear_min + t * (self.linear_max - self.linear_min)

    def _current_angular(self):
        t = (self.current_step - 1) / max(self.steps - 1, 1)
        return self.angular_min + t * (self.angular_max - self.angular_min)

    def decrease_callback(self, goal_handle):
        self.current_step = max(1, self.current_step - 1)
        self.get_logger().info(
            f'Turbo DECREASE → step {self.current_step}/{self.steps} | '
            f'linear={self._current_linear():.2f} angular={self._current_angular():.2f}'
        )
        goal_handle.succeed()
        return JoyTurbo.Result()

    def increase_callback(self, goal_handle):
        self.current_step = min(self.steps, self.current_step + 1)
        self.get_logger().info(
            f'Turbo INCREASE → step {self.current_step}/{self.steps} | '
            f'linear={self._current_linear():.2f} angular={self._current_angular():.2f}'
        )
        goal_handle.succeed()
        return JoyTurbo.Result()

    def cmd_callback(self, msg: Twist):
        scaled = Twist()
        lin = self._current_linear()
        ang = self._current_angular()
        scaled.linear.x  = max(-lin, min(lin, msg.linear.x))
        scaled.angular.z = max(-ang, min(ang, msg.angular.z))
        self.pub.publish(scaled)


def main():
    rclpy.init()
    node = TurboScaler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()