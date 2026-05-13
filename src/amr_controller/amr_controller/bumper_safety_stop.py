import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math


class BumperSafetyStop(Node):

    def __init__(self):
        super().__init__('bumper_safety_stop_node')

        self.prev_stop = False

        # Declare parameters
        self.declare_parameter('bumper_safety_stop_topic', 'bumper_safety_stop')
        self.declare_parameter('front_bumper_joint', 'Front_Bumper_Joint')
        self.declare_parameter('rear_bumper_joint', 'Rear_Bumper_Joint')
        self.declare_parameter('press_threshold', 0.002)

        # Get parameters
        self.stop_topic = self.get_parameter(
            'bumper_safety_stop_topic').get_parameter_value().string_value

        self.front_joint_name = self.get_parameter(
            'front_bumper_joint').get_parameter_value().string_value

        self.rear_joint_name = self.get_parameter(
            'rear_bumper_joint').get_parameter_value().string_value

        self.press_threshold = self.get_parameter(
            'press_threshold').get_parameter_value().double_value

        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher
        self.safety_stop_pub = self.create_publisher(Bool, self.stop_topic, 10)

        self.get_logger().info(
            f"BumperSafetyStop started. Watching joints: "
            f"[{self.front_joint_name}, {self.rear_joint_name}] | "
            f"threshold: {self.press_threshold:.4f} m"
        )

    def joint_state_callback(self, msg: JointState):

        front_pressed = False
        rear_pressed = False

        for i in range(len(msg.name)):
            if msg.name[i] == self.front_joint_name:
                front_pressed = abs(msg.position[i]) > self.press_threshold

            elif msg.name[i] == self.rear_joint_name:
                rear_pressed = abs(msg.position[i]) > self.press_threshold

        any_pressed = front_pressed or rear_pressed

        if any_pressed != self.prev_stop:
            out = Bool()
            out.data = any_pressed
            self.safety_stop_pub.publish(out)

            self.get_logger().warn(
                f"Bumper state changed → "
                f"{'STOP (PRESSED)' if any_pressed else 'CLEAR'}  "
                f"[front={'PRESSED' if front_pressed else 'clear'} | "
                f"rear={'PRESSED' if rear_pressed else 'clear'}]"
            )

            self.prev_stop = any_pressed


def main(args=None):
    rclpy.init(args=args)
    node = BumperSafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()