import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SafetyArbitrator(Node):

    def __init__(self):
        super().__init__('safety_arbitrator_node')

        self.lidar_stop = False
        self.ultrasonic_stop = False
        self.bumper_stop = False

        # Declare parameters
        self.declare_parameter('lidar_safety_stop_topic', 'lidar_safety_stop')
        self.declare_parameter('ultrasonic_safety_stop_topic', 'ultrasonic_safety_stop')
        self.declare_parameter('bumper_safety_stop_topic', 'bumper_safety_stop')
        self.declare_parameter('safety_stop_topic', 'safety_stop')

        # Get parameters
        lidar_topic = self.get_parameter(
            'lidar_safety_stop_topic').get_parameter_value().string_value

        ultrasonic_topic = self.get_parameter(
            'ultrasonic_safety_stop_topic').get_parameter_value().string_value

        bumper_topic = self.get_parameter(
            'bumper_safety_stop_topic').get_parameter_value().string_value

        output_topic = self.get_parameter(
            'safety_stop_topic').get_parameter_value().string_value

        # Subscribers
        self.lidar_sub = self.create_subscription(
            Bool, lidar_topic, self.lidar_callback, 10)

        self.ultrasonic_sub = self.create_subscription(
            Bool, ultrasonic_topic, self.ultrasonic_callback, 10)

        self.bumper_sub = self.create_subscription(
            Bool, bumper_topic, self.bumper_callback, 10)

        # Publisher
        self.safety_stop_pub = self.create_publisher(Bool, output_topic, 10)

        self.get_logger().info("SafetyArbitrator started.")
        self.get_logger().info(
            f"Listening to: {lidar_topic}, {ultrasonic_topic}, {bumper_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")

    def publish_arbitrated_stop(self):
        combined = self.lidar_stop or self.ultrasonic_stop or self.bumper_stop

        msg = Bool()
        msg.data = combined
        self.safety_stop_pub.publish(msg)

        self.get_logger().info(
            f"Safety arbitration -> {'STOP' if combined else 'CLEAR'}  "
            f"[lidar={'STOP' if self.lidar_stop else 'clear'} | "
            f"ultrasonic={'STOP' if self.ultrasonic_stop else 'clear'} | "
            f"bumper={'STOP' if self.bumper_stop else 'clear'}]"
        )

    def lidar_callback(self, msg: Bool):
        if msg.data != self.lidar_stop:
            self.lidar_stop = msg.data
            self.publish_arbitrated_stop()

    def ultrasonic_callback(self, msg: Bool):
        if msg.data != self.ultrasonic_stop:
            self.ultrasonic_stop = msg.data
            self.publish_arbitrated_stop()

    def bumper_callback(self, msg: Bool):
        if msg.data != self.bumper_stop:
            self.bumper_stop = msg.data
            self.publish_arbitrated_stop()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyArbitrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()