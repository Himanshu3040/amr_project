import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from amr_interfaces.msg import UltrasonicRaw, UltrasonicZones


class UltrasonicSafetyStop(Node):

    def __init__(self):
        super().__init__('ultrasonic_safety_stop_node')

        # Declare parameters
        self.declare_parameter('front_protective_distance_mm', 250.0)
        self.declare_parameter('rear_protective_distance_mm', 130.0)
        self.declare_parameter('ultrasonic_raw_topic', 'ultrasonic_raw')
        self.declare_parameter('ultrasonic_safety_stop_topic', 'ultrasonic_safety_stop')
        self.declare_parameter('ultrasonic_zones_topic', 'ultrasonic_zones')

        # Get parameters
        self.front_threshold_mm = self.get_parameter(
            'front_protective_distance_mm').get_parameter_value().double_value

        self.rear_threshold_mm = self.get_parameter(
            'rear_protective_distance_mm').get_parameter_value().double_value

        raw_topic = self.get_parameter(
            'ultrasonic_raw_topic').get_parameter_value().string_value

        stop_topic = self.get_parameter(
            'ultrasonic_safety_stop_topic').get_parameter_value().string_value

        zones_topic = self.get_parameter(
            'ultrasonic_zones_topic').get_parameter_value().string_value

        self.get_logger().info(
            f"UltrasonicSafetyStop started. Front threshold: {self.front_threshold_mm:.1f} mm | "
            f"Rear threshold: {self.rear_threshold_mm:.1f} mm"
        )

        # Subscriber
        self.ultrasonic_raw_sub = self.create_subscription(
            UltrasonicRaw,
            raw_topic,
            self.ultrasonic_callback,
            10
        )

        # Publishers
        self.safety_stop_pub = self.create_publisher(Bool, stop_topic, 10)
        self.zones_pub = self.create_publisher(UltrasonicZones, zones_topic, 10)

        # Initial state
        self.prev_any_protective = False

    def ultrasonic_callback(self, msg: UltrasonicRaw):

        thresholds = [
            self.front_threshold_mm,  # Front Left
            self.front_threshold_mm,  # Front Right
            self.rear_threshold_mm,   # Rear Left
            self.rear_threshold_mm   # Rear Right
        ]

        zones_msg = UltrasonicZones()
        zones_msg.header = msg.header

        any_protective = False

        for i in range(4):
            dist = msg.distances_mm[i]
            zones_msg.distances_mm[i] = dist

            # Same logic as C++
            if dist > 0.0 and dist <= thresholds[i]:
                zones_msg.zones[i] = UltrasonicZones.ZONE_PROTECTIVE
                any_protective = True

                self.get_logger().warn(
                    f"Sensor [{i}] in PROTECTIVE zone: {dist:.1f} mm "
                    f"(threshold: {thresholds[i]:.1f} mm)"
                )
            else:
                zones_msg.zones[i] = UltrasonicZones.ZONE_CLEAR

        # Overall robot zone
        if any_protective:
            zones_msg.robot_zone = UltrasonicZones.ZONE_PROTECTIVE
        else:
            zones_msg.robot_zone = UltrasonicZones.ZONE_CLEAR

        # Always publish zones
        self.zones_pub.publish(zones_msg)

        # Publish stop ONLY on change
        if any_protective != self.prev_any_protective:
            stop_msg = Bool()
            stop_msg.data = any_protective
            self.safety_stop_pub.publish(stop_msg)

            self.get_logger().info(
                f"Ultrasonic safety state changed → "
                f"{'STOP (PROTECTIVE zone)' if any_protective else 'CLEAR'}"
            )

            self.prev_any_protective = any_protective


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    