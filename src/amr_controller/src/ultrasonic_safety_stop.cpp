#include <string>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "amr_interfaces/msg/ultrasonic_raw.hpp"
#include "amr_interfaces/msg/ultrasonic_zones.hpp"

using std::placeholders::_1;

/*
 * UltrasonicSafetyStop Node
 *
 * Subscribes to raw ultrasonic distances published by the ethernet bridge node.
 * Classifies each sensor reading into a zone (CLEAR or PROTECTIVE) based on
 * direction-specific thresholds, then publishes the result.
 *
 * Sensor index mapping (matches UltrasonicRaw.msg):
 *   Index 0 = Front Left   → protective threshold: 250 mm
 *   Index 1 = Front Right  → protective threshold: 250 mm
 *   Index 2 = Rear Left    → protective threshold: 130 mm
 *   Index 3 = Rear Right   → protective threshold: 130 mm
 *
 * Published topics:
 *   /ultrasonic_safety_stop  [std_msgs/Bool]              → consumed by safety_arbitrator
 *   /ultrasonic_zones        [amr_interfaces/UltrasonicZones] → consumed by diagnostics
 */

class UltrasonicSafetyStop : public rclcpp::Node
{
public:
  UltrasonicSafetyStop()
  : Node("ultrasonic_safety_stop_node")
  {
    // Declare parameters with default threshold values.
    // These can be overridden in a launch file or YAML config.
    declare_parameter<double>("front_protective_distance_mm", 250.0);
    declare_parameter<double>("rear_protective_distance_mm", 130.0);
    declare_parameter<std::string>("ultrasonic_raw_topic", "ultrasonic_raw");
    declare_parameter<std::string>("ultrasonic_safety_stop_topic", "ultrasonic_safety_stop");
    declare_parameter<std::string>("ultrasonic_zones_topic", "ultrasonic_zones");

    front_threshold_mm_ = get_parameter("front_protective_distance_mm").as_double();
    rear_threshold_mm_  = get_parameter("rear_protective_distance_mm").as_double();

    std::string raw_topic   = get_parameter("ultrasonic_raw_topic").as_string();
    std::string stop_topic  = get_parameter("ultrasonic_safety_stop_topic").as_string();
    std::string zones_topic = get_parameter("ultrasonic_zones_topic").as_string();

    RCLCPP_INFO(get_logger(),
      "UltrasonicSafetyStop started. Front threshold: %.1f mm | Rear threshold: %.1f mm",
      front_threshold_mm_, rear_threshold_mm_);

    // Subscriber: raw distances from STM32 via ethernet bridge
    ultrasonic_raw_sub_ = create_subscription<amr_interfaces::msg::UltrasonicRaw>(
      raw_topic, 10,
      std::bind(&UltrasonicSafetyStop::ultrasonicCallback, this, _1));

    // Publisher: simple stop/go signal for the safety arbitrator
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic, 10);

    // Publisher: detailed zone info for diagnostics/visualization
    zones_pub_ = create_publisher<amr_interfaces::msg::UltrasonicZones>(zones_topic, 10);

    // Initialize previous state to CLEAR so first message is always published
    prev_any_protective_ = false;
  }

private:
  double front_threshold_mm_;
  double rear_threshold_mm_;
  bool prev_any_protective_;

  rclcpp::Subscription<amr_interfaces::msg::UltrasonicRaw>::SharedPtr ultrasonic_raw_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp::Publisher<amr_interfaces::msg::UltrasonicZones>::SharedPtr zones_pub_;

  void ultrasonicCallback(const amr_interfaces::msg::UltrasonicRaw & msg)
  {
    // Per-sensor protective thresholds.
    // Index 0,1 = front sensors → 250 mm
    // Index 2,3 = rear sensors  → 130 mm
    const std::array<double, 4> thresholds = {
      front_threshold_mm_,   // index 0: Front Left
      front_threshold_mm_,   // index 1: Front Right
      rear_threshold_mm_,    // index 2: Rear Left
      rear_threshold_mm_     // index 3: Rear Right
    };

    amr_interfaces::msg::UltrasonicZones zones_msg;
    zones_msg.header = msg.header;

    bool any_protective = false;

    for (size_t i = 0; i < 4; ++i)
    {
      float dist = msg.distances_mm[i];
      zones_msg.distances_mm[i] = dist;

      // A reading of 0.0 means the STM32 returned no valid measurement.
      // Treat it as CLEAR to avoid false stops from sensor errors.
      // If you want 0.0 to be treated as PROTECTIVE (fail-safe), change
      // this condition to:  if (dist <= thresholds[i])
      if (dist > 0.0f && dist <= static_cast<float>(thresholds[i]))
      {
        zones_msg.zones[i] = amr_interfaces::msg::UltrasonicZones::ZONE_PROTECTIVE;
        any_protective = true;
        RCLCPP_WARN(get_logger(),
          "Sensor [%zu] in PROTECTIVE zone: %.1f mm (threshold: %.1f mm)",
          i, dist, thresholds[i]);
      }
      else
      {
        zones_msg.zones[i] = amr_interfaces::msg::UltrasonicZones::ZONE_CLEAR;
      }
    }

    // Set overall robot zone to worst across all sensors
    zones_msg.robot_zone = any_protective
      ? amr_interfaces::msg::UltrasonicZones::ZONE_PROTECTIVE
      : amr_interfaces::msg::UltrasonicZones::ZONE_CLEAR;

    // Always publish zone details (useful for diagnostics even when state hasn't changed)
    zones_pub_->publish(zones_msg);

    // Only publish safety stop signal when state CHANGES.
    // This matches the pattern in lidar_safety_stop.cpp and avoids
    // flooding the arbitrator with redundant messages.
    if (any_protective != prev_any_protective_)
    {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = any_protective;
      safety_stop_pub_->publish(stop_msg);

      RCLCPP_INFO(get_logger(),
        "Ultrasonic safety state changed → %s",
        any_protective ? "STOP (PROTECTIVE zone)" : "CLEAR");

      prev_any_protective_ = any_protective;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSafetyStop>());
  rclcpp::shutdown();
  return 0;
}