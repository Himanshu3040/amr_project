#include <string>
#include <memory>
#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "amr_interfaces/msg/ultrasonic_raw.hpp"

using std::placeholders::_1;

class UltrasonicSimBridge : public rclcpp::Node
{
public:
  UltrasonicSimBridge()
  : Node("ultrasonic_sim_bridge_node")
  {
    // Default to 5.0 m — well beyond any protective threshold.
    // Safe value before first sensor message arrives.
    distances_m_.fill(5.0);

      // Gazebo Harmonic publishes sensor topics with BEST_EFFORT QoS.
    // Our subscriptions must match, otherwise ROS 2 will not connect them.
    auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    sub_1_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_1_raw", qos_best_effort,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 0); });

    sub_2_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_2_raw", qos_best_effort,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 1); });

    sub_3_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_3_raw", qos_best_effort,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 2); });

    sub_4_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_4_raw", qos_best_effort,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 3); });

    sub_5_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_5_raw", qos_best_effort,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 4); });

    // Publisher — same topic and message type as the real ethernet_bridge_node
    pub_ = create_publisher<amr_interfaces::msg::UltrasonicRaw>("ultrasonic_raw", 10);

    // Publish the combined message at 10 Hz.
    // This decouples the publish rate from individual sensor update timing.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UltrasonicSimBridge::publishCombined, this));

    RCLCPP_INFO(get_logger(),
      "UltrasonicSimBridge started. Bridging Gazebo sensor topics → /ultrasonic_raw");
  }

private:
  std::array<double, 5> distances_m_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_3_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_4_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_5_;
  rclcpp::Publisher<amr_interfaces::msg::UltrasonicRaw>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void sensorCallback(const sensor_msgs::msg::LaserScan & msg, size_t index)
  {
    if (msg.ranges.empty()) {
      return;
    }

    // With 7 samples across a 40° cone, take the minimum range —
    // closest detected obstacle in the cone is what matters for safety.
    float min_dist = std::numeric_limits<float>::infinity();
    for (const float r : msg.ranges) {
      if (!std::isinf(r) && !std::isnan(r) && r < min_dist) {
        min_dist = r;
      }
    }

    // If all rays returned inf/nan, no object detected — use max range
    if (std::isinf(min_dist) || std::isnan(min_dist)) {
      distances_m_[index] = 1.0;
    } else {
      distances_m_[index] = static_cast<double>(min_dist);
    }
  }

  void publishCombined()
  {
    amr_interfaces::msg::UltrasonicRaw out;
    out.header.stamp = this->get_clock()->now();
    out.header.frame_id = "base_link";

    // Convert metres → millimetres to match UltrasonicRaw.msg field units
    out.distances_mm[0] = static_cast<float>(distances_m_[0] * 1000.0);  // Front Left
    out.distances_mm[1] = static_cast<float>(distances_m_[1] * 1000.0);  // Front Right
    out.distances_mm[2] = static_cast<float>(distances_m_[2] * 1000.0);  // Rear Right
    out.distances_mm[3] = static_cast<float>(distances_m_[3] * 1000.0);  // Rear Left
    out.distances_mm[4] = static_cast<float>(distances_m_[4] * 1000.0);  // Rear Middle

    pub_->publish(out);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSimBridge>());
  rclcpp::shutdown();
  return 0;
}