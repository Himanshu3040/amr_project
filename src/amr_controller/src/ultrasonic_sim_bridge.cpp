#include <string>
#include <memory>
#include <array>
#include <cmath>

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

    // One subscriber per Gazebo sensor topic.
    // gpu_lidar with 1 sample publishes sensor_msgs/LaserScan.
    sub_1_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_1_raw", 10,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 0); });

    sub_2_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_2_raw", 10,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 1); });

    sub_3_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_3_raw", 10,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 2); });

    sub_4_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "ultrasonic_4_raw", 10,
      [this](const sensor_msgs::msg::LaserScan & msg) { sensorCallback(msg, 3); });

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
  std::array<double, 4> distances_m_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_3_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_4_;
  rclcpp::Publisher<amr_interfaces::msg::UltrasonicRaw>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void sensorCallback(const sensor_msgs::msg::LaserScan & msg, size_t index)
  {
    if (msg.ranges.empty()) {
      return;
    }

    float dist = msg.ranges[0];

    // Gazebo publishes inf when no object is detected within max range.
    // nan can appear on sensor errors. Both map to max range (5 m).
    if (std::isinf(dist) || std::isnan(dist)) {
      distances_m_[index] = 1.0;
    } else {
      distances_m_[index] = static_cast<double>(dist);
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