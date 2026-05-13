#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class SafetyArbitrator : public rclcpp::Node
{
public:
  SafetyArbitrator()
  : Node("safety_arbitrator_node"),
    lidar_stop_(false),
    ultrasonic_stop_(false),
    bumper_stop_(false)
  {
    declare_parameter<std::string>("lidar_safety_stop_topic",      "lidar_safety_stop");
    declare_parameter<std::string>("ultrasonic_safety_stop_topic", "ultrasonic_safety_stop");
    declare_parameter<std::string>("bumper_safety_stop_topic",     "bumper_safety_stop");
    declare_parameter<std::string>("safety_stop_topic",            "safety_stop");

    std::string lidar_topic      = get_parameter("lidar_safety_stop_topic").as_string();
    std::string ultrasonic_topic = get_parameter("ultrasonic_safety_stop_topic").as_string();
    std::string bumper_topic     = get_parameter("bumper_safety_stop_topic").as_string();
    std::string output_topic     = get_parameter("safety_stop_topic").as_string();

    lidar_sub_ = create_subscription<std_msgs::msg::Bool>(
      lidar_topic, 10,
      std::bind(&SafetyArbitrator::lidarCallback, this, _1));

    ultrasonic_sub_ = create_subscription<std_msgs::msg::Bool>(
      ultrasonic_topic, 10,
      std::bind(&SafetyArbitrator::ultrasonicCallback, this, _1));

    bumper_sub_ = create_subscription<std_msgs::msg::Bool>(
      bumper_topic, 10,
      std::bind(&SafetyArbitrator::bumperCallback, this, _1));

    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(output_topic, 10);

    RCLCPP_INFO(get_logger(), "SafetyArbitrator started.");
    RCLCPP_INFO(get_logger(), "Listening to: %s, %s, %s",
      lidar_topic.c_str(), ultrasonic_topic.c_str(), bumper_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publishing to: %s", output_topic.c_str());
  }

private:
  bool lidar_stop_;
  bool ultrasonic_stop_;
  bool bumper_stop_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    safety_stop_pub_;

  void publishArbitratedStop()
  {
    bool combined = lidar_stop_ || ultrasonic_stop_ || bumper_stop_;
    std_msgs::msg::Bool msg;
    msg.data = combined;
    safety_stop_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "Safety arbitration -> %s  [lidar=%s | ultrasonic=%s | bumper=%s]",
      combined         ? "STOP"  : "CLEAR",
      lidar_stop_      ? "STOP"  : "clear",
      ultrasonic_stop_ ? "STOP"  : "clear",
      bumper_stop_     ? "STOP"  : "clear");
  }

  void lidarCallback(const std_msgs::msg::Bool & msg)
  {
    if (msg.data != lidar_stop_) {
      lidar_stop_ = msg.data;
      publishArbitratedStop();
    }
  }

  void ultrasonicCallback(const std_msgs::msg::Bool & msg)
  {
    if (msg.data != ultrasonic_stop_) {
      ultrasonic_stop_ = msg.data;
      publishArbitratedStop();
    }
  }

  void bumperCallback(const std_msgs::msg::Bool & msg)
  {
    if (msg.data != bumper_stop_) {
      bumper_stop_ = msg.data;
      publishArbitratedStop();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyArbitrator>());
  rclcpp::shutdown();
  return 0;
}
