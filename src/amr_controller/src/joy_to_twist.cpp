#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToTwist : public rclcpp::Node
{
public:
  JoyToTwist() : Node("joy_to_twist"),
    linear_axis_(3),
    angular_axis_(0),
    deadman_axis_1_(4),
    deadman_axis_2_(5), 
    linear_scale_(1.0),
    angular_scale_(1.0)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("joy_vel", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyToTwist::joy_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "joy_to_twist node started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    bool deadman_pressed = (msg->axes[deadman_axis_1_] == -1.0) ||
                           (msg->axes[deadman_axis_2_] == -1.0);

    if (!deadman_pressed) return;

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = msg->axes[linear_axis_]  * linear_scale_;
    twist.angular.z = msg->axes[angular_axis_] * angular_scale_;
    pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

  const int    linear_axis_, angular_axis_, deadman_axis_1_, deadman_axis_2_;
  const double linear_scale_, angular_scale_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToTwist>());
  rclcpp::shutdown();
  return 0;
}