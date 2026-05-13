#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TurboScaler : public rclcpp::Node
{
public:
  TurboScaler() : Node("turbo_scaler")
  {
    declare_parameter<double>("linear_min");
    declare_parameter<double>("linear_max");
    declare_parameter<double>("angular_min");
    declare_parameter<double>("angular_max");
    declare_parameter<int>("steps");

    linear_min_  = get_parameter("linear_min").as_double();
    linear_max_  = get_parameter("linear_max").as_double();
    angular_min_ = get_parameter("angular_min").as_double();
    angular_max_ = get_parameter("angular_max").as_double();
    steps_       = get_parameter("steps").as_int();
    current_step_ = steps_;  // start at max speed

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_combined", 10,
      std::bind(&TurboScaler::cmdCallback, this, _1)
    );

    pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "amr_controller/cmd_vel_unstamped", 10
    );

    decrease_server_ = rclcpp_action::create_server<twist_mux_msgs::action::JoyTurbo>(
      this, "joy_turbo_decrease",
      std::bind(&TurboScaler::handleGoal,   this, _1, _2),
      std::bind(&TurboScaler::handleCancel, this, _1),
      std::bind(&TurboScaler::handleDecreaseAccepted, this, _1)
    );

    increase_server_ = rclcpp_action::create_server<twist_mux_msgs::action::JoyTurbo>(
      this, "joy_turbo_increase",
      std::bind(&TurboScaler::handleGoal,   this, _1, _2),
      std::bind(&TurboScaler::handleCancel, this, _1),
      std::bind(&TurboScaler::handleIncreaseAccepted, this, _1)
    );

    RCLCPP_INFO(get_logger(),
      "TurboScaler ready | steps=%d | linear=%.2f~%.2f | angular=%.2f~%.2f",
      steps_, linear_min_, linear_max_, angular_min_, angular_max_);
  }

private:
  double linear_min_, linear_max_, angular_min_, angular_max_;
  int steps_, current_step_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp_action::Server<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_server_;
  rclcpp_action::Server<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_server_;

  double currentLinear()
  {
    double t = (double)(current_step_ - 1) / std::max(steps_ - 1, 1);
    return linear_min_ + t * (linear_max_ - linear_min_);
  }

  double currentAngular()
  {
    double t = (double)(current_step_ - 1) / std::max(steps_ - 1, 1);
    return angular_min_ + t * (angular_max_ - angular_min_);
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double lin = currentLinear();
    double ang = currentAngular();
    geometry_msgs::msg::Twist scaled;
    scaled.linear.x  = std::clamp(msg->linear.x,  -lin, lin);
    scaled.angular.z = std::clamp(msg->angular.z, -ang, ang);
    pub_->publish(scaled);
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const twist_mux_msgs::action::JoyTurbo::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<twist_mux_msgs::action::JoyTurbo>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleDecreaseAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<twist_mux_msgs::action::JoyTurbo>> goal_handle)
  {
    current_step_ = std::max(1, current_step_ - 1);
    RCLCPP_INFO(get_logger(), "Turbo DECREASE → step %d/%d | linear=%.2f angular=%.2f",
      current_step_, steps_, currentLinear(), currentAngular());
    goal_handle->succeed(std::make_shared<twist_mux_msgs::action::JoyTurbo::Result>());
  }

  void handleIncreaseAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<twist_mux_msgs::action::JoyTurbo>> goal_handle)
  {
    current_step_ = std::min(steps_, current_step_ + 1);
    RCLCPP_INFO(get_logger(), "Turbo INCREASE → step %d/%d | linear=%.2f angular=%.2f",
      current_step_, steps_, currentLinear(), currentAngular());
    goal_handle->succeed(std::make_shared<twist_mux_msgs::action::JoyTurbo::Result>());
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurboScaler>());
  rclcpp::shutdown();
  return 0;
}