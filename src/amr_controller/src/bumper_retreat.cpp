#include <string>
#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class BumperState { CLEAR, RETREAT };

class BumperSafetyStop : public rclcpp::Node
{
public:
  BumperSafetyStop()
  : Node("bumper_retreat_node"),
    state_(BumperState::CLEAR),
    front_pressed_(false),
    rear_pressed_(false),
    retreat_direction_(0.0)
  {
    declare_parameter<std::string>("bumper_retreat_topic", "bumper_retreat_cmd_vel");
    declare_parameter<std::string>("front_bumper_joint",   "Front_Bumper_Joint");
    declare_parameter<std::string>("rear_bumper_joint",    "Rear_Bumper_Joint");
    declare_parameter<double>     ("press_threshold");
    declare_parameter<double>     ("retreat_speed");
    declare_parameter<double>     ("retreat_duration");

    retreat_topic_    = get_parameter("bumper_retreat_topic").as_string();
    front_joint_name_ = get_parameter("front_bumper_joint").as_string();
    rear_joint_name_  = get_parameter("rear_bumper_joint").as_string();
    press_threshold_  = get_parameter("press_threshold").as_double();
    retreat_speed_    = get_parameter("retreat_speed").as_double();
    retreat_duration_ = get_parameter("retreat_duration").as_double();

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&BumperSafetyStop::jointStateCallback, this, _1));

    retreat_pub_ = create_publisher<geometry_msgs::msg::Twist>(retreat_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "BumperSafetyStop started | joints: [%s, %s] | threshold: %.4f m | "
      "retreat: %.2f m/s for %.2f s",
      front_joint_name_.c_str(), rear_joint_name_.c_str(),
      press_threshold_, retreat_speed_, retreat_duration_);
  }

private:
  std::string  retreat_topic_, front_joint_name_, rear_joint_name_;
  double       press_threshold_, retreat_speed_, retreat_duration_;
  double       retreat_direction_;
  BumperState  state_;
  bool         front_pressed_, rear_pressed_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       retreat_pub_;
  rclcpp::TimerBase::SharedPtr                                  retreat_timer_;

  void startRetreat(double linear_x)
  {
    state_             = BumperState::RETREAT;
    retreat_direction_ = linear_x;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    retreat_pub_->publish(twist);

    RCLCPP_WARN(get_logger(),
      "RETREAT: linear.x = %.2f m/s for %.2f s", linear_x, retreat_duration_);

    // Cancel any existing timer before creating a new one
    if (retreat_timer_) {
      retreat_timer_->cancel();
    }

    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(retreat_duration_));

    retreat_timer_ = create_wall_timer(ns, [this]() {
      retreat_timer_->cancel();
      onRetreatDone();
    });
  }

  void onRetreatDone()
  {
    bool still_pressed = front_pressed_ || rear_pressed_;

    if (still_pressed) {
      // Bumper still pressed — keep retreating in the same direction
      RCLCPP_WARN(get_logger(),
        "Retreat done — bumper still pressed → retreating again");
      startRetreat(retreat_direction_);
    } else {
      // Bumper released — stop and go clear
      retreat_pub_->publish(geometry_msgs::msg::Twist());
      state_ = BumperState::CLEAR;
      RCLCPP_INFO(get_logger(), "Retreat done — bumper released → CLEAR");
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if      (msg.name[i] == front_joint_name_)
        front_pressed_ = std::abs(msg.position[i]) > press_threshold_;
      else if (msg.name[i] == rear_joint_name_)
        rear_pressed_  = std::abs(msg.position[i]) > press_threshold_;
    }

    bool any_pressed = front_pressed_ || rear_pressed_;

    switch (state_)
    {
      case BumperState::CLEAR:
        if (any_pressed) {
          double speed = front_pressed_ ? -retreat_speed_ : +retreat_speed_;
          startRetreat(speed);
        }
        break;

      case BumperState::RETREAT:
        // Timer is in charge — do not interrupt
        break;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BumperSafetyStop>());
  rclcpp::shutdown();
  return 0;
}