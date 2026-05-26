#include<algorithm>
#include<cmath>

#include "amr_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_motion
{
  PurePursuit::PurePursuit() : Node("pure_pursuit"),
    look_ahead_distance_(1.0), max_linear_velocity_(0.5), max_angular_velocity_(0.5)
  {
    declare_parameter("look_ahead_distance", look_ahead_distance_);
    declare_parameter("max_linear_velocity", max_linear_velocity_);
    declare_parameter("max_angular_velocity", max_angular_velocity_);

    look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

    navigation_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/navigation_cmd_vel", 10);
    carrot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&PurePursuit::controlLoop, this));
  }

  void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    global_plan_ = *path;
  }

  void PurePursuit::controlLoop()
  {
    if(global_plan_.poses.empty()) {
      navigation_cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    geometry_msgs::msg::TransformStamped robot_pose;
    try{
      robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
      return;
    }

    if(!transformPlan(robot_pose.header.frame_id)) {
      RCLCPP_ERROR(get_logger(), "Transform failed");
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose_stamped;
    robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

    // GOAL CHECK
    const auto & goal = global_plan_.poses.back();
    double dx_g = goal.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy_g = goal.pose.position.y - robot_pose_stamped.pose.position.y;

    if (std::sqrt(dx_g*dx_g + dy_g*dy_g) <= 0.1) {
      navigation_cmd_pub_->publish(geometry_msgs::msg::Twist{});
      global_plan_.poses.clear();
      RCLCPP_INFO(get_logger(), "Goal reached");
      return;
    }

    // REMOVE PASSED POINTS
    while (global_plan_.poses.size() > 1) {
      double dx = global_plan_.poses.front().pose.position.x - robot_pose_stamped.pose.position.x;
      double dy = global_plan_.poses.front().pose.position.y - robot_pose_stamped.pose.position.y;
      if (std::sqrt(dx*dx + dy*dy) < look_ahead_distance_ * 0.5) {
        global_plan_.poses.erase(global_plan_.poses.begin());
      } else break;
    }

    auto carrot_pose = getCarrotPose(robot_pose_stamped);
    carrot_pose_pub_->publish(carrot_pose);

    // Transform carrot to robot frame
    tf2::Transform robot_tf, carrot_tf, carrot_robot_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
    tf2::fromMsg(carrot_pose.pose, carrot_tf);

    carrot_robot_tf = robot_tf.inverse() * carrot_tf;

    geometry_msgs::msg::Pose carrot_in_robot;
    tf2::toMsg(carrot_robot_tf, carrot_in_robot);

    double angle_to_carrot = atan2(carrot_in_robot.position.y,
                                   carrot_in_robot.position.x);

    double curvature = getCurvature(carrot_in_robot);

    geometry_msgs::msg::Twist cmd;

    if (std::fabs(angle_to_carrot) > 0.7) {  // ~40 degrees
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(angle_to_carrot,
                                 -max_angular_velocity_,
                                 max_angular_velocity_);
    }
    else {
      double direction = (carrot_in_robot.position.x >= 0) ? 1.0 : -1.0;

      cmd.linear.x = direction * max_linear_velocity_;

      cmd.angular.z = std::clamp(
        curvature * cmd.linear.x,
        -max_angular_velocity_,
        max_angular_velocity_);
    }

    navigation_cmd_pub_->publish(cmd);
  }

  bool PurePursuit::transformPlan(const std::string & frame)
  {
    if(global_plan_.header.frame_id == frame) return true;

    geometry_msgs::msg::TransformStamped transform;
    try{
      transform = tf_buffer_->lookupTransform(frame,
                                              global_plan_.header.frame_id,
                                              tf2::TimePointZero);
    }
    catch (...) {
      return false;
    }

    for(auto & pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(pose, transformed_pose, transform);
      pose = transformed_pose;
    }

    global_plan_.header.frame_id = frame;
    return true;
  }

  geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(
    const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
      double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
      double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;

      if (std::sqrt(dx*dx + dy*dy) >= look_ahead_distance_) {
        return global_plan_.poses[i];
      }
    }

    return global_plan_.poses.back();
  }

  double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
  {
    double L = carrot_pose.position.x * carrot_pose.position.x +
               carrot_pose.position.y * carrot_pose.position.y;

    if(L > 0.001)
      return 2.0 * carrot_pose.position.y / L;

    return 0.0;
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_motion::PurePursuit>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}