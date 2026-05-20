/**
 * @file heading_aligner.cpp
 * @brief Common heading alignment stage for local controllers.
 */
#include "utils/heading_aligner.h"

#include <algorithm>
#include <cmath>

#include "tf2/utils.h"
#include "util/log.h"

namespace rmp::controller::utils {

namespace {

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double planarDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

}  // namespace

void HeadingAligner::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_prefix)
{
  cfg_.enable_start_alignment = node->declare_parameter<bool>(
    parameter_prefix + "enable_start_alignment", cfg_.enable_start_alignment);
  cfg_.enable_goal_alignment = node->declare_parameter<bool>(
    parameter_prefix + "enable_goal_alignment", cfg_.enable_goal_alignment);
  cfg_.control_frequency = node->declare_parameter<double>(
    parameter_prefix + "control_frequency", cfg_.control_frequency);
  cfg_.start_alignment_max_distance = node->declare_parameter<double>(
    parameter_prefix + "start_alignment_max_distance", cfg_.start_alignment_max_distance);
  cfg_.start_lookahead_dist = node->declare_parameter<double>(
    parameter_prefix + "start_lookahead_dist", cfg_.start_lookahead_dist);
  cfg_.start_yaw_threshold = node->declare_parameter<double>(
    parameter_prefix + "start_yaw_threshold", cfg_.start_yaw_threshold);
  cfg_.start_yaw_tolerance = node->declare_parameter<double>(
    parameter_prefix + "start_yaw_tolerance", cfg_.start_yaw_tolerance);
  cfg_.goal_position_tolerance = node->declare_parameter<double>(
    parameter_prefix + "goal_position_tolerance", cfg_.goal_position_tolerance);
  cfg_.goal_yaw_tolerance = node->declare_parameter<double>(
    parameter_prefix + "goal_yaw_tolerance", cfg_.goal_yaw_tolerance);
  cfg_.angular_kp = node->declare_parameter<double>(
    parameter_prefix + "angular_kp", cfg_.angular_kp);
  cfg_.angular_kd = node->declare_parameter<double>(
    parameter_prefix + "angular_kd", cfg_.angular_kd);
  cfg_.max_angular_velocity = node->declare_parameter<double>(
    parameter_prefix + "max_angular_velocity", cfg_.max_angular_velocity);
  cfg_.min_angular_velocity = node->declare_parameter<double>(
    parameter_prefix + "min_angular_velocity", cfg_.min_angular_velocity);
  cfg_.max_angular_velocity_increment = node->declare_parameter<double>(
    parameter_prefix + "max_angular_velocity_increment", cfg_.max_angular_velocity_increment);

  AINFO << "[HeadingAligner] configured: prefix=" << parameter_prefix
        << ", start_enabled=" << cfg_.enable_start_alignment
        << ", goal_enabled=" << cfg_.enable_goal_alignment
        << ", start_max_dist=" << cfg_.start_alignment_max_distance
        << ", start_lookahead_dist=" << cfg_.start_lookahead_dist
        << ", start_threshold=" << cfg_.start_yaw_threshold
        << ", start_tolerance=" << cfg_.start_yaw_tolerance
        << ", goal_position_tolerance=" << cfg_.goal_position_tolerance
        << ", goal_yaw_tolerance=" << cfg_.goal_yaw_tolerance
        << ", angular_kp=" << cfg_.angular_kp
        << ", angular_kd=" << cfg_.angular_kd
        << ", max_w=" << cfg_.max_angular_velocity
        << ", dw_limit=" << cfg_.max_angular_velocity_increment;
}

void HeadingAligner::reset()
{
  start_alignment_done_ = false;
  prev_heading_error_valid_ = false;
}

double HeadingAligner::startLookaheadDistance() const
{
  return cfg_.start_lookahead_dist;
}

std::optional<geometry_msgs::msg::Twist> HeadingAligner::computeStartAlignmentCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double target_x,
  double target_y,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!cfg_.enable_start_alignment || start_alignment_done_) {
    return std::nullopt;
  }

  const double dx = target_x - robot_pose.pose.position.x;
  const double dy = target_y - robot_pose.pose.position.y;
  const double dist_to_target = std::hypot(dx, dy);
  if (dist_to_target > cfg_.start_alignment_max_distance) {
    start_alignment_done_ = true;
    return std::nullopt;
  }

  const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double target_heading = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(target_heading - current_yaw);
  const double abs_error = std::fabs(heading_error);

  if (abs_error <= cfg_.start_yaw_tolerance) {
    start_alignment_done_ = true;
    return std::nullopt;
  }
  if (abs_error < cfg_.start_yaw_threshold) {
    start_alignment_done_ = true;
    return std::nullopt;
  }

  auto cmd = makeRotateCommand(heading_error, velocity.angular.z);
  AINFO_EVERY(20) << "[HeadingAligner] start-align: target=(" << target_x << ", " << target_y
                  << "), target_heading=" << target_heading
                  << ", current_yaw=" << current_yaw
                  << ", heading_error=" << heading_error
                  << ", cmd_w=" << cmd.angular.z;
  return cmd;
}

std::optional<geometry_msgs::msg::Twist> HeadingAligner::computeGoalAlignmentCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!cfg_.enable_goal_alignment) {
    return std::nullopt;
  }
  if (planarDistance(robot_pose, goal_pose) > cfg_.goal_position_tolerance) {
    return std::nullopt;
  }

  const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  const double heading_error = normalizeAngle(goal_yaw - current_yaw);

  if (std::fabs(heading_error) <= cfg_.goal_yaw_tolerance) {
    geometry_msgs::msg::Twist stop_cmd;
    AINFO_EVERY(20) << "[HeadingAligner] goal-align complete: heading_error="
                    << heading_error;
    return stop_cmd;
  }

  auto cmd = makeRotateCommand(heading_error, velocity.angular.z);
  AINFO_EVERY(20) << "[HeadingAligner] goal-align: heading_error=" << heading_error
                  << ", current_w=" << velocity.angular.z
                  << ", cmd_w=" << cmd.angular.z;
  return cmd;
}

geometry_msgs::msg::Twist HeadingAligner::makeRotateCommand(
  double heading_error,
  double current_w)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  const double dt = 1.0 / std::max(cfg_.control_frequency, 1e-3);
  double d_heading_error = 0.0;
  if (prev_heading_error_valid_) {
    d_heading_error = normalizeAngle(heading_error - prev_heading_error_) / dt;
  }
  prev_heading_error_ = heading_error;
  prev_heading_error_valid_ = true;

  double desired_w = clamp(
    cfg_.angular_kp * heading_error + cfg_.angular_kd * d_heading_error,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);

  if (std::fabs(desired_w) < cfg_.min_angular_velocity) {
    desired_w = std::copysign(cfg_.min_angular_velocity, desired_w);
  }

  double inc = desired_w - current_w;
  inc = clamp(
    inc,
    -cfg_.max_angular_velocity_increment,
    cfg_.max_angular_velocity_increment);

  cmd.angular.z = clamp(
    current_w + inc,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  return cmd;
}

}  // namespace rmp::controller::utils
