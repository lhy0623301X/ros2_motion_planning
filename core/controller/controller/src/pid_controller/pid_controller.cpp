/**
 * @file pid_controller.cpp
 * @brief PID local controller plugin migrated to ROS2 Nav2.
 */
#include "pid_controller/pid_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/utils.h"

namespace rmp::controller {

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

void PIDController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  parameter_prefix_ = plugin_name + "." + controller_name + ".";
  controller_name_ = controller_name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  readParameters();
  nominal_max_linear_velocity_ = cfg_.max_linear_velocity;
  resetPidState();

  RCLCPP_INFO(
    node_->get_logger(),
    "%s configured in %s mode.",
    controller_name_.c_str(),
    cfg_.model_based_mode ? "model-based" : "model-free");
}

void PIDController::cleanup()
{
  global_plan_.poses.clear();
  resetPidState();
}

void PIDController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "PIDController activated.");
}

void PIDController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "PIDController deactivated.");
}

void PIDController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_reached_ = false;
  resetPidState();
  RCLCPP_INFO(node_->get_logger(), "PIDController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped PIDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "PIDController has no global plan.");
    return cmd;
  }

  const auto & goal_pose = global_plan_.poses.back();
  if (goal_checker && goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
    goal_reached_ = true;
    resetPidState();
    return cmd;
  }

  prunePlan(pose);
  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);
  const double current_v = velocity.linear.x;
  const double current_w = velocity.angular.z;
  const double current_yaw = tf2::getYaw(pose.pose.orientation);

  if (shouldRotateToGoal(pose)) {
    const double heading_error = normalizeAngle(goalYaw() - current_yaw);
    if (!shouldRotateToPath(std::fabs(heading_error))) {
      goal_reached_ = true;
      resetPidState();
      return cmd;
    }
    cmd.twist.angular.z = angularRegularization(current_w, heading_error / dt);
    return cmd;
  }

  const double lookahead_dist = clamp(
    std::fabs(current_v) * cfg_.lookahead_time,
    cfg_.min_lookahead_dist,
    cfg_.max_lookahead_dist);
  const TrackingPoint target = getLookAheadPoint(lookahead_dist, pose);

  const Eigen::Vector3d state(
    pose.pose.position.x,
    pose.pose.position.y,
    current_yaw);
  const Eigen::Vector3d desired(
    target.x,
    target.y,
    target.theta);
  const Eigen::Vector2d current_input(current_v, current_w);

  const Eigen::Vector2d control = cfg_.model_based_mode ?
    modelBasedPIDControl(state, desired) :
    modelFreePIDControl(state, desired, current_input);

  cmd.twist.linear.x = linearRegularization(current_v, control[0]);
  cmd.twist.angular.z = angularRegularization(current_w, control[1]);
  return cmd;
}

void PIDController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

void PIDController::readParameters()
{
  cfg_.control_frequency = node_->declare_parameter<double>(
    parameter_prefix_ + "control_frequency", cfg_.control_frequency);
  cfg_.goal_dist_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "goal_dist_tolerance", cfg_.goal_dist_tolerance);
  cfg_.rotate_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "rotate_tolerance", cfg_.rotate_tolerance);

  cfg_.max_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity", cfg_.max_linear_velocity);
  cfg_.min_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_linear_velocity", cfg_.min_linear_velocity);
  cfg_.max_linear_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity_increment", cfg_.max_linear_velocity_increment);
  cfg_.max_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity", cfg_.max_angular_velocity);
  cfg_.min_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_angular_velocity", cfg_.min_angular_velocity);
  cfg_.max_angular_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity_increment", cfg_.max_angular_velocity_increment);

  cfg_.lookahead_time = node_->declare_parameter<double>(
    parameter_prefix_ + "lookahead_time", cfg_.lookahead_time);
  cfg_.min_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "min_lookahead_dist", cfg_.min_lookahead_dist);
  cfg_.max_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "max_lookahead_dist", cfg_.max_lookahead_dist);

  cfg_.p_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "p_linear_velocity", cfg_.p_linear_velocity);
  cfg_.i_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "i_linear_velocity", cfg_.i_linear_velocity);
  cfg_.d_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "d_linear_velocity", cfg_.d_linear_velocity);
  cfg_.p_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "p_angular_velocity", cfg_.p_angular_velocity);
  cfg_.i_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "i_angular_velocity", cfg_.i_angular_velocity);
  cfg_.d_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "d_angular_velocity", cfg_.d_angular_velocity);

  cfg_.k_feedback = node_->declare_parameter<double>(
    parameter_prefix_ + "k_feedback", cfg_.k_feedback);
  cfg_.dist_from_center_to_front_edge = node_->declare_parameter<double>(
    parameter_prefix_ + "dist_from_center_to_front_edge", cfg_.dist_from_center_to_front_edge);
  cfg_.model_based_mode = node_->declare_parameter<bool>(
    parameter_prefix_ + "model_based_mode", cfg_.model_based_mode);
}

void PIDController::resetPidState()
{
  e_v_ = 0.0;
  e_w_ = 0.0;
  i_v_ = 0.0;
  i_w_ = 0.0;
}

void PIDController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2) {
    return;
  }

  const double search_distance = costmap_ros_ && costmap_ros_->getCostmap() ?
    costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0 :
    std::numeric_limits<double>::max();

  auto search_end = global_plan_.poses.end();
  double integrated = 0.0;
  for (auto it = global_plan_.poses.begin(); it + 1 != global_plan_.poses.end(); ++it) {
    integrated += planarDistance(*it, *(it + 1));
    if (integrated > search_distance) {
      search_end = it + 1;
      break;
    }
  }

  auto closest = std::min_element(
    global_plan_.poses.begin(), search_end,
    [&](const auto & lhs, const auto & rhs) {
      return planarDistance(robot_pose, lhs) < planarDistance(robot_pose, rhs);
    });

  if (closest != global_plan_.poses.begin() && closest != global_plan_.poses.end()) {
    global_plan_.poses.erase(global_plan_.poses.begin(), closest);
  }
}

PIDController::TrackingPoint PIDController::getLookAheadPoint(
  double lookahead_dist,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  auto target_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & plan_pose) {
      return std::hypot(plan_pose.pose.position.x - rx, plan_pose.pose.position.y - ry) >=
             lookahead_dist;
    });

  if (target_it == global_plan_.poses.end()) {
    target_it = std::prev(global_plan_.poses.end());
  }

  TrackingPoint target;
  target.x = target_it->pose.position.x;
  target.y = target_it->pose.position.y;

  if (target_it + 1 != global_plan_.poses.end()) {
    const auto & next = *(target_it + 1);
    target.theta = std::atan2(
      next.pose.position.y - target.y,
      next.pose.position.x - target.x);
  } else {
    target.theta = tf2::getYaw(target_it->pose.orientation);
    if (!std::isfinite(target.theta)) {
      target.theta = std::atan2(target.y - ry, target.x - rx);
    }
  }
  return target;
}

Eigen::Vector2d PIDController::modelFreePIDControl(
  const Eigen::Vector3d & state,
  const Eigen::Vector3d & desired,
  const Eigen::Vector2d & current_velocity)
{
  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);
  const double e_x = desired[0] - state[0];
  const double e_y = desired[1] - state[1];
  const double e_theta = normalizeAngle(desired[2] - state[2]);

  const double v_desired = clamp(
    std::hypot(e_x, e_y) / dt,
    -cfg_.max_linear_velocity,
    cfg_.max_linear_velocity);
  const double w_desired = clamp(
    e_theta / dt,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);

  const double e_v = v_desired - current_velocity[0];
  const double e_w = w_desired - current_velocity[1];
  i_v_ += e_v * dt;
  i_w_ += e_w * dt;

  const double d_v = (e_v - e_v_) / dt;
  const double d_w = (e_w - e_w_) / dt;
  e_v_ = e_v;
  e_w_ = e_w;

  const double v_cmd = current_velocity[0] +
    cfg_.p_linear_velocity * e_v +
    cfg_.i_linear_velocity * i_v_ +
    cfg_.d_linear_velocity * d_v;
  const double w_cmd = current_velocity[1] +
    cfg_.p_angular_velocity * e_w +
    cfg_.i_angular_velocity * i_w_ +
    cfg_.d_angular_velocity * d_w;

  return {v_cmd, w_cmd};
}

Eigen::Vector2d PIDController::modelBasedPIDControl(
  const Eigen::Vector3d & state,
  const Eigen::Vector3d & desired) const
{
  const Eigen::Vector3d error(
    desired[0] - state[0],
    desired[1] - state[1],
    normalizeAngle(desired[2] - state[2]));
  const Eigen::Vector2d position_dot(
    cfg_.k_feedback * error[0],
    cfg_.k_feedback * error[1]);

  const double front_edge = std::max(0.01, cfg_.dist_from_center_to_front_edge);
  Eigen::Matrix2d inverse_model;
  inverse_model << std::cos(state[2]), std::sin(state[2]),
    -std::sin(state[2]) / front_edge, std::cos(state[2]) / front_edge;
  return inverse_model * position_dot;
}

double PIDController::linearRegularization(double current, double desired) const
{
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_linear_velocity_increment,
    cfg_.max_linear_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_linear_velocity,
    cfg_.max_linear_velocity);

  if (std::fabs(cmd) < cfg_.min_linear_velocity) {
    return 0.0;
  }
  return cmd;
}

double PIDController::angularRegularization(double current, double desired) const
{
  desired = clamp(
    desired,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_angular_velocity_increment,
    cfg_.max_angular_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);

  if (std::fabs(cmd) < cfg_.min_angular_velocity) {
    return 0.0;
  }
  return cmd;
}

bool PIDController::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped & pose) const
{
  return planarDistance(pose, global_plan_.poses.back()) < cfg_.goal_dist_tolerance;
}

bool PIDController::shouldRotateToPath(double heading_error) const
{
  return heading_error > cfg_.rotate_tolerance;
}

double PIDController::goalYaw() const
{
  return tf2::getYaw(global_plan_.poses.back().pose.orientation);
}

}  // namespace rmp::controller
