/**
 * @file rpp_controller.cpp
 * @brief Regulated Pure Pursuit controller migrated to ROS2 Nav2.
 */
#include "rpp_controller/rpp_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "util/log.h"

namespace rmp::controller {

namespace {

constexpr double kLargeAngleRad = M_PI_2;

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

void RPPController::configure(
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
  lookahead_point_publisher_ =
    std::make_unique<utils::LookaheadPointPublisher>(node_, "lookahead_point");

  RCLCPP_INFO(node_->get_logger(), "%s configured in RPP mode.", controller_name_.c_str());
}

void RPPController::cleanup()
{
  global_plan_.poses.clear();
}

void RPPController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "RPPController activated.");
}

void RPPController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "RPPController deactivated.");
}

void RPPController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(node_->get_logger(), "RPPController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped RPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    AWARN << "[RPPController] computeVelocityCommands skipped: global plan is empty.";
    return cmd;
  }

  prunePlan(pose);

  const double vt = std::hypot(velocity.linear.x, velocity.linear.y);
  const double wt = velocity.angular.z;
  const double lookahead_dist = clamp(
    std::fabs(vt) * cfg_.lookahead_time,
    cfg_.min_lookahead_dist,
    cfg_.max_lookahead_dist);

  const auto lookahead = selectLookaheadPoint(global_plan_, pose, lookahead_dist);
  if (lookahead_point_publisher_) {
    const auto frame_id = global_plan_.header.frame_id.empty() ?
      (pose.header.frame_id.empty() ? "map" : pose.header.frame_id) :
      global_plan_.header.frame_id;
    lookahead_point_publisher_->publish(
      lookahead.x, lookahead.y, lookahead.theta, frame_id, node_->now());
  }

  const double dphi = std::atan2(
    lookahead.y - pose.pose.position.y,
    lookahead.x - pose.pose.position.x) - tf2::getYaw(pose.pose.orientation);
  const double e_theta = normalizeAngle(dphi);
  const double lookahead_k = 2.0 * std::sin(dphi) / std::max(lookahead_dist, 1.0e-6);

  if (std::fabs(e_theta) > kLargeAngleRad) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z =
      angularRegularization(wt, e_theta * std::max(cfg_.control_frequency, 1.0));
    return cmd;
  }

  double desired_v = applyCurvatureConstraint(cfg_.max_linear_velocity, lookahead_k);
  desired_v = applyObstacleConstraint(desired_v);
  desired_v = applyApproachConstraint(desired_v, pose, global_plan_);

  cmd.twist.linear.x = linearRegularization(vt, desired_v);
  cmd.twist.angular.z = angularRegularization(wt, desired_v * lookahead_k);

  AINFO_EVERY(20) << "[RPPController] tracking: lookahead_dist=" << lookahead_dist
                  << ", lookahead=(" << lookahead.x << ", " << lookahead.y << ")"
                  << ", dphi=" << dphi
                  << ", curvature=" << lookahead_k
                  << ", desired_v=" << desired_v
                  << ", cmd=(" << cmd.twist.linear.x << ", " << cmd.twist.angular.z << ")";
  return cmd;
}

void RPPController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

double RPPController::applyCurvatureConstraint(double raw_linear_vel, double curvature) const
{
  if (std::fabs(curvature) <= 1.0e-6) {
    return raw_linear_vel;
  }
  const double radius = std::fabs(1.0 / curvature);
  return radius < cfg_.regulated_min_radius ?
    raw_linear_vel * (radius / cfg_.regulated_min_radius) :
    raw_linear_vel;
}

double RPPController::applyObstacleConstraint(double raw_linear_vel) const
{
  if (!costmap_ros_ || !costmap_ros_->getCostmap()) {
    return raw_linear_vel;
  }

  const int size_x = static_cast<int>(costmap_ros_->getCostmap()->getSizeInCellsX() / 2U);
  const int size_y = static_cast<int>(costmap_ros_->getCostmap()->getSizeInCellsY() / 2U);
  const double robot_cost =
    static_cast<double>(costmap_ros_->getCostmap()->getCost(size_x, size_y));

  if (robot_cost != static_cast<double>(nav2_costmap_2d::FREE_SPACE) &&
      robot_cost != static_cast<double>(nav2_costmap_2d::NO_INFORMATION))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double obs_dist =
      inscribed_radius -
      (std::log(robot_cost) -
      std::log(static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) /
      std::max(cfg_.inflation_cost_factor, 1.0e-6);

    if (obs_dist < cfg_.scaling_dist) {
      return raw_linear_vel * cfg_.scaling_gain * obs_dist / std::max(cfg_.scaling_dist, 1.0e-6);
    }
  }
  return raw_linear_vel;
}

double RPPController::applyApproachConstraint(
  double raw_linear_vel,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & prune_plan) const
{
  if (prune_plan.poses.empty()) {
    return raw_linear_vel;
  }

  double remain_dist = 0.0;
  for (std::size_t i = 0; i + 1 < prune_plan.poses.size(); ++i) {
    remain_dist += planarDistance(prune_plan.poses[i], prune_plan.poses[i + 1]);
  }

  const double scale = remain_dist < cfg_.approach_dist ?
    planarDistance(prune_plan.poses.back(), robot_pose) / std::max(cfg_.approach_dist, 1.0e-6) :
    1.0;

  return std::min(raw_linear_vel, std::max(cfg_.approach_min_v, raw_linear_vel * scale));
}

void RPPController::readParameters()
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

  cfg_.regulated_min_radius = node_->declare_parameter<double>(
    parameter_prefix_ + "regulated_min_radius", cfg_.regulated_min_radius);
  cfg_.inflation_cost_factor = node_->declare_parameter<double>(
    parameter_prefix_ + "inflation_cost_factor", cfg_.inflation_cost_factor);
  cfg_.scaling_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "scaling_dist", cfg_.scaling_dist);
  cfg_.scaling_gain = node_->declare_parameter<double>(
    parameter_prefix_ + "scaling_gain", cfg_.scaling_gain);
  cfg_.approach_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "approach_dist", cfg_.approach_dist);
  cfg_.approach_min_v = node_->declare_parameter<double>(
    parameter_prefix_ + "approach_min_v", cfg_.approach_min_v);
}

void RPPController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
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

double RPPController::linearRegularization(double current, double desired) const
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

double RPPController::angularRegularization(double current, double desired) const
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

}  // namespace rmp::controller
