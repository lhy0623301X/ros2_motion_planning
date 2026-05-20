/**
 * @file controller_node.cpp
 * @brief Unified Nav2 controller plugin shell.
 */
#include "controller_node.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "controller_factory.h"
#include "pluginlib/class_list_macros.hpp"
#include "util/log.h"

namespace rmp::controller {

void ControllerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("ControllerNode failed to lock lifecycle node");
  }

  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  controller_name_ = node_->declare_parameter<std::string>(
    plugin_name_ + ".controller_name", "PID");
  heading_aligner_.configure(node_, plugin_name_ + ".heading_aligner.");
  const auto goal_speed_prefix = plugin_name_ + ".goal_speed_limiter.";
  goal_speed_limit_cfg_.enabled = node_->declare_parameter<bool>(
    goal_speed_prefix + "enabled", goal_speed_limit_cfg_.enabled);
  goal_speed_limit_cfg_.max_linear_velocity = node_->declare_parameter<double>(
    goal_speed_prefix + "max_linear_velocity", goal_speed_limit_cfg_.max_linear_velocity);
  goal_speed_limit_cfg_.max_decel = node_->declare_parameter<double>(
    goal_speed_prefix + "max_decel", goal_speed_limit_cfg_.max_decel);
  goal_speed_limit_cfg_.brake_distance_scale = node_->declare_parameter<double>(
    goal_speed_prefix + "brake_distance_scale", goal_speed_limit_cfg_.brake_distance_scale);
  goal_speed_limit_cfg_.brake_distance_buffer = node_->declare_parameter<double>(
    goal_speed_prefix + "brake_distance_buffer", goal_speed_limit_cfg_.brake_distance_buffer);
  goal_speed_limit_cfg_.min_linear_velocity = node_->declare_parameter<double>(
    goal_speed_prefix + "min_linear_velocity", goal_speed_limit_cfg_.min_linear_velocity);
  controller_ = ControllerFactory::create(controller_name_);

  if (!controller_) {
    throw std::runtime_error("Controller '" + controller_name_ + "' is not migrated yet");
  }

  controller_->configure(
    node_, plugin_name_, controller_name_, tf_, std::move(costmap_ros));

  RCLCPP_INFO(
    node_->get_logger(),
    "Using migrated controller '%s' through unified controller plugin '%s'.",
    controller_name_.c_str(), plugin_name_.c_str());
}

void ControllerNode::cleanup()
{
  global_plan_.poses.clear();
  heading_aligner_.reset();
  if (controller_) {
    controller_->cleanup();
  }
}

void ControllerNode::activate()
{
  if (controller_) {
    controller_->activate();
  }
}

void ControllerNode::deactivate()
{
  if (controller_) {
    controller_->deactivate();
  }
}

void ControllerNode::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  heading_aligner_.reset();
  if (controller_) {
    controller_->setPlan(path);
  }
}

geometry_msgs::msg::TwistStamped ControllerNode::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (!controller_) {
    throw std::runtime_error("ControllerNode is not configured with an internal controller");
  }

  if (global_plan_.poses.empty()) {
    return controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  const auto plan_frame_pose = controller_->transformPoseToPathFrame(pose, global_plan_, tf_);
  cmd.header.frame_id = plan_frame_pose.header.frame_id;

  const auto & goal_pose = global_plan_.poses.back();
  if (goal_checker && goal_checker->isGoalReached(plan_frame_pose.pose, goal_pose.pose, velocity)) {
    AINFO_EVERY(20) << "[ControllerNode] goal reached by Nav2 goal checker.";
    return cmd;
  }

  if (auto goal_align_cmd =
      heading_aligner_.computeGoalAlignmentCommand(plan_frame_pose, goal_pose, velocity))
  {
    cmd.twist = *goal_align_cmd;
    return cmd;
  }

  const auto lookahead = controller_->selectLookaheadPoint(
    global_plan_, plan_frame_pose, heading_aligner_.startLookaheadDistance());
  if (auto start_align_cmd =
      heading_aligner_.computeStartAlignmentCommand(
        plan_frame_pose, lookahead.x, lookahead.y, velocity))
  {
    cmd.twist = *start_align_cmd;
    return cmd;
  }

  auto tracking_cmd = controller_->computeVelocityCommands(plan_frame_pose, velocity, goal_checker);
  const double distance_to_goal = std::hypot(
    goal_pose.pose.position.x - plan_frame_pose.pose.position.x,
    goal_pose.pose.position.y - plan_frame_pose.pose.position.y);
  tracking_cmd.twist.linear.x = controller_->limitLinearSpeedByGoalDistance(
    tracking_cmd.twist.linear.x,
    distance_to_goal,
    goal_speed_limit_cfg_);
  return tracking_cmd;
}

void ControllerNode::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (controller_) {
    controller_->setSpeedLimit(speed_limit, percentage);
  }
}

}  // namespace rmp::controller

PLUGINLIB_EXPORT_CLASS(rmp::controller::ControllerNode, nav2_core::Controller)
