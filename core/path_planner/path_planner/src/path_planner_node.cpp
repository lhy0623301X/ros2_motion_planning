/**
 * @file path_planner_node.cpp
 * @brief Minimal Nav2 global planner plugin shell for phased migration.
 */
#include "path_planner_node.h"

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rmp::path_planner::PathPlannerNode, nav2_core::GlobalPlanner)

namespace rmp::path_planner {

void PathPlannerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  name_ = std::move(name);

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node while configuring path_planner.");
  }

  logger_ = node->get_logger();
  global_frame_ = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string{};
  visualizer_ = std::make_unique<common::util::PlannerVisualizer>(node, name_ + "/debug_markers");

  PathPlannerFactory::PlannerProps props;
  const bool created =
    PathPlannerFactory::createPlanner(node, name_, costmap_ros_, props);
  planner_ = props.planner_ptr;
  planner_type_ = props.planner_type;

  if (created && planner_) {
    RCLCPP_INFO(logger_, "Configured path_planner plugin '%s'.", name_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "Configured minimal path_planner shell for '%s'. No concrete planner is available yet.",
      name_.c_str());
  }
}

void PathPlannerNode::cleanup()
{
  planner_.reset();
  visualizer_.reset();
  costmap_ros_.reset();
  tf_.reset();
}

void PathPlannerNode::activate()
{
  RCLCPP_INFO(logger_, "Activating path_planner shell '%s'.", name_.c_str());
}

void PathPlannerNode::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating path_planner shell '%s'.", name_.c_str());
}

nav_msgs::msg::Path PathPlannerNode::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!planner_) {
    RCLCPP_ERROR(
      logger_,
      "No concrete planner has been migrated into '%s' yet. Returning an empty plan.",
      name_.c_str());
    return makeEmptyPlan(start, goal);
  }

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_,
      "Start and goal must be in the global frame '%s' but received start='%s', goal='%s'.",
      global_frame_.c_str(), start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return makeEmptyPlan(start, goal);
  }

  auto plan = planner_->createPlan(start, goal);
  if (visualizer_ && planner_->config().enable_debug_visualization) {
    const double resolution = costmap_ros_ ? costmap_ros_->getCostmap()->getResolution() : 0.05;
    visualizer_->publish(planner_->debugInfo(), global_frame_, goal.header.stamp, resolution);
  }
  return plan;
}

nav_msgs::msg::Path PathPlannerNode::makeEmptyPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id =
    !global_frame_.empty() ? global_frame_ :
    (!goal.header.frame_id.empty() ? goal.header.frame_id : start.header.frame_id);
  path.header.stamp = goal.header.stamp;
  return path;
}

}  // namespace rmp::path_planner
