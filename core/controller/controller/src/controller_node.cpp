/**
 * @file controller_node.cpp
 * @brief Unified Nav2 controller plugin shell.
 */
#include "controller_node.h"

#include <stdexcept>
#include <utility>

#include "controller_factory.h"
#include "pluginlib/class_list_macros.hpp"

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
  controller_name_ = node_->declare_parameter<std::string>(
    plugin_name_ + ".controller_name", "PID");
  controller_ = ControllerFactory::create(controller_name_);

  if (!controller_) {
    throw std::runtime_error("Controller '" + controller_name_ + "' is not migrated yet");
  }

  controller_->configure(
    node_, plugin_name_, controller_name_, std::move(tf), std::move(costmap_ros));

  RCLCPP_INFO(
    node_->get_logger(),
    "Using migrated controller '%s' through unified controller plugin '%s'.",
    controller_name_.c_str(), plugin_name_.c_str());
}

void ControllerNode::cleanup()
{
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
  return controller_->computeVelocityCommands(pose, velocity, goal_checker);
}

void ControllerNode::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (controller_) {
    controller_->setSpeedLimit(speed_limit, percentage);
  }
}

}  // namespace rmp::controller

PLUGINLIB_EXPORT_CLASS(rmp::controller::ControllerNode, nav2_core::Controller)
