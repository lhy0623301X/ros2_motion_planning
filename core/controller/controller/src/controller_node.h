/**
 * @file controller_node.h
 * @brief Unified Nav2 controller plugin shell.
 */
#ifndef RMP_CONTROLLER_CONTROLLER_NODE_H_
#define RMP_CONTROLLER_CONTROLLER_NODE_H_

#include <memory>
#include <string>

#include "controller_algorithm.h"
#include "nav2_core/controller.hpp"

namespace rmp::controller {

class ControllerNode : public nav2_core::Controller
{
public:
  ControllerNode() = default;
  ~ControllerNode() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::string controller_name_;
  std::shared_ptr<ControllerAlgorithm> controller_;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_CONTROLLER_NODE_H_
