/**
 * @file dwa_controller.h
 * @brief Minimal DWA controller implemented for the ROS2 controller framework.
 */
#ifndef RMP_CONTROLLER_DWA_CONTROLLER_H_
#define RMP_CONTROLLER_DWA_CONTROLLER_H_

#include <memory>
#include <string>

#include "controller_algorithm.h"
#include "dwa_controller/dwa_types.h"
#include "dwa_controller/dwa_visualizer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace rmp::controller {

class DWAController : public ControllerAlgorithm
{
public:
  DWAController() = default;
  ~DWAController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name,
    const std::string & controller_name,
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
  void readParameters();
  void prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose);
  DWATrajectory plan(const DWAState & state) const;
  double linearRegularization(double current, double desired) const;
  double angularRegularization(double current, double desired) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string parameter_prefix_;
  std::string controller_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  DWAControllerConfig cfg_;
  std::unique_ptr<DWAVisualizer> visualizer_;
  double nominal_max_linear_velocity_{0.6};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_DWA_CONTROLLER_H_
