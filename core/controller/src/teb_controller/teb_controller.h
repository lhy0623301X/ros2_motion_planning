/**
 * @file teb_controller.h
 * @brief TEB controller shell for the unified ROS2 controller framework.
 */
#ifndef RMP_CONTROLLER_TEB_CONTROLLER_H_
#define RMP_CONTROLLER_TEB_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>

#include "controller_algorithm.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "teb_controller/teb_optimizer.h"
#include "teb_controller/teb_types.h"
#include "teb_controller/teb_visualizer.h"
#include "tf2_ros/buffer.h"

namespace rmp::controller {

class TEBController : public ControllerAlgorithm
{
public:
  TEBController() = default;
  ~TEBController() override = default;

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
  double linearRegularization(double current, double desired) const;
  double angularRegularization(double current, double desired) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string parameter_prefix_;
  std::string controller_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  TEBControllerConfig cfg_;
  TEBOptimizer optimizer_;
  std::unique_ptr<TEBVisualizer> visualizer_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  double nominal_max_linear_velocity_{0.8};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_TEB_CONTROLLER_H_
