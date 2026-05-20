/**
 * @file controller_algorithm.h
 * @brief Internal controller algorithm interface used behind the unified Nav2 plugin.
 */
#ifndef RMP_CONTROLLER_CONTROLLER_ALGORITHM_H_
#define RMP_CONTROLLER_CONTROLLER_ALGORITHM_H_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace rmp::controller {

class ControllerAlgorithm
{
public:
  virtual ~ControllerAlgorithm() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name,
    const std::string & controller_name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;

  struct LookaheadPoint
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
  };

  struct GoalSpeedLimitConfig
  {
    bool enabled{true};
    double max_linear_velocity{0.6};
    double max_decel{0.3};
    double brake_distance_scale{1.5};
    double brake_distance_buffer{0.2};
    double min_linear_velocity{0.0};
  };

  geometry_msgs::msg::PoseStamped transformPoseToPathFrame(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path,
    const std::shared_ptr<tf2_ros::Buffer> & tf) const;

  LookaheadPoint selectLookaheadPoint(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead_dist) const;

  double limitLinearSpeedByGoalDistance(
    double desired_v,
    double distance_to_goal,
    const GoalSpeedLimitConfig & config) const;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_CONTROLLER_ALGORITHM_H_
