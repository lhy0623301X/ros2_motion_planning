/**
 * @file curvature_speed_limiter.h
 * @brief Limit forward speed by local path curvature.
 */
#ifndef RMP_CONTROLLER_UTILS_CURVATURE_SPEED_LIMITER_H_
#define RMP_CONTROLLER_UTILS_CURVATURE_SPEED_LIMITER_H_

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rmp::controller::utils {

struct CurvatureSpeedLimiterConfig
{
  bool enabled{true};
  double max_lateral_accel{0.6};
  double min_linear_velocity{0.0};
  double curvature_epsilon{1.0e-3};
  double sample_distance{0.3};
  double preview_distance{1.2};
};

class CurvatureSpeedLimiter
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_prefix);

  double limitSpeed(
    double desired_v,
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

private:
  double estimateMaxCurvatureAhead(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  std::size_t findClosestPoseIndex(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  std::size_t advanceByDistance(
    const nav_msgs::msg::Path & path,
    std::size_t start_index,
    double target_distance) const;

  double computeCurvature(
    const geometry_msgs::msg::PoseStamped & p0,
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2) const;

  CurvatureSpeedLimiterConfig cfg_;
};

}  // namespace rmp::controller::utils

#endif  // RMP_CONTROLLER_UTILS_CURVATURE_SPEED_LIMITER_H_
