/**
 * @file heading_aligner.h
 * @brief Common heading alignment stage for local controllers.
 */
#ifndef RMP_CONTROLLER_UTILS_HEADING_ALIGNER_H_
#define RMP_CONTROLLER_UTILS_HEADING_ALIGNER_H_

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rmp::controller::utils {

struct HeadingAlignerConfig
{
  bool enable_start_alignment{true};
  bool enable_goal_alignment{true};
  double control_frequency{20.0};
  double start_alignment_max_distance{2.0};
  double start_lookahead_dist{0.5};
  double start_yaw_threshold{0.5};
  double start_yaw_tolerance{0.08};
  double goal_position_tolerance{0.25};
  double goal_yaw_tolerance{0.25};
  double angular_kp{1.5};
  double max_angular_velocity{1.2};
  double min_angular_velocity{0.0};
  double max_angular_velocity_increment{0.2};
};

class HeadingAligner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_prefix);

  void reset();

  double startLookaheadDistance() const;

  std::optional<geometry_msgs::msg::Twist> computeStartAlignmentCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double target_x,
    double target_y,
    const geometry_msgs::msg::Twist & velocity);

  std::optional<geometry_msgs::msg::Twist> computeGoalAlignmentCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const geometry_msgs::msg::Twist & velocity) const;

private:
  geometry_msgs::msg::Twist makeRotateCommand(
    double heading_error,
    double current_w) const;

  HeadingAlignerConfig cfg_;
  bool start_alignment_done_{false};
};

}  // namespace rmp::controller::utils

#endif  // RMP_CONTROLLER_UTILS_HEADING_ALIGNER_H_
