/**
 * @file pid_controller.h
 * @brief PID local controller plugin migrated to ROS2 Nav2.
 */
#ifndef RMP_CONTROLLER_PID_CONTROLLER_H_
#define RMP_CONTROLLER_PID_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "controller_algorithm.h"

namespace rmp::controller {

struct PIDControllerConfig
{
  double control_frequency{20.0};
  double goal_dist_tolerance{0.25};
  double rotate_tolerance{0.25};

  double max_linear_velocity{0.5};
  double min_linear_velocity{0.0};
  double max_linear_velocity_increment{0.05};
  double max_angular_velocity{1.82};
  double min_angular_velocity{0.0};
  double max_angular_velocity_increment{0.1};

  double lookahead_time{1.0};
  double min_lookahead_dist{0.3};
  double max_lookahead_dist{0.9};

  double p_linear_velocity{1.0};
  double i_linear_velocity{0.0};
  double d_linear_velocity{0.0};
  double p_angular_velocity{2.0};
  double i_angular_velocity{0.0};
  double d_angular_velocity{0.0};

  double k_feedback{1.0};
  double dist_from_center_to_front_edge{0.3};
  bool model_based_mode{false};
};

class PIDController : public ControllerAlgorithm
{
public:
  PIDController() = default;
  ~PIDController() override = default;

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
  struct TrackingPoint
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
  };

  void readParameters();
  void resetPidState();
  void prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose);
  TrackingPoint getLookAheadPoint(
    double lookahead_dist,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  Eigen::Vector2d modelFreePIDControl(
    const Eigen::Vector3d & state,
    const Eigen::Vector3d & desired,
    const Eigen::Vector2d & current_velocity);
  Eigen::Vector2d modelBasedPIDControl(
    const Eigen::Vector3d & state,
    const Eigen::Vector3d & desired) const;

  double linearRegularization(double current, double desired) const;
  double angularRegularization(double current, double desired) const;
  bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped & pose) const;
  bool shouldRotateToPath(double heading_error) const;
  double goalYaw() const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string parameter_prefix_;
  std::string controller_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  PIDControllerConfig cfg_;

  double nominal_max_linear_velocity_{0.5};
  double e_v_{0.0};
  double e_w_{0.0};
  double i_v_{0.0};
  double i_w_{0.0};
  bool goal_reached_{false};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_PID_CONTROLLER_H_
