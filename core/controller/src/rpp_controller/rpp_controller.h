/**
 * @file rpp_controller.h
 * @brief Regulated Pure Pursuit controller migrated to ROS2 Nav2.
 */
#ifndef RMP_CONTROLLER_RPP_CONTROLLER_H_
#define RMP_CONTROLLER_RPP_CONTROLLER_H_

#include <memory>
#include <string>

#include "controller_algorithm.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "utils/lookahead_point_publisher.h"

namespace rmp::controller {

struct RPPControllerConfig
{
  double control_frequency{20.0};
  double goal_dist_tolerance{0.25};
  double rotate_tolerance{0.25};

  double max_linear_velocity{0.6};
  double min_linear_velocity{0.0};
  double max_linear_velocity_increment{0.01};
  double max_angular_velocity{1.2};
  double min_angular_velocity{0.0};
  double max_angular_velocity_increment{0.2};

  double lookahead_time{1.0};
  double min_lookahead_dist{0.3};
  double max_lookahead_dist{0.9};

  double regulated_min_radius{0.9};
  double inflation_cost_factor{3.0};
  double scaling_dist{0.6};
  double scaling_gain{1.0};
  double approach_dist{0.8};
  double approach_min_v{0.1};
};

class RPPController : public ControllerAlgorithm
{
public:
  RPPController() = default;
  ~RPPController() override = default;

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
  double applyCurvatureConstraint(double raw_linear_vel, double curvature) const;
  double applyObstacleConstraint(double raw_linear_vel) const;
  double applyApproachConstraint(
    double raw_linear_vel,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & prune_plan) const;
  void readParameters();
  void prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose);
  double linearRegularization(double current, double desired) const;
  double angularRegularization(double current, double desired) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string parameter_prefix_;
  std::string controller_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<utils::LookaheadPointPublisher> lookahead_point_publisher_;
  nav_msgs::msg::Path global_plan_;
  RPPControllerConfig cfg_;
  double nominal_max_linear_velocity_{0.6};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_RPP_CONTROLLER_H_
