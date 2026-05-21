/**
 * @file lqr_controller.h
 * @brief LQR local controller migrated to ROS2 Nav2.
 */
#ifndef RMP_CONTROLLER_LQR_CONTROLLER_H_
#define RMP_CONTROLLER_LQR_CONTROLLER_H_

#include <memory>
#include <string>

#include <Eigen/Dense>

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

struct LQRControllerConfig
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

  int solver_max_iterations{100};
  double solver_eps{0.1};

  double q_matrix_x{1.0};
  double q_matrix_y{1.0};
  double q_matrix_theta{1.0};
  double r_matrix_v{1.0};
  double r_matrix_w{1.0};
};

class LQRController : public ControllerAlgorithm
{
public:
  LQRController() = default;
  ~LQRController() override = default;

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
  struct ReferencePoint
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double kappa{0.0};
  };

  void readParameters();
  void prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose);
  ReferencePoint computeReferencePoint(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead_dist) const;
  Eigen::Vector2d lqrControl(
    const Eigen::Vector3d & s,
    const Eigen::Vector3d & s_d,
    const Eigen::Vector2d & u_r) const;
  double linearRegularization(double current, double desired) const;
  double angularRegularization(double current, double desired) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string parameter_prefix_;
  std::string controller_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<utils::LookaheadPointPublisher> lookahead_point_publisher_;
  nav_msgs::msg::Path global_plan_;
  LQRControllerConfig cfg_;
  double nominal_max_linear_velocity_{0.6};
  Eigen::Matrix3d Q_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix2d R_{Eigen::Matrix2d::Identity()};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_LQR_CONTROLLER_H_
