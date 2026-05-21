/**
 * @file lqr_controller.cpp
 * @brief LQR local controller migrated to ROS2 Nav2.
 */
#include "lqr_controller/lqr_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry/angles.h"
#include "geometry/point.h"
#include "geometry/vec2d.h"
#include "math/math_helper.h"
#include "tf2/utils.h"
#include "util/log.h"

namespace rmp::controller {

namespace {

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double planarDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

}  // namespace

void LQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  parameter_prefix_ = plugin_name + "." + controller_name + ".";
  controller_name_ = controller_name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  readParameters();
  nominal_max_linear_velocity_ = cfg_.max_linear_velocity;
  lookahead_point_publisher_ =
    std::make_unique<utils::LookaheadPointPublisher>(node_, "lookahead_point");

  Q_.setZero();
  Q_(0, 0) = cfg_.q_matrix_x;
  Q_(1, 1) = cfg_.q_matrix_y;
  Q_(2, 2) = cfg_.q_matrix_theta;

  R_.setZero();
  R_(0, 0) = cfg_.r_matrix_v;
  R_(1, 1) = cfg_.r_matrix_w;

  RCLCPP_INFO(node_->get_logger(), "%s configured in LQR tracking mode.", controller_name_.c_str());
}

void LQRController::cleanup()
{
  global_plan_.poses.clear();
}

void LQRController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "LQRController activated.");
}

void LQRController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "LQRController deactivated.");
}

void LQRController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(node_->get_logger(), "LQRController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    AWARN << "[LQRController] computeVelocityCommands skipped: global plan is empty.";
    return cmd;
  }

  prunePlan(pose);

  const double current_v = std::fabs(velocity.linear.x);
  const double current_w = velocity.angular.z;
  const double current_yaw = tf2::getYaw(pose.pose.orientation);
  // 速度自适应前视距离：速度越快，看得越远。
  const double lookahead_dist = clamp(
    current_v * cfg_.lookahead_time,
    cfg_.min_lookahead_dist,
    cfg_.max_lookahead_dist);

  // 参考状态 s_d = [x_ref, y_ref, theta_ref]，并顺带提取局部曲率 kappa_ref。
  const auto ref = computeReferencePoint(pose, lookahead_dist);
  if (lookahead_point_publisher_) {
    const auto frame_id = global_plan_.header.frame_id.empty() ?
      (pose.header.frame_id.empty() ? "map" : pose.header.frame_id) :
      global_plan_.header.frame_id;
    lookahead_point_publisher_->publish(ref.x, ref.y, ref.theta, frame_id, node_->now());
  }

  Eigen::Vector3d s(pose.pose.position.x, pose.pose.position.y, current_yaw);
  Eigen::Vector3d s_d(ref.x, ref.y, ref.theta);
  // 参考输入 u_r = [v_ref, w_ref]，其中 w_ref = v_ref * kappa_ref。
  Eigen::Vector2d u_r(current_v, current_v * ref.kappa);
  Eigen::Vector2d u = lqrControl(s, s_d, u_r);

  // LQR 给出的是理想控制量，最终仍需要过一层速度/角速度变化率约束。
  cmd.twist.linear.x = linearRegularization(velocity.linear.x, u[0]);
  cmd.twist.angular.z = angularRegularization(current_w, u[1]);
  return cmd;
}

void LQRController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

void LQRController::readParameters()
{
  cfg_.control_frequency = node_->declare_parameter<double>(
    parameter_prefix_ + "control_frequency", cfg_.control_frequency);
  cfg_.goal_dist_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "goal_dist_tolerance", cfg_.goal_dist_tolerance);
  cfg_.rotate_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "rotate_tolerance", cfg_.rotate_tolerance);

  cfg_.max_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity", cfg_.max_linear_velocity);
  cfg_.min_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_linear_velocity", cfg_.min_linear_velocity);
  cfg_.max_linear_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity_increment", cfg_.max_linear_velocity_increment);
  cfg_.max_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity", cfg_.max_angular_velocity);
  cfg_.min_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_angular_velocity", cfg_.min_angular_velocity);
  cfg_.max_angular_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity_increment", cfg_.max_angular_velocity_increment);

  cfg_.lookahead_time = node_->declare_parameter<double>(
    parameter_prefix_ + "lookahead_time", cfg_.lookahead_time);
  cfg_.min_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "min_lookahead_dist", cfg_.min_lookahead_dist);
  cfg_.max_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "max_lookahead_dist", cfg_.max_lookahead_dist);

  cfg_.solver_max_iterations = node_->declare_parameter<int>(
    parameter_prefix_ + "solver_max_iterations", cfg_.solver_max_iterations);
  cfg_.solver_eps = node_->declare_parameter<double>(
    parameter_prefix_ + "solver_eps", cfg_.solver_eps);
  cfg_.q_matrix_x = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_x", cfg_.q_matrix_x);
  cfg_.q_matrix_y = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_y", cfg_.q_matrix_y);
  cfg_.q_matrix_theta = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_theta", cfg_.q_matrix_theta);
  cfg_.r_matrix_v = node_->declare_parameter<double>(
    parameter_prefix_ + "r_matrix_v", cfg_.r_matrix_v);
  cfg_.r_matrix_w = node_->declare_parameter<double>(
    parameter_prefix_ + "r_matrix_w", cfg_.r_matrix_w);
}

void LQRController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2) {
    return;
  }

  const double search_distance = costmap_ros_ && costmap_ros_->getCostmap() ?
    costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0 :
    std::numeric_limits<double>::max();

  auto search_end = global_plan_.poses.end();
  double integrated = 0.0;
  for (auto it = global_plan_.poses.begin(); it + 1 != global_plan_.poses.end(); ++it) {
    integrated += planarDistance(*it, *(it + 1));
    if (integrated > search_distance) {
      search_end = it + 1;
      break;
    }
  }

  auto closest = std::min_element(
    global_plan_.poses.begin(), search_end,
    [&](const auto & lhs, const auto & rhs) {
      return planarDistance(robot_pose, lhs) < planarDistance(robot_pose, rhs);
    });

  if (closest != global_plan_.poses.begin() && closest != global_plan_.poses.end()) {
    global_plan_.poses.erase(global_plan_.poses.begin(), closest);
  }
}

LQRController::ReferencePoint LQRController::computeReferencePoint(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_dist) const
{
  if (global_plan_.poses.empty()) {
    throw std::invalid_argument("Cannot compute LQR reference point from an empty path.");
  }

  using rmp::common::geometry::Point3d;
  using rmp::common::geometry::Vec2d;
  using rmp::common::math::arcCenter;
  using rmp::common::math::circleSegmentIntersection;

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  auto goal_pose_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & ps) {
      return std::hypot(
        ps.pose.position.x - rx,
        ps.pose.position.y - ry) >= lookahead_dist;
    });

  Point3d lookahead_pt;
  double kappa = 0.0;

  if (goal_pose_it == global_plan_.poses.end()) {
    // 路径剩余长度不足 lookahead 时，退化为直接跟踪最后一个点。
    goal_pose_it = std::prev(global_plan_.poses.end());
    lookahead_pt.setX(goal_pose_it->pose.position.x);
    lookahead_pt.setY(goal_pose_it->pose.position.y);
    lookahead_pt.setTheta(std::atan2(lookahead_pt.y() - ry, lookahead_pt.x() - rx));
  } else {
    double px;
    double py;
    const double gx = goal_pose_it->pose.position.x;
    const double gy = goal_pose_it->pose.position.y;
    if (goal_pose_it == global_plan_.poses.begin()) {
      px = rx;
      py = ry;
    } else {
      auto prev_pose_it = std::prev(goal_pose_it);
      px = prev_pose_it->pose.position.x;
      py = prev_pose_it->pose.position.y;
    }

    // 求“路径段与前视圆”的交点，
    // 这样参考点会更贴近真实前视距离，而不是跳在某个栅格点上。
    Vec2d prev_p(px - rx, py - ry);
    Vec2d goal_p(gx - rx, gy - ry);
    const auto intersection_points =
      circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    double dist_to_goal = std::numeric_limits<double>::max();
    for (const auto & p : intersection_points) {
      const double dist = std::hypot(p.x() + rx - gx, p.y() + ry - gy);
      if (dist < dist_to_goal) {
        dist_to_goal = dist;
        lookahead_pt.setX(p.x() + rx);
        lookahead_pt.setY(p.y() + ry);
      }
    }

    auto next_pose_it = std::next(goal_pose_it);
    if (next_pose_it != global_plan_.poses.end()) {
      // 用相邻三点估计参考曲率，为参考角速度 w_ref = v_ref * kappa_ref 服务。
      Vec2d p1(px, py);
      Vec2d p2(gx, gy);
      Vec2d p3(next_pose_it->pose.position.x, next_pose_it->pose.position.y);
      kappa = arcCenter(p1, p2, p3, false);
      if (!std::isfinite(kappa)) {
        kappa = 0.0;
      }
    }
    // 参考朝向改为当前位置指向参考点的连线方向，
    // 让 LQR 的参考姿态和当前控制周期的“前视目标方向”保持一致。
    lookahead_pt.setTheta(std::atan2(lookahead_pt.y() - ry, lookahead_pt.x() - rx));
  }

  return {lookahead_pt.x(), lookahead_pt.y(), lookahead_pt.theta(), kappa};
}

Eigen::Vector2d LQRController::lqrControl(
  const Eigen::Vector3d & s,
  const Eigen::Vector3d & s_d,
  const Eigen::Vector2d & u_r) const
{
  using rmp::common::geometry::normalizeAngle;

  Eigen::Vector3d e = s - s_d;
  e[2] = normalizeAngle(e[2]);

  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);

  // 围绕参考状态/参考输入做一阶离散线性化，保持与原 ROS1 LQR 一致。
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * std::sin(s_d[2]) * dt;
  A(1, 2) = u_r[0] * std::cos(s_d[2]) * dt;

  Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
  B(0, 0) = std::cos(s_d[2]) * dt;
  B(1, 0) = std::sin(s_d[2]) * dt;
  B(2, 1) = dt;
  const auto B_trans = B.transpose();

  Eigen::Matrix3d P = Q_;
  Eigen::Matrix3d P_next = Q_;
  const int max_iterations = std::max(1, cfg_.solver_max_iterations);
  const double solver_eps = std::max(1.0e-9, cfg_.solver_eps);

  // 迭代求解离散 Riccati 方程，得到稳定反馈增益所需的 P。
  for (int i = 0; i < max_iterations; ++i) {
    const Eigen::Matrix2d temp = R_ + B_trans * P * B;
    P_next = Q_ + A.transpose() * P * A -
      A.transpose() * P * B * temp.inverse() * B_trans * P * A;
    if ((P - P_next).array().abs().maxCoeff() < solver_eps) {
      break;
    }
    P = P_next;
  }

  // 反馈律：u = u_r + K * e
  const Eigen::Matrix<double, 2, 3> K =
    -(R_ + B_trans * P_next * B).inverse() * B_trans * P_next * A;
  return u_r + K * e;
}

double LQRController::linearRegularization(double current, double desired) const
{
  // 先限单周期增量，再限绝对速度，避免 LQR 输出直接产生过大的速度跳变。
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_linear_velocity_increment,
    cfg_.max_linear_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_linear_velocity,
    cfg_.max_linear_velocity);
  if (std::fabs(cmd) < cfg_.min_linear_velocity) {
    return 0.0;
  }
  return cmd;
}

double LQRController::angularRegularization(double current, double desired) const
{
  // 角速度同理：先限目标幅值，再限单周期变化率。
  desired = clamp(
    desired,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_angular_velocity_increment,
    cfg_.max_angular_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  if (std::fabs(cmd) < cfg_.min_angular_velocity) {
    return 0.0;
  }
  return cmd;
}

}  // namespace rmp::controller
