/**
 * @file pid_controller.cpp
 * @brief PID local controller plugin migrated to ROS2 Nav2.
 */
#include "pid_controller/pid_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "util/log.h"

namespace rmp::controller {

namespace {

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double planarDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

bool shouldResetIntegral(double previous_error, double current_error)
{
  constexpr double kZeroErrorEpsilon = 1e-6;
  if (std::fabs(current_error) <= kZeroErrorEpsilon) {
    return true;
  }
  if (std::fabs(previous_error) <= kZeroErrorEpsilon) {
    return false;
  }
  return previous_error * current_error < 0.0;
}

}  // namespace

void PIDController::configure(
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
    std::make_unique<utils::LookaheadPointPublisher>(node_, "pid_lookahead_point");
  resetPidState();

  RCLCPP_INFO(
    node_->get_logger(),
    "%s configured in dual-channel PID mode.",
    controller_name_.c_str());
  AINFO << "[PIDController] configured: plugin=" << plugin_name
        << ", controller=" << controller_name_
        << ", parameter_prefix=" << parameter_prefix_
        << ", control_frequency=" << cfg_.control_frequency
        << ", goal_dist_tolerance=" << cfg_.goal_dist_tolerance
        << ", rotate_tolerance=" << cfg_.rotate_tolerance
        << ", max_linear_velocity=" << cfg_.max_linear_velocity
        << ", max_linear_velocity_increment=" << cfg_.max_linear_velocity_increment
        << ", max_angular_velocity=" << cfg_.max_angular_velocity
        << ", max_angular_velocity_increment=" << cfg_.max_angular_velocity_increment
        << ", lookahead_time=" << cfg_.lookahead_time
        << ", min_lookahead_dist=" << cfg_.min_lookahead_dist
        << ", max_lookahead_dist=" << cfg_.max_lookahead_dist
        << ", linear_pid=(" << cfg_.p_linear_velocity << ", "
        << cfg_.i_linear_velocity << ", " << cfg_.d_linear_velocity << ")"
        << ", angular_pid=(" << cfg_.p_angular_velocity << ", "
        << cfg_.i_angular_velocity << ", " << cfg_.d_angular_velocity << ")";
}

void PIDController::cleanup()
{
  global_plan_.poses.clear();
  resetPidState();
}

void PIDController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "PIDController activated.");
}

void PIDController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "PIDController deactivated.");
}

void PIDController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_reached_ = false;
  if (global_plan_.poses.empty()) {
    resetPidState();
  }
  RCLCPP_INFO(node_->get_logger(), "PIDController received path with %zu poses.", path.poses.size());
  if (!global_plan_.poses.empty()) {
    const auto & first = global_plan_.poses.front().pose.position;
    const auto & last = global_plan_.poses.back().pose.position;
    AINFO << "[PIDController] setPlan: poses=" << global_plan_.poses.size()
          << ", first=(" << first.x << ", " << first.y << ")"
          << ", last=(" << last.x << ", " << last.y << ")";
  } else {
    AWARN << "[PIDController] setPlan: received empty path.";
  }
}

geometry_msgs::msg::TwistStamped PIDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    AWARN << "[PIDController] computeVelocityCommands skipped: global plan is empty.";
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "PIDController has no global plan.");
    return cmd;
  }

  const auto plan_frame_pose = transformPoseToPlanFrame(pose);

  const auto & goal_pose = global_plan_.poses.back();
  if (goal_checker && goal_checker->isGoalReached(plan_frame_pose.pose, goal_pose.pose, velocity)) {
    goal_reached_ = true;
    resetPidState();
    AINFO << "[PIDController] goal reached by Nav2 goal checker. pose=("
          << plan_frame_pose.pose.position.x << ", " << plan_frame_pose.pose.position.y << ")";
    return cmd;
  }

  prunePlan(plan_frame_pose);
  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);
  const double current_v = velocity.linear.x;
  const double current_w = velocity.angular.z;
  const double current_yaw = tf2::getYaw(plan_frame_pose.pose.orientation);

  // 如果已接近终点，原地旋转对准目标朝向
  if (shouldRotateToGoal(plan_frame_pose)) {
    const double heading_error = normalizeAngle(goalYaw() - current_yaw);
    if (!shouldRotateToPath(std::fabs(heading_error))) {
      goal_reached_ = true;
      resetPidState();
      AINFO << "[PIDController] goal orientation reached: heading_error="
            << heading_error << ", rotate_tolerance=" << cfg_.rotate_tolerance;
      return cmd;
    }
    cmd.twist.angular.z = angularRegularization(current_w, heading_error / dt);
    AINFO_EVERY(20) << "[PIDController] rotate-to-goal: heading_error="
                    << heading_error << ", current_w=" << current_w
                    << ", cmd_w=" << cmd.twist.angular.z;
    return cmd;
  }

  // 正常跟踪：计算预瞄点，执行 PID 控制

  const double lookahead_dist = clamp(
    std::fabs(current_v) * cfg_.lookahead_time,
    cfg_.min_lookahead_dist,
    cfg_.max_lookahead_dist);
  const TrackingPoint target = getLookAheadPoint(lookahead_dist, plan_frame_pose);
  if (lookahead_point_publisher_) {
    const auto frame_id = global_plan_.header.frame_id.empty() ?
      (plan_frame_pose.header.frame_id.empty() ? "map" : plan_frame_pose.header.frame_id) :
      global_plan_.header.frame_id;
    lookahead_point_publisher_->publish(target.x, target.y, target.theta, frame_id, node_->now());
  }

  const Eigen::Vector2d control = dualChannelPIDControl(
    plan_frame_pose, target, current_yaw, current_v, current_w);

  cmd.twist.linear.x = linearRegularization(current_v, control[0]);
  cmd.twist.angular.z = angularRegularization(current_w, control[1]);
  [[maybe_unused]] const double angular_accel = (cmd.twist.angular.z - current_w) *
    std::max(1.0, cfg_.control_frequency);
  const double target_heading = std::atan2(
    target.y - plan_frame_pose.pose.position.y,
    target.x - plan_frame_pose.pose.position.x);
  [[maybe_unused]] const double heading_error = normalizeAngle(target_heading - current_yaw);
  AINFO_EVERY(20) << "[PIDController] tracking: pose=("
                  << plan_frame_pose.pose.position.x << ", " << plan_frame_pose.pose.position.y
                  << ", frame=" << plan_frame_pose.header.frame_id
                  << ", lookahead_dist=" << lookahead_dist
                  << ", target=(" << target.x << ", " << target.y
                  << ", heading=" << target_heading << ")";
  return cmd;
}

void PIDController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    AINFO << "[PIDController] speed limit reset: max_linear_velocity="
          << cfg_.max_linear_velocity;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
  AINFO << "[PIDController] speed limit updated: input=" << speed_limit
        << ", percentage=" << percentage
        << ", max_linear_velocity=" << cfg_.max_linear_velocity;
}

void PIDController::readParameters()
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

  cfg_.p_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "p_linear_velocity", cfg_.p_linear_velocity);
  cfg_.i_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "i_linear_velocity", cfg_.i_linear_velocity);
  cfg_.d_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "d_linear_velocity", cfg_.d_linear_velocity);
  cfg_.p_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "p_angular_velocity", cfg_.p_angular_velocity);
  cfg_.i_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "i_angular_velocity", cfg_.i_angular_velocity);
  cfg_.d_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "d_angular_velocity", cfg_.d_angular_velocity);

  cfg_.k_feedback = node_->declare_parameter<double>(
    parameter_prefix_ + "k_feedback", cfg_.k_feedback);
  cfg_.dist_from_center_to_front_edge = node_->declare_parameter<double>(
    parameter_prefix_ + "dist_from_center_to_front_edge", cfg_.dist_from_center_to_front_edge);
  cfg_.model_based_mode = node_->declare_parameter<bool>(
    parameter_prefix_ + "model_based_mode", cfg_.model_based_mode);
  cfg_.heading_slowdown_start = node_->declare_parameter<double>(
    parameter_prefix_ + "heading_slowdown_start", cfg_.heading_slowdown_start);
  cfg_.heading_slowdown_end = node_->declare_parameter<double>(
    parameter_prefix_ + "heading_slowdown_end", cfg_.heading_slowdown_end);
  cfg_.heading_stop_threshold = node_->declare_parameter<double>(
    parameter_prefix_ + "heading_stop_threshold", cfg_.heading_stop_threshold);
  cfg_.min_heading_slowdown_speed_ratio = node_->declare_parameter<double>(
    parameter_prefix_ + "min_heading_slowdown_speed_ratio",
    cfg_.min_heading_slowdown_speed_ratio);
  AINFO << "[PIDController] parameters loaded from prefix " << parameter_prefix_
        << ": max_v=" << cfg_.max_linear_velocity
        << ", max_w=" << cfg_.max_angular_velocity
        << ", dv_limit=" << cfg_.max_linear_velocity_increment
        << ", dw_limit=" << cfg_.max_angular_velocity_increment
        << ", lookahead=(" << cfg_.lookahead_time << ", "
        << cfg_.min_lookahead_dist << ", " << cfg_.max_lookahead_dist << ")"
        << ", linear_pid=(" << cfg_.p_linear_velocity << ", "
        << cfg_.i_linear_velocity << ", " << cfg_.d_linear_velocity << ")"
        << ", angular_pid=(" << cfg_.p_angular_velocity << ", "
        << cfg_.i_angular_velocity << ", " << cfg_.d_angular_velocity << ")"
        << ", heading_slowdown=(start=" << cfg_.heading_slowdown_start
        << ", end=" << cfg_.heading_slowdown_end
        << ", stop=" << cfg_.heading_stop_threshold
        << ", min_ratio=" << cfg_.min_heading_slowdown_speed_ratio << ")";
}

void PIDController::resetPidState()
{
  e_v_ = 0.0;
  e_w_ = 0.0;
  i_v_ = 0.0;
  i_w_ = 0.0;
}

geometry_msgs::msg::PoseStamped PIDController::transformPoseToPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  const auto & plan_frame = global_plan_.header.frame_id;
  if (plan_frame.empty() || pose.header.frame_id.empty() || pose.header.frame_id == plan_frame) {
    return pose;
  }

  try {
    return tf_->transform(pose, plan_frame, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    AWARN << "[PIDController] failed to transform robot pose from "
          << pose.header.frame_id << " to " << plan_frame
          << ": " << ex.what()
          << ". Falling back to the original pose; tracking may have frame error.";
    return pose;
  }
}

void PIDController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2) {
    return;
  }

  // 限定搜索范围：只在前方半个 costmap 宽度内查找最近点，
  // 避免机器人在路径后方时错误地追溯到远处已走过的点。
  const double search_distance = costmap_ros_ && costmap_ros_->getCostmap() ?
    costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0 :
    std::numeric_limits<double>::max();

  // 按路径积分距离截断搜索窗口
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

  // 删除最近点之前的所有点，使路径从机器人前方开始
  if (closest != global_plan_.poses.begin() && closest != global_plan_.poses.end()) {
    const auto pruned_count = std::distance(global_plan_.poses.begin(), closest);
    global_plan_.poses.erase(global_plan_.poses.begin(), closest);
    AINFO_EVERY(20) << "[PIDController] pruned global plan: removed="
                    << pruned_count << ", remaining=" << global_plan_.poses.size();
  }
}

PIDController::TrackingPoint PIDController::getLookAheadPoint(
  double lookahead_dist,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  // 沿路径查找第一个距离机器人 >= lookahead_dist 的点作为预瞄点
  auto target_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & plan_pose) {
      return std::hypot(plan_pose.pose.position.x - rx, plan_pose.pose.position.y - ry) >=
             lookahead_dist;
    });

  // 路径太短时退化为瞄准终点
  if (target_it == global_plan_.poses.end()) {
    target_it = std::prev(global_plan_.poses.end());
  }

  TrackingPoint target;
  target.x = target_it->pose.position.x;
  target.y = target_it->pose.position.y;

  // 预瞄方向取目标点指向下一个路径点的方向，保证前瞻性；
  // 若已是最后一个点，则用本身的 yaw 或机器人到目标的方向作为 fallback。
  if (target_it + 1 != global_plan_.poses.end()) {
    const auto & next = *(target_it + 1);
    target.theta = std::atan2(
      next.pose.position.y - target.y,
      next.pose.position.x - target.x);
  } else {
    target.theta = tf2::getYaw(target_it->pose.orientation);
    if (!std::isfinite(target.theta)) {
      target.theta = std::atan2(target.y - ry, target.x - rx);
    }
  }
  return target;
}

Eigen::Vector2d PIDController::dualChannelPIDControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const TrackingPoint & target,
  double current_yaw,
  double current_v,
  double current_w)
{
  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);

  // 横向通道：预瞄点只用来生成目标朝向角，角速度由航向误差闭环得到。
  // 这样横向控制通过调整车头朝向间接把机器人拉回路径。
  const double target_heading = std::atan2(
    target.y - pose.pose.position.y,
    target.x - pose.pose.position.x);
  const double e_w = normalizeAngle(target_heading - current_yaw);

  // 纵向通道：基础期望速度为最大速度，但会根据航向误差降速。
  // 车头偏差较小时正常巡航；偏差较大时低速修正；接近反向时原地转向。
  const double abs_heading_error = std::fabs(e_w);
  double speed_ratio = 1.0;
  if (abs_heading_error >= cfg_.heading_stop_threshold) {
    speed_ratio = 0.0;
  } else if (abs_heading_error > cfg_.heading_slowdown_start) {
    const double denom = std::max(
      1e-6, cfg_.heading_slowdown_end - cfg_.heading_slowdown_start);
    const double progress = clamp(
      (abs_heading_error - cfg_.heading_slowdown_start) / denom, 0.0, 1.0);
    speed_ratio = 1.0 - progress * (1.0 - cfg_.min_heading_slowdown_speed_ratio);
  }

  const double desired_v = cfg_.max_linear_velocity * speed_ratio;
  const double e_v = desired_v - current_v;

  if (shouldResetIntegral(e_v_, e_v)) {
    i_v_ = 0.0;
  }
  if (shouldResetIntegral(e_w_, e_w)) {
    i_w_ = 0.0;
  }

  i_v_ += e_v * dt;
  i_w_ += e_w * dt;

  const double d_v = (e_v - e_v_) / dt;
  const double d_w = normalizeAngle(e_w - e_w_) / dt;
  e_v_ = e_v;
  e_w_ = e_w;

  // 线速度输出绝对目标速度：以期望速度作为前馈，PID 只负责根据速度误差修正。
  // 这里不再叠加 current_v，避免把控制量解释成“速度增量”。
  double v_cmd = desired_v +
    cfg_.p_linear_velocity * e_v +
    cfg_.i_linear_velocity * i_v_ +
    cfg_.d_linear_velocity * d_v;
  v_cmd = clamp(v_cmd, 0.0, std::max(0.0, desired_v));
  const double p_w = e_w;
  const double i_w = i_w_;
  const double d_w_raw = d_w;

  // 角速度输出绝对目标角速度：PID 直接给出目标 yaw rate。
  // 后续 angularRegularization 只做角速度和角加速度约束。
  const double w_cmd =
    cfg_.p_angular_velocity * p_w +
    cfg_.i_angular_velocity * i_w +
    cfg_.d_angular_velocity * d_w_raw;

  AINFO_EVERY(20) << "[PIDController] dual-channel PID: desired_v=" << desired_v
                  << ", current_v=" << current_v
                  << ", speed_error=" << e_v
                  << ", speed_integral=" << i_v_
                  << ", speed_derivative=" << d_v
                  << ", target_heading=" << target_heading
                  << ", current_yaw=" << current_yaw
                  << ", heading_error=" << e_w
                  << ", heading_integral=" << i_w_
                  << ", heading_derivative=" << d_w
                  << ", angular_terms_unweighted=(p=" << p_w
                  << ", i=" << i_w
                  << ", d=" << d_w_raw << ")"
                  << ", actual_w=" << current_w
                  << ", target_angular_accel=" << (w_cmd - current_w) / dt
                  << ", raw_output=(" << v_cmd << ", " << w_cmd << ")";

  return {v_cmd, w_cmd};
}

double PIDController::linearRegularization(double current, double desired) const
{
  // 限幅增量，防止加速度过大
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_linear_velocity_increment,
    cfg_.max_linear_velocity_increment);
  // 限幅最终值
  double cmd = clamp(
    current + inc,
    -cfg_.max_linear_velocity,
    cfg_.max_linear_velocity);

  // 死区：低于最小速度则输出 0
  if (std::fabs(cmd) < cfg_.min_linear_velocity) {
    return 0.0;
  }
  return cmd;
}

double PIDController::angularRegularization(double current, double desired) const
{
  // 先对期望值限幅
  desired = clamp(
    desired,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  // 限幅增量
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_angular_velocity_increment,
    cfg_.max_angular_velocity_increment);
  // 限幅最终值
  double cmd = clamp(
    current + inc,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);

  if (std::fabs(cmd) < cfg_.min_angular_velocity) {
    return 0.0;
  }
  return cmd;
}

bool PIDController::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped & pose) const
{
  return planarDistance(pose, global_plan_.poses.back()) < cfg_.goal_dist_tolerance;
}

bool PIDController::shouldRotateToPath(double heading_error) const
{
  return heading_error > cfg_.rotate_tolerance;
}

double PIDController::goalYaw() const
{
  return tf2::getYaw(global_plan_.poses.back().pose.orientation);
}

}  // namespace rmp::controller
