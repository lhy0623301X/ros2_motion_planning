/**
 * @file teb_controller.cpp
 * @brief TEB controller shell for the unified ROS2 controller framework.
 */
#include "teb_controller/teb_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "tf2/utils.h"

namespace rmp::controller {

namespace {

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

}  // namespace

void TEBController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // TEBController 只负责插件接入、参数管理和调用 optimizer。
  node_ = node;
  parameter_prefix_ = plugin_name + "." + controller_name + ".";
  controller_name_ = controller_name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  readParameters();
  nominal_max_linear_velocity_ = cfg_.max_linear_velocity;
  // footprint 由 Nav2 costmap 提供，optimizer 内部用它构造 obstacle edge。
  footprint_ = costmap_ros_ ? costmap_ros_->getRobotFootprint() : std::vector<geometry_msgs::msg::Point>{};
  optimizer_.setFootprint(footprint_);
  visualizer_ = std::make_unique<TEBVisualizer>(node_);

  RCLCPP_INFO(
    node_->get_logger(),
    "%s configured with TEB skeleton optimizer.",
    controller_name_.c_str());
}

void TEBController::cleanup()
{
  global_plan_.poses.clear();
  footprint_.clear();
}

void TEBController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "TEBController activated.");
}

void TEBController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "TEBController deactivated.");
}

void TEBController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(node_->get_logger(), "TEBController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped TEBController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "TEBController has no global plan.");
    return cmd;
  }

  // 外层 ControllerNode 已经做过目标到达、起终点对齐和坐标系统一，
  // 这里开始只关心 TEB 自己的局部优化。
  prunePlan(pose);

  TEBState state;
  state.x = pose.pose.position.x;
  state.y = pose.pose.position.y;
  state.theta = tf2::getYaw(pose.pose.orientation);
  state.v = velocity.linear.x;
  state.w = velocity.angular.z;
  footprint_ = costmap_ros_ ? costmap_ros_->getRobotFootprint() : footprint_;
  optimizer_.setFootprint(footprint_);

  // 第一步：按当前局部路径初始化一条 timed elastic band。
  auto trajectory = optimizer_.initializeTrajectory(
    global_plan_, pose, cfg_, costmap_ros_ ? costmap_ros_->getCostmap() : nullptr);
  const auto initial_trajectory = trajectory;
  // 第二步：进入 g2o 图优化，把 pose 顶点和 dt 顶点一起优化。
  const auto summary = optimizer_.optimize(
    trajectory, state, global_plan_, *costmap_ros_, cfg_);

  if (!summary.feasible) {
    std::string failure_reason = "unknown";
    (void)optimizer_.isTrajectoryFeasible(
      trajectory, *costmap_ros_, cfg_, &failure_reason);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "TEBController produced no feasible trajectory: %s | initial_states=%zu optimized_states=%zu iterations=%d total_cost=%.3f",
      failure_reason.c_str(),
      initial_trajectory.states.size(),
      trajectory.states.size(),
      summary.iterations,
      summary.total_cost);
    return cmd;
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "TEB optimize success: initial_states=%zu optimized_states=%zu iterations=%d total_cost=%.3f",
    initial_trajectory.states.size(),
    trajectory.states.size(),
    summary.iterations,
    summary.total_cost);

  // 第三步：从优化后的前两段轨迹中提取本周期控制量。
  const auto teb_command = optimizer_.extractCommand(trajectory, state, cfg_);
  cmd.twist.linear.x = linearRegularization(state.v, teb_command.v);
  cmd.twist.angular.z = angularRegularization(state.w, teb_command.w);

  if (visualizer_) {
    const auto frame_id = global_plan_.header.frame_id.empty() ?
      cmd.header.frame_id : global_plan_.header.frame_id;
    visualizer_->publishTrajectory(
      initial_trajectory, trajectory, summary, frame_id, cmd.header.stamp);
  }

  return cmd;
}

void TEBController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }

  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

void TEBController::readParameters()
{
  cfg_.control_frequency = node_->declare_parameter<double>(
    parameter_prefix_ + "control_frequency", cfg_.control_frequency);
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
  cfg_.max_linear_acceleration = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_acceleration", cfg_.max_linear_acceleration);
  cfg_.max_angular_acceleration = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_acceleration", cfg_.max_angular_acceleration);

  cfg_.dt_ref = node_->declare_parameter<double>(parameter_prefix_ + "dt_ref", cfg_.dt_ref);
  cfg_.dt_hysteresis = node_->declare_parameter<double>(
    parameter_prefix_ + "dt_hysteresis", cfg_.dt_hysteresis);
  cfg_.min_samples = node_->declare_parameter<int>(
    parameter_prefix_ + "min_samples", cfg_.min_samples);
  cfg_.max_samples = node_->declare_parameter<int>(
    parameter_prefix_ + "max_samples", cfg_.max_samples);
  cfg_.teb_length_scale = node_->declare_parameter<double>(
    parameter_prefix_ + "teb_length_scale", cfg_.teb_length_scale);
  cfg_.local_goal_distance = node_->declare_parameter<double>(
    parameter_prefix_ + "local_goal_distance", cfg_.local_goal_distance);
  cfg_.max_iterations = node_->declare_parameter<int>(
    parameter_prefix_ + "max_iterations", cfg_.max_iterations);
  cfg_.feasibility_check_poses = node_->declare_parameter<int>(
    parameter_prefix_ + "feasibility_check_poses", cfg_.feasibility_check_poses);
  cfg_.enable_final_feasibility_check = node_->declare_parameter<bool>(
    parameter_prefix_ + "enable_final_feasibility_check", cfg_.enable_final_feasibility_check);
  cfg_.convergence_epsilon = node_->declare_parameter<double>(
    parameter_prefix_ + "convergence_epsilon", cfg_.convergence_epsilon);

  cfg_.weight_kinematics = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_kinematics", cfg_.weight_kinematics);
  cfg_.weight_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_velocity", cfg_.weight_velocity);
  cfg_.weight_acceleration = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_acceleration", cfg_.weight_acceleration);
  cfg_.weight_obstacle = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_obstacle", cfg_.weight_obstacle);
  cfg_.weight_time = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_time", cfg_.weight_time);
  cfg_.weight_goal = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_goal", cfg_.weight_goal);
  cfg_.weight_smoothness = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_smoothness", cfg_.weight_smoothness);
  cfg_.weight_goal_heading = node_->declare_parameter<double>(
    parameter_prefix_ + "weight_goal_heading", cfg_.weight_goal_heading);

  cfg_.min_obstacle_distance = node_->declare_parameter<double>(
    parameter_prefix_ + "min_obstacle_distance", cfg_.min_obstacle_distance);
  cfg_.obstacle_inflation_distance = node_->declare_parameter<double>(
    parameter_prefix_ + "obstacle_inflation_distance", cfg_.obstacle_inflation_distance);
  cfg_.obstacle_hard_distance_weight = node_->declare_parameter<double>(
    parameter_prefix_ + "obstacle_hard_distance_weight", cfg_.obstacle_hard_distance_weight);
  cfg_.obstacle_soft_distance_weight = node_->declare_parameter<double>(
    parameter_prefix_ + "obstacle_soft_distance_weight", cfg_.obstacle_soft_distance_weight);
  cfg_.candidate_offset_step = node_->declare_parameter<double>(
    parameter_prefix_ + "candidate_offset_step", cfg_.candidate_offset_step);
  cfg_.candidate_max_offset = node_->declare_parameter<double>(
    parameter_prefix_ + "candidate_max_offset", cfg_.candidate_max_offset);
  cfg_.candidate_full_offset_progress = node_->declare_parameter<double>(
    parameter_prefix_ + "candidate_full_offset_progress", cfg_.candidate_full_offset_progress);
  cfg_.robot_radius = node_->declare_parameter<double>(
    parameter_prefix_ + "robot_radius", cfg_.robot_radius);
  cfg_.debug_candidate_bands = node_->declare_parameter<bool>(
    parameter_prefix_ + "debug_candidate_bands", cfg_.debug_candidate_bands);
  cfg_.unknown_as_obstacle = node_->declare_parameter<bool>(
    parameter_prefix_ + "unknown_as_obstacle", cfg_.unknown_as_obstacle);
  cfg_.allow_backward_motion = node_->declare_parameter<bool>(
    parameter_prefix_ + "allow_backward_motion", cfg_.allow_backward_motion);
}

void TEBController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2U) {
    return;
  }

  // 这里只做最小裁剪：找到离机器人最近的路径点，把已经走过的前缀去掉。
  std::size_t closest_index = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }

  if (closest_index > 0U) {
    global_plan_.poses.erase(
      global_plan_.poses.begin(),
      global_plan_.poses.begin() + static_cast<std::ptrdiff_t>(closest_index));
  }
}

double TEBController::linearRegularization(double current, double desired) const
{
  // 和其它控制器保持一致：最终输出再过一层速度变化率约束。
  const double max_delta = cfg_.max_linear_velocity_increment;
  const double limited = clamp(desired, current - max_delta, current + max_delta);
  return clamp(limited, cfg_.min_linear_velocity, cfg_.max_linear_velocity);
}

double TEBController::angularRegularization(double current, double desired) const
{
  const double max_delta = cfg_.max_angular_velocity_increment;
  const double limited = clamp(desired, current - max_delta, current + max_delta);
  return clamp(limited, -cfg_.max_angular_velocity, cfg_.max_angular_velocity);
}

}  // namespace rmp::controller
