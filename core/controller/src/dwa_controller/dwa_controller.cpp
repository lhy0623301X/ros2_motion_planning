/**
 * @file dwa_controller.cpp
 * @brief Minimal DWA controller implemented for the ROS2 controller framework.
 */
#include "dwa_controller/dwa_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "dwa_controller/dwa_critic.h"
#include "dwa_controller/dwa_motion.h"
#include "dwa_controller/dwa_visualizer.h"
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

void DWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 步骤 1：接入 Nav2 controller 插件上下文，并按 FollowPath.DWA.* 读取参数。
  node_ = node;
  parameter_prefix_ = plugin_name + "." + controller_name + ".";
  controller_name_ = controller_name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  readParameters();
  nominal_max_linear_velocity_ = cfg_.max_linear_velocity;
  visualizer_ = std::make_unique<DWAVisualizer>(node_, "dwa_trajectories");
  RCLCPP_INFO(node_->get_logger(), "%s configured in minimal DWA mode.", controller_name_.c_str());
}

void DWAController::cleanup()
{
  global_plan_.poses.clear();
  visualizer_.reset();
}

void DWAController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "DWAController activated.");
}

void DWAController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "DWAController deactivated.");
}

void DWAController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(node_->get_logger(), "DWAController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped DWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    AWARN << "[DWAController] computeVelocityCommands skipped: global plan is empty.";
    return cmd;
  }

  // 步骤 1：裁剪已走过的路径点，让 DWA 的路径评分只关注机器人前方局部路径。
  prunePlan(pose);

  // 步骤 2：把 Nav2 传入的当前位姿与速度整理成 DWA 内部状态。
  DWAState state;
  state.x = pose.pose.position.x;
  state.y = pose.pose.position.y;
  state.theta = tf2::getYaw(pose.pose.orientation);
  state.v = velocity.linear.x;
  state.w = velocity.angular.z;

  // 步骤 3：在当前动态窗口内采样轨迹，并选择综合代价最低的合法轨迹。
  const auto best = plan(state);
  if (!best.legal) {
    AWARN << "[DWAController] no legal trajectory found; publishing zero command.";
    return cmd;
  }

  // 步骤 4：DWA 输出的是采样速度，最终仍过一层速度变化率约束。
  cmd.twist.linear.x = linearRegularization(velocity.linear.x, best.command.v);
  cmd.twist.angular.z = angularRegularization(velocity.angular.z, best.command.w);
  AINFO_EVERY(20) << "[DWAController] best command: v=" << best.command.v
                  << ", w=" << best.command.w
                  << ", score=" << best.score
                  << ", cmd=(" << cmd.twist.linear.x << ", " << cmd.twist.angular.z << ")";
  return cmd;
}

void DWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

void DWAController::readParameters()
{
  // 所有参数保持在 FollowPath.DWA.* 下，避免额外配置文件或动态配置系统。
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

  cfg_.sim_time = node_->declare_parameter<double>(
    parameter_prefix_ + "sim_time", cfg_.sim_time);
  cfg_.sim_time_step = node_->declare_parameter<double>(
    parameter_prefix_ + "sim_time_step", cfg_.sim_time_step);
  cfg_.vx_samples = node_->declare_parameter<int>(
    parameter_prefix_ + "vx_samples", cfg_.vx_samples);
  cfg_.vtheta_samples = node_->declare_parameter<int>(
    parameter_prefix_ + "vtheta_samples", cfg_.vtheta_samples);

  cfg_.path_distance_bias = node_->declare_parameter<double>(
    parameter_prefix_ + "path_distance_bias", cfg_.path_distance_bias);
  cfg_.goal_distance_bias = node_->declare_parameter<double>(
    parameter_prefix_ + "goal_distance_bias", cfg_.goal_distance_bias);
  cfg_.obstacle_distance_bias = node_->declare_parameter<double>(
    parameter_prefix_ + "obstacle_distance_bias", cfg_.obstacle_distance_bias);
  cfg_.velocity_bias = node_->declare_parameter<double>(
    parameter_prefix_ + "velocity_bias", cfg_.velocity_bias);
  cfg_.twirling_bias = node_->declare_parameter<double>(
    parameter_prefix_ + "twirling_bias", cfg_.twirling_bias);
  cfg_.unknown_as_obstacle = node_->declare_parameter<bool>(
    parameter_prefix_ + "unknown_as_obstacle", cfg_.unknown_as_obstacle);
}

void DWAController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2) {
    return;
  }

  // 只在局部 costmap 半径附近找最近点，避免机器人靠近路径后段时误裁剪。
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

DWATrajectory DWAController::plan(const DWAState & state) const
{
  // 步骤 1：根据当前速度和速度变化率限制，得到本周期可采样的动态窗口。
  const auto window = dwa_motion::calcDynamicWindow(cfg_, state);

  // 步骤 2：在动态窗口内采样 v/w，并把每个速度直接前向积分成轨迹。
  auto trajectories = dwa_motion::generateTrajectorySamples(cfg_, state, window);

  DWATrajectory best;
  best.score = std::numeric_limits<double>::infinity();
  best.legal = false;

  const auto * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  for (auto & trajectory : trajectories) {
    // 步骤 3：碰撞轨迹直接丢弃，合法轨迹按综合代价取最小。
    const double score = dwa_critic::evaluateTrajectory(
      trajectory, global_plan_, costmap, cfg_);
    if (!std::isfinite(score)) {
      continue;
    }
    trajectory.score = score;
    trajectory.legal = true;
    if (score < best.score) {
      best = trajectory;
    }
  }

  // 步骤 4：发布本周期采样轨迹与最终选择，便于在 RViz 中观察 DWA 决策过程。
  if (visualizer_) {
    std::string frame_id = global_plan_.header.frame_id;
    if (frame_id.empty() && !global_plan_.poses.empty()) {
      frame_id = global_plan_.poses.front().header.frame_id;
    }
    if (frame_id.empty() && costmap_ros_) {
      frame_id = costmap_ros_->getGlobalFrameID();
    }
    visualizer_->publish(trajectories, best, frame_id, node_->now());
  }
  return best;
}

double DWAController::linearRegularization(double current, double desired) const
{
  // DWA 已经在动态窗口里约束速度，这里再做最终输出保护，和其他控制器保持一致。
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

double DWAController::angularRegularization(double current, double desired) const
{
  // 先限制目标角速度，再限制单周期角速度增量。
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
