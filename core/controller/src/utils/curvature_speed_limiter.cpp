/**
 * @file curvature_speed_limiter.cpp
 * @brief Limit forward speed by local path curvature.
 */
#include "utils/curvature_speed_limiter.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "util/log.h"

namespace rmp::controller::utils {

namespace {

double planarDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

}  // namespace

void CurvatureSpeedLimiter::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_prefix)
{
  cfg_.enabled = node->declare_parameter<bool>(
    parameter_prefix + "enabled", cfg_.enabled);
  cfg_.max_lateral_accel = node->declare_parameter<double>(
    parameter_prefix + "max_lateral_accel", cfg_.max_lateral_accel);
  cfg_.min_linear_velocity = node->declare_parameter<double>(
    parameter_prefix + "min_linear_velocity", cfg_.min_linear_velocity);
  cfg_.curvature_epsilon = node->declare_parameter<double>(
    parameter_prefix + "curvature_epsilon", cfg_.curvature_epsilon);
  cfg_.sample_distance = node->declare_parameter<double>(
    parameter_prefix + "sample_distance", cfg_.sample_distance);
  cfg_.preview_distance = node->declare_parameter<double>(
    parameter_prefix + "preview_distance", cfg_.preview_distance);

  AINFO << "[CurvatureSpeedLimiter] configured: prefix=" << parameter_prefix
        << ", enabled=" << cfg_.enabled
        << ", max_lateral_accel=" << cfg_.max_lateral_accel
        << ", min_linear_velocity=" << cfg_.min_linear_velocity
        << ", curvature_epsilon=" << cfg_.curvature_epsilon
        << ", sample_distance=" << cfg_.sample_distance
        << ", preview_distance=" << cfg_.preview_distance;
}

double CurvatureSpeedLimiter::limitSpeed(
  double desired_v,
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // Step 1: 基本早退检查。
  // 如果模块关闭、路径点不足，或者当前期望速度本来就是 0，就直接放行。
  if (!cfg_.enabled || path.poses.size() < 3) {
    return desired_v;
  }

  const double desired_speed = std::fabs(desired_v);
  if (desired_speed <= 0.0) {
    return desired_v;
  }

  // Step 2: 估计机器人前方预览窗口里的最大曲率。
  // 这里取“前方最大曲率”而不是单点曲率，是为了在临近急弯前就提前降速。
  const double max_curvature = estimateMaxCurvatureAhead(path, robot_pose);
  if (!std::isfinite(max_curvature) || max_curvature <= cfg_.curvature_epsilon) {
    return desired_v;
  }

  // Step 3: 用横向加速度约束把曲率转换成速度上限。
  // 基本模型：a_lat = v^2 * kappa  =>  v_max = sqrt(a_lat_max / kappa)
  const double max_lateral_accel = std::max(1.0e-6, cfg_.max_lateral_accel);
  const double curvature_limited_speed =
    std::sqrt(max_lateral_accel / std::max(max_curvature, cfg_.curvature_epsilon));

  // Step 4: 将曲率允许的最大速度和控制器当前输出速度比较，取更保守的那个。
  double limited_speed = std::min(desired_speed, curvature_limited_speed);

  // Step 5: 应用最小速度阈值。
  // 如果限完之后已经非常小，就直接清零，避免低速抖动。
  if (limited_speed < cfg_.min_linear_velocity) {
    limited_speed = 0.0;
  }

  // Step 6: 恢复原始前进/后退方向。
  // 当前项目主要控制前向速度，但这里保留符号，接口会更通用。
  const double limited_v = desired_v < 0.0 ? -limited_speed : limited_speed;
  AINFO_EVERY(20) << "[CurvatureSpeedLimiter] limiting speed: desired_v=" << desired_v
                  << ", max_curvature=" << max_curvature
                  << ", curvature_limited_speed=" << curvature_limited_speed
                  << ", limited_v=" << limited_v;
  return limited_v;
}

double CurvatureSpeedLimiter::estimateMaxCurvatureAhead(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const std::size_t closest_index = findClosestPoseIndex(path, robot_pose);
  const double sample_distance = std::max(1.0e-3, cfg_.sample_distance);
  const double preview_distance = std::max(sample_distance * 2.0, cfg_.preview_distance);

  double max_curvature = 0.0;
  for (double offset = 0.0; offset + 2.0 * sample_distance <= preview_distance;
       offset += sample_distance)
  {
    const std::size_t p0_index = advanceByDistance(path, closest_index, offset);
    const std::size_t p1_index = advanceByDistance(path, p0_index, sample_distance);
    const std::size_t p2_index = advanceByDistance(path, p1_index, sample_distance);
    if (p0_index == p1_index || p1_index == p2_index) {
      break;
    }

    max_curvature = std::max(
      max_curvature,
      computeCurvature(path.poses[p0_index], path.poses[p1_index], path.poses[p2_index]));
  }

  return max_curvature;
}

std::size_t CurvatureSpeedLimiter::findClosestPoseIndex(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  std::size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double distance = planarDistance(robot_pose, path.poses[i]);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

std::size_t CurvatureSpeedLimiter::advanceByDistance(
  const nav_msgs::msg::Path & path,
  std::size_t start_index,
  double target_distance) const
{
  if (path.poses.empty() || start_index >= path.poses.size() - 1 || target_distance <= 0.0) {
    return std::min(start_index, path.poses.empty() ? std::size_t{0} : path.poses.size() - 1);
  }

  double accumulated = 0.0;
  for (std::size_t i = start_index; i + 1 < path.poses.size(); ++i) {
    accumulated += planarDistance(path.poses[i], path.poses[i + 1]);
    if (accumulated >= target_distance) {
      return i + 1;
    }
  }
  return path.poses.size() - 1;
}

double CurvatureSpeedLimiter::computeCurvature(
  const geometry_msgs::msg::PoseStamped & p0,
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2) const
{
  const double a = planarDistance(p0, p1);
  const double b = planarDistance(p1, p2);
  const double c = planarDistance(p0, p2);
  if (a <= 1.0e-6 || b <= 1.0e-6 || c <= 1.0e-6) {
    return 0.0;
  }

  const double x1 = p1.pose.position.x - p0.pose.position.x;
  const double y1 = p1.pose.position.y - p0.pose.position.y;
  const double x2 = p2.pose.position.x - p0.pose.position.x;
  const double y2 = p2.pose.position.y - p0.pose.position.y;
  const double twice_area = std::fabs(x1 * y2 - y1 * x2);
  if (twice_area <= 1.0e-9) {
    return 0.0;
  }

  return 2.0 * twice_area / (a * b * c);
}

}  // namespace rmp::controller::utils
