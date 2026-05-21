/**
 * @file dwa_critic.cpp
 * @brief Basic trajectory scoring utilities for the minimal DWA controller.
 */
#include "dwa_controller/dwa_critic.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::controller::dwa_critic {

namespace {

double distanceToPose(
  const DWATrajectoryPoint & point,
  const geometry_msgs::msg::PoseStamped & pose)
{
  return std::hypot(point.x - pose.pose.position.x, point.y - pose.pose.position.y);
}

double distanceToPoint(
  const DWATrajectoryPoint & lhs,
  const DWATrajectoryPoint & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

const DWATrajectoryPoint * terminalPoint(const DWATrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return nullptr;
  }
  return &trajectory.points.back();
}

unsigned char costAtWorld(
  const nav2_costmap_2d::Costmap2D * costmap,
  double x,
  double y,
  bool unknown_as_obstacle)
{
  // 世界坐标超出 costmap 时直接视为不可行，避免轨迹跑出局部地图。
  if (!costmap) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const unsigned char cost = costmap->getCost(mx, my);
  // 未知区域是否可走由参数决定；默认保持宽松，便于仿真先跑通。
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return unknown_as_obstacle ?
      nav2_costmap_2d::LETHAL_OBSTACLE :
      nav2_costmap_2d::FREE_SPACE;
  }
  return cost;
}

unsigned char costAtPoint(
  const nav2_costmap_2d::Costmap2D * costmap,
  const DWATrajectoryPoint & point,
  bool unknown_as_obstacle)
{
  return costAtWorld(costmap, point.x, point.y, unknown_as_obstacle);
}

DWATrajectoryPoint forwardPoint(const DWATrajectoryPoint & point, double distance)
{
  DWATrajectoryPoint shifted = point;
  shifted.x += distance * std::cos(point.theta);
  shifted.y += distance * std::sin(point.theta);
  return shifted;
}

std::size_t nearestPathIndex(
  const DWATrajectoryPoint & point,
  const nav_msgs::msg::Path & path)
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double distance = distanceToPose(point, path.poses[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

double pathYawAtIndex(const nav_msgs::msg::Path & path, std::size_t index)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }
  const std::size_t next = std::min(index + 1, path.poses.size() - 1);
  const std::size_t prev = next == index ? index - 1 : index;
  const auto & from = path.poses[prev].pose.position;
  const auto & to = path.poses[next].pose.position;
  return std::atan2(to.y - from.y, to.x - from.x);
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

double footprintScaleFactor(const DWATrajectory & trajectory, const DWAControllerConfig & config)
{
  const double speed = std::fabs(trajectory.command.v);
  if (config.max_footprint_scaling_factor <= 0.0 ||
      speed <= config.footprint_scaling_speed)
  {
    return 1.0;
  }

  const double denominator = std::max(
    1.0e-6, config.max_linear_velocity - config.footprint_scaling_speed);
  const double ratio = std::clamp((speed - config.footprint_scaling_speed) / denominator, 0.0, 1.0);
  return 1.0 + ratio * config.max_footprint_scaling_factor;
}

std::vector<geometry_msgs::msg::Point> makeFallbackFootprint(double radius)
{
  std::vector<geometry_msgs::msg::Point> footprint;
  const double safe_radius = std::max(radius, 1.0e-3);
  constexpr int kSegments = 12;
  footprint.reserve(kSegments);
  for (int i = 0; i < kSegments; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kSegments);
    geometry_msgs::msg::Point point;
    point.x = safe_radius * std::cos(angle);
    point.y = safe_radius * std::sin(angle);
    point.z = 0.0;
    footprint.push_back(point);
  }
  return footprint;
}

std::vector<geometry_msgs::msg::Point> scaledFootprint(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const DWAControllerConfig & config,
  double scale)
{
  std::vector<geometry_msgs::msg::Point> scaled =
    footprint.empty() ? makeFallbackFootprint(config.robot_radius) : footprint;
  for (auto & point : scaled) {
    point.x *= scale;
    point.y *= scale;
  }
  return scaled;
}

geometry_msgs::msg::Point transformFootprintPoint(
  const geometry_msgs::msg::Point & local_point,
  const DWATrajectoryPoint & pose)
{
  geometry_msgs::msg::Point world_point;
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);
  world_point.x = pose.x + local_point.x * cos_theta - local_point.y * sin_theta;
  world_point.y = pose.y + local_point.x * sin_theta + local_point.y * cos_theta;
  world_point.z = 0.0;
  return world_point;
}

unsigned char maxCostAlongLine(
  const nav2_costmap_2d::Costmap2D * costmap,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  bool unknown_as_obstacle)
{
  const double distance = std::hypot(end.x - start.x, end.y - start.y);
  const double step = costmap ? std::max(0.01, costmap->getResolution() * 0.5) : 0.02;
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));

  unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const double x = start.x + ratio * (end.x - start.x);
    const double y = start.y + ratio * (end.y - start.y);
    max_cost = std::max(max_cost, costAtWorld(costmap, x, y, unknown_as_obstacle));
  }
  return max_cost;
}

unsigned char footprintCostAtPose(
  const nav2_costmap_2d::Costmap2D * costmap,
  const DWATrajectoryPoint & pose,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  bool unknown_as_obstacle)
{
  if (!costmap) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  if (footprint.size() < 3) {
    return costAtPoint(costmap, pose, unknown_as_obstacle);
  }

  unsigned char max_cost = costAtPoint(costmap, pose, unknown_as_obstacle);
  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const auto start = transformFootprintPoint(footprint[i], pose);
    const auto end = transformFootprintPoint(footprint[(i + 1) % footprint.size()], pose);
    max_cost = std::max(max_cost, maxCostAlongLine(costmap, start, end, unknown_as_obstacle));
  }
  return max_cost;
}

DWATrajectoryPoint motionStep(
  const DWATrajectoryPoint & state,
  const DWACommand & command,
  double dt)
{
  DWATrajectoryPoint next = state;
  next.x += command.v * std::cos(state.theta) * dt;
  next.y += command.v * std::sin(state.theta) * dt;
  next.theta = normalizeAngle(state.theta + command.w * dt);
  next.t = state.t + dt;
  return next;
}

double stopTimeHorizon(const DWATrajectory & trajectory, const DWAControllerConfig & config)
{
  const double linear_deceleration =
    std::max(1.0e-6, config.max_linear_velocity_increment * config.control_frequency);
  const double angular_deceleration =
    std::max(1.0e-6, config.max_angular_velocity_increment * config.control_frequency);
  const double linear_stop_time = std::fabs(trajectory.command.v) / linear_deceleration;
  const double angular_stop_time = std::fabs(trajectory.command.w) / angular_deceleration;
  return std::max(linear_stop_time, angular_stop_time) + std::max(0.0, config.stop_time_buffer);
}

unsigned char maxTrajectoryFootprintCost(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const DWAControllerConfig & config)
{
  const auto scaled_footprint = scaledFootprint(
    footprint, config, footprintScaleFactor(trajectory, config));
  unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
  for (const auto & point : trajectory.points) {
    max_cost = std::max(
      max_cost,
      footprintCostAtPose(costmap, point, scaled_footprint, config.unknown_as_obstacle));
  }

  if (!trajectory.points.empty()) {
    const double horizon = stopTimeHorizon(trajectory, config);
    const double dt = std::max(1.0e-3, config.sim_time_step);
    DWATrajectoryPoint state = trajectory.points.back();
    while (state.t + dt <= horizon) {
      state = motionStep(state, trajectory.command, dt);
      max_cost = std::max(
        max_cost,
        footprintCostAtPose(costmap, state, scaled_footprint, config.unknown_as_obstacle));
    }
  }

  return max_cost;
}

}  // namespace

bool checkCollision(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const DWAControllerConfig & config)
{
  // 步骤 1：使用 footprint 多边形检查轨迹，并按 stop_time_buffer 延长安全检查时间。
  // 只有真正致命障碍才作为硬碰撞；膨胀区保留给 obstacle cost 做软惩罚。
  if (!costmap || trajectory.points.empty()) {
    return false;
  }

  return maxTrajectoryFootprintCost(trajectory, costmap, footprint, config) >=
         nav2_costmap_2d::LETHAL_OBSTACLE;
}

double scoreObstacle(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const DWAControllerConfig & config)
{
  if (!costmap || trajectory.points.empty()) {
    return 0.0;
  }

  const unsigned char max_cost = maxTrajectoryFootprintCost(trajectory, costmap, footprint, config);
  return static_cast<double>(max_cost) /
         static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

double scorePath(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path)
{
  // 最小版 DWA 只用轨迹终点评价路径贴合，成本低，行为也容易解释。
  const auto * end = terminalPoint(trajectory);
  if (!end || path.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  double min_distance = std::numeric_limits<double>::max();
  for (const auto & pose : path.poses) {
    min_distance = std::min(min_distance, distanceToPose(*end, pose));
  }
  return min_distance;
}

double scoreGoal(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path)
{
  const auto * end = terminalPoint(trajectory);
  if (!end || path.poses.empty()) {
    return std::numeric_limits<double>::max();
  }
  return distanceToPose(*end, path.poses.back());
}

double scoreAlignment(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path,
  const DWAControllerConfig & config)
{
  // 步骤 2：用轨迹终点的“前鼻子”贴近路径，避免中心点贴路但车头乱摆。
  const auto * end = terminalPoint(trajectory);
  if (!end || path.poses.empty() || config.forward_point_distance <= 0.0) {
    return 0.0;
  }

  const double goal_distance = distanceToPose(*end, path.poses.back());
  const double disable_distance =
    config.forward_point_distance * std::sqrt(std::max(0.0, config.alignment_goal_distance_scale));
  if (goal_distance <= disable_distance) {
    return 0.0;
  }

  const auto nose = forwardPoint(*end, config.forward_point_distance);
  const std::size_t nearest_index = nearestPathIndex(nose, path);
  const double nose_path_distance = distanceToPose(nose, path.poses[nearest_index]);
  const double path_yaw = pathYawAtIndex(path, nearest_index);
  const double heading_error = std::fabs(normalizeAngle(end->theta - path_yaw));
  return nose_path_distance + 0.25 * heading_error;
}

double scoreGoalFront(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path,
  const DWAControllerConfig & config)
{
  // 步骤 3：让前鼻子朝局部目标推进，而不是只让机器人中心靠近目标。
  const auto * end = terminalPoint(trajectory);
  if (!end || trajectory.points.empty() || path.poses.empty() ||
      config.forward_point_distance <= 0.0)
  {
    return 0.0;
  }

  const auto & goal = path.poses.back().pose.position;
  const auto & start = trajectory.points.front();
  const double angle_to_goal = std::atan2(goal.y - start.y, goal.x - start.x);
  DWATrajectoryPoint shifted_goal;
  shifted_goal.x = goal.x + config.forward_point_distance * std::cos(angle_to_goal);
  shifted_goal.y = goal.y + config.forward_point_distance * std::sin(angle_to_goal);

  const auto nose = forwardPoint(*end, config.forward_point_distance);
  return distanceToPoint(nose, shifted_goal);
}

double scoreVelocity(
  const DWATrajectory & trajectory,
  double max_linear_velocity)
{
  // 分数越小越好：速度越接近最大速度，velocity_score 越小。
  if (max_linear_velocity <= 1.0e-6) {
    return 0.0;
  }
  const double normalized_v = std::clamp(
    trajectory.command.v / max_linear_velocity, 0.0, 1.0);
  return 1.0 - normalized_v;
}

double scoreTwirling(
  const DWATrajectory & trajectory,
  double max_angular_velocity)
{
  // 分数越小越好：角速度越大，twirling_score 越大。
  if (max_angular_velocity <= 1.0e-6) {
    return 0.0;
  }
  return std::clamp(
    std::fabs(trajectory.command.w) / max_angular_velocity, 0.0, 1.0);
}

double evaluateTrajectory(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const DWAControllerConfig & config)
{
  // 碰撞是硬失败，不进入加权评分。
  if (checkCollision(trajectory, costmap, footprint, config)) {
    return std::numeric_limits<double>::infinity();
  }

  const double path_score = scorePath(trajectory, path);
  const double goal_score = scoreGoal(trajectory, path);
  const double obstacle_score = scoreObstacle(trajectory, costmap, footprint, config);
  const double alignment_score = scoreAlignment(trajectory, path, config);
  const double goal_front_score = scoreGoalFront(trajectory, path, config);
  const double velocity_score = scoreVelocity(trajectory, config.max_linear_velocity);
  const double twirling_score = scoreTwirling(trajectory, config.max_angular_velocity);

  return config.path_distance_bias * path_score +
         config.goal_distance_bias * goal_score +
         config.obstacle_distance_bias * obstacle_score +
         config.alignment_bias * alignment_score +
         config.goal_front_bias * goal_front_score +
         config.velocity_bias * velocity_score +
         config.twirling_bias * twirling_score;
}

}  // namespace rmp::controller::dwa_critic
