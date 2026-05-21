/**
 * @file dwa_critic.cpp
 * @brief Basic trajectory scoring utilities for the minimal DWA controller.
 */
#include "dwa_controller/dwa_critic.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::controller::dwa_critic {

namespace {

double distanceToPose(
  const DWATrajectoryPoint & point,
  const geometry_msgs::msg::PoseStamped & pose)
{
  return std::hypot(point.x - pose.pose.position.x, point.y - pose.pose.position.y);
}

const DWATrajectoryPoint * terminalPoint(const DWATrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return nullptr;
  }
  return &trajectory.points.back();
}

unsigned char costAtPoint(
  const nav2_costmap_2d::Costmap2D * costmap,
  const DWATrajectoryPoint & point,
  bool unknown_as_obstacle)
{
  // 世界坐标超出 costmap 时直接视为不可行，避免轨迹跑出局部地图。
  if (!costmap) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(point.x, point.y, mx, my)) {
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

}  // namespace

bool checkCollision(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool unknown_as_obstacle)
{
  // 第一版使用轨迹点中心检查；后续如果需要，可升级为 footprint sweep。
  if (!costmap || trajectory.points.empty()) {
    return false;
  }

  for (const auto & point : trajectory.points) {
    const auto cost = costAtPoint(costmap, point, unknown_as_obstacle);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }
  return false;
}

double scoreObstacle(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool unknown_as_obstacle)
{
  if (!costmap || trajectory.points.empty()) {
    return 0.0;
  }

  unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
  for (const auto & point : trajectory.points) {
    max_cost = std::max(max_cost, costAtPoint(costmap, point, unknown_as_obstacle));
  }
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
  const DWAControllerConfig & config)
{
  // 碰撞是硬失败，不进入加权评分。
  if (checkCollision(trajectory, costmap, config.unknown_as_obstacle)) {
    return std::numeric_limits<double>::infinity();
  }

  const double path_score = scorePath(trajectory, path);
  const double goal_score = scoreGoal(trajectory, path);
  const double obstacle_score = scoreObstacle(trajectory, costmap, config.unknown_as_obstacle);
  const double velocity_score = scoreVelocity(trajectory, config.max_linear_velocity);
  const double twirling_score = scoreTwirling(trajectory, config.max_angular_velocity);

  return config.path_distance_bias * path_score +
         config.goal_distance_bias * goal_score +
         config.obstacle_distance_bias * obstacle_score +
         config.velocity_bias * velocity_score +
         config.twirling_bias * twirling_score;
}

}  // namespace rmp::controller::dwa_critic
