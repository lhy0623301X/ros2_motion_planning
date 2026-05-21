/**
 * @file dwa_critic.h
 * @brief Basic trajectory scoring utilities for the minimal DWA controller.
 */
#ifndef RMP_CONTROLLER_DWA_CRITIC_H_
#define RMP_CONTROLLER_DWA_CRITIC_H_

#include "dwa_controller/dwa_types.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace rmp::controller::dwa_critic {

// 硬约束：轨迹任一点碰到致命/膨胀障碍则判为非法。
bool checkCollision(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool unknown_as_obstacle);

// 软约束：轨迹经过的最高 costmap 代价，归一化后作为障碍物评分。
double scoreObstacle(
  const DWATrajectory & trajectory,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool unknown_as_obstacle);

// 软约束：轨迹终点到局部路径最近点的距离。
double scorePath(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path);

// 软约束：轨迹终点到当前局部目标的距离。
double scoreGoal(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path);

// 软约束：鼓励选择更大的前向速度，避免低速候选轨迹长期占优。
double scoreVelocity(
  const DWATrajectory & trajectory,
  double max_linear_velocity);

// 软约束：惩罚过大的角速度，避免无必要的原地转圈。
double scoreTwirling(
  const DWATrajectory & trajectory,
  double max_angular_velocity);

// 总评分入口：先做碰撞剔除，再组合 path / goal / obstacle 三项代价。
double evaluateTrajectory(
  const DWATrajectory & trajectory,
  const nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const DWAControllerConfig & config);

}  // namespace rmp::controller::dwa_critic

#endif  // RMP_CONTROLLER_DWA_CRITIC_H_
