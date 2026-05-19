/**
 * @file informed_rrt_star_planner.cpp
 * @brief Informed RRT* 采样规划器实现。
 */
#include "sample_planner/informed_rrt_star_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

namespace {

bool isInsideMap(const RRTPathPlanner::Node & node, int size_x, int size_y)
{
  return node.x >= 0 && node.y >= 0 && node.x < size_x && node.y < size_y;
}

}  // namespace

InformedRRTStarPathPlanner::InformedRRTStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: RRTStarPathPlanner(std::move(costmap_ros))
{
}

bool InformedRRTStarPathPlanner::plan(
  const Point3d & start, const Point3d & goal,
  Points3d * path, Points3d * expand)
{
  // 步骤 1：输入预处理，并初始化椭圆采样所需的最短理论距离。
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheck(start.x, start.y, m_start_x, m_start_y) ||
      !validityCheck(goal.x, goal.y, m_goal_x, m_goal_y))
  {
    return false;
  }

  path->clear();
  expand->clear();
  sample_list_.clear();

  start_ = Node(
    static_cast<int>(m_start_x), static_cast<int>(m_start_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_start_x), static_cast<int>(m_start_y)), 0);
  start_.parent_id = start_.id;
  goal_ = Node(
    static_cast<int>(m_goal_x), static_cast<int>(m_goal_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_goal_x), static_cast<int>(m_goal_y)), 0);
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = std::hypot(start_.x - goal_.x, start_.y - goal_.y);

  sample_list_.insert({start_.id, start_});
  expand->push_back({m_start_x, m_start_y, 0.0});

  // 步骤 2：找到首条路径前全图采样；找到后进入椭圆区域内采样并持续优化。
  int best_parent = -1;
  for (int iteration = 0; iteration < sample_cfg_.sample_points; ++iteration) {
    const Node sample_node = generateInformedRandomNode();
    if (!isInsideMap(sample_node, getSizeInCellsX(), getSizeInCellsY())) {
      continue;
    }
    if (costmap_->getCharMap()[sample_node.id] >=
        nav2_costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor)
    {
      continue;
    }
    if (sample_list_.find(sample_node.id) != sample_list_.end()) {
      continue;
    }

    // 步骤 3：复用 RRT* 的邻域重连逻辑扩展新节点。
    const Node new_node = findNearestAndRewire(sample_list_, sample_node);
    if (new_node.id == -1 || sample_list_.find(new_node.id) != sample_list_.end()) {
      continue;
    }
    sample_list_.insert({new_node.id, new_node});
    expand->push_back({
      static_cast<double>(new_node.x),
      static_cast<double>(new_node.y),
      static_cast<double>(new_node.parent_id)});

    // 步骤 4：若可直连目标，则更新当前最优代价；后续采样会自动收缩到椭圆。
    const double goal_dist = std::hypot(new_node.x - goal_.x, new_node.y - goal_.y);
    if (goal_dist <= sample_cfg_.sample_max_distance &&
        !isLineCollision(new_node.x, new_node.y, goal_.x, goal_.y))
    {
      const double cost = new_node.g + goal_dist;
      if (cost < c_best_) {
        c_best_ = cost;
        best_parent = new_node.id;
      }
    }
  }

  // 步骤 5：回溯最优路径，若整个采样周期未找到可行解则失败返回。
  fillSampleVisualization(*expand);
  if (best_parent == -1) {
    return false;
  }

  goal_.g = c_best_;
  goal_.parent_id = best_parent;
  sample_list_.insert_or_assign(goal_.id, goal_);
  const auto backtrace = convertClosedListToPath<Node>(sample_list_, start_, goal_);
  if (backtrace.empty()) {
    return false;
  }

  for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(iter->x),
      static_cast<unsigned int>(iter->y), wx, wy);
    path->push_back({wx, wy, 0.0});
  }
  return true;
}

RRTPathPlanner::Node InformedRRTStarPathPlanner::generateInformedRandomNode()
{
  // 尚未有可行解时，退化为普通 RRT* 的全图随机/目标偏置采样。
  if (c_best_ == std::numeric_limits<double>::max()) {
    return generateRandomNode();
  }

  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int attempt = 0; attempt < 100; ++attempt) {
    double x = 0.0;
    double y = 0.0;
    do {
      x = unit(rng_);
      y = unit(rng_);
    } while (x * x + y * y > 1.0);

    const Node node = transformFromUnitBall(x, y);
    if (isInsideMap(node, getSizeInCellsX(), getSizeInCellsY())) {
      return node;
    }
  }

  return generateRandomNode();
}

RRTPathPlanner::Node InformedRRTStarPathPlanner::transformFromUnitBall(double x, double y) const
{
  // 将单位圆采样点缩放、旋转、平移到以 start/goal 为焦点的椭圆内。
  const double center_x = (start_.x + goal_.x) / 2.0;
  const double center_y = (start_.y + goal_.y) / 2.0;
  const double theta = std::atan2(goal_.y - start_.y, goal_.x - start_.x);
  const double a = c_best_ / 2.0;
  const double c = c_min_ / 2.0;
  const double b = std::sqrt(std::max(0.0, a * a - c * c));

  const int tx = static_cast<int>(a * std::cos(theta) * x - b * std::sin(theta) * y + center_x);
  const int ty = static_cast<int>(a * std::sin(theta) * x + b * std::cos(theta) * y + center_y);
  return Node(tx, ty, 0.0, 0.0, grid2Index(tx, ty), 0);
}

}  // namespace rmp::path_planner
