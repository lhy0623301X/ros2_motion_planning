/**
 * @file rrt_star_planner.cpp
 * @brief RRT* 采样规划器实现。
 */
#include "sample_planner/rrt_star_planner.h"

#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

namespace {

bool isInsideMap(const RRTPathPlanner::Node & node, int size_x, int size_y)
{
  return node.x >= 0 && node.y >= 0 && node.x < size_x && node.y < size_y;
}

}  // namespace

RRTStarPathPlanner::RRTStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: RRTPathPlanner(std::move(costmap_ros))
{
}

bool RRTStarPathPlanner::plan(
  const Point3d & start, const Point3d & goal,
  Points3d * path, Points3d * expand)
{
  // 步骤 1：输入预处理，将世界坐标转换为 costmap 栅格坐标。
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheck(start.x, start.y, m_start_x, m_start_y) ||
      !validityCheck(goal.x, goal.y, m_goal_x, m_goal_y))
  {
    return false;
  }

  path->clear();
  expand->clear();
  sample_list_.clear();

  // 步骤 2：初始化采样树根节点和目标节点。
  start_ = Node(
    static_cast<int>(m_start_x), static_cast<int>(m_start_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_start_x), static_cast<int>(m_start_y)), 0);
  start_.parent_id = start_.id;
  goal_ = Node(
    static_cast<int>(m_goal_x), static_cast<int>(m_goal_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_goal_x), static_cast<int>(m_goal_y)), 0);
  sample_list_.insert({start_.id, start_});
  expand->push_back({m_start_x, m_start_y, 0.0});

  // 步骤 3：持续采样和重连，RRT* 不在第一次命中目标时立即停止。
  int best_parent = -1;
  double best_cost = std::numeric_limits<double>::max();
  for (int iteration = 0; iteration < sample_cfg_.sample_points; ++iteration) {
    const Node sample_node = generateRandomNode();
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

    // 步骤 4：扩展新节点，并在优化半径内选择更优父节点、重连邻域节点。
    const Node new_node = findNearestAndRewire(sample_list_, sample_node);
    if (new_node.id == -1 || sample_list_.find(new_node.id) != sample_list_.end()) {
      continue;
    }
    sample_list_.insert({new_node.id, new_node});
    expand->push_back({
      static_cast<double>(new_node.x),
      static_cast<double>(new_node.y),
      static_cast<double>(new_node.parent_id)});

    // 步骤 5：若新节点可直连目标，则只更新当前最优目标父节点。
    const double goal_dist = std::hypot(new_node.x - goal_.x, new_node.y - goal_.y);
    if (goal_dist <= sample_cfg_.sample_max_distance &&
        !isLineCollision(new_node.x, new_node.y, goal_.x, goal_.y))
    {
      const double cost = new_node.g + goal_dist;
      if (cost < best_cost) {
        best_cost = cost;
        best_parent = new_node.id;
      }
    }
  }

  // 步骤 6：采样结束后，如果存在目标连接，则回溯当前最优路径。
  fillSampleVisualization(*expand);
  if (best_parent == -1) {
    return false;
  }

  goal_.g = best_cost;
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

RRTPathPlanner::Node RRTStarPathPlanner::findNearestAndRewire(
  std::unordered_map<int, Node> & tree, const Node & sample)
{
  // 步骤 A：先按普通 RRT 找到最近节点并做步长截断。
  Node nearest_node;
  Node new_node = sample;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & [node_id, node] : tree) {
    (void)node_id;
    const double dist = std::hypot(node.x - sample.x, node.y - sample.y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_node = node;
    }
  }

  if (min_dist > sample_cfg_.sample_max_distance) {
    const double theta = std::atan2(sample.y - nearest_node.y, sample.x - nearest_node.x);
    new_node.x = nearest_node.x + static_cast<int>(sample_cfg_.sample_max_distance * std::cos(theta));
    new_node.y = nearest_node.y + static_cast<int>(sample_cfg_.sample_max_distance * std::sin(theta));
    if (!isInsideMap(new_node, getSizeInCellsX(), getSizeInCellsY())) {
      new_node.id = -1;
      return new_node;
    }
    new_node.id = grid2Index(new_node.x, new_node.y);
    min_dist = sample_cfg_.sample_max_distance;
  }

  new_node.parent_id = nearest_node.id;
  new_node.g = nearest_node.g + min_dist;
  if (new_node.id == nearest_node.id ||
      isLineCollision(nearest_node.x, nearest_node.y, new_node.x, new_node.y))
  {
    new_node.id = -1;
    return new_node;
  }

  // 步骤 B：在优化半径内寻找更低代价父节点。
  for (const auto & [node_id, node] : tree) {
    (void)node_id;
    const double dist = std::hypot(node.x - new_node.x, node.y - new_node.y);
    const double candidate_cost = node.g + dist;
    if (dist < sample_cfg_.optimization_radius &&
        candidate_cost < new_node.g &&
        !isLineCollision(node.x, node.y, new_node.x, new_node.y))
    {
      new_node.parent_id = node.id;
      new_node.g = candidate_cost;
    }
  }

  // 步骤 C：用新节点尝试优化邻域内已有节点。
  for (auto & [node_id, node] : tree) {
    (void)node_id;
    if (node.id == start_.id) {
      continue;
    }
    const double dist = std::hypot(node.x - new_node.x, node.y - new_node.y);
    const double candidate_cost = new_node.g + dist;
    if (dist < sample_cfg_.optimization_radius &&
        candidate_cost < node.g &&
        !isLineCollision(node.x, node.y, new_node.x, new_node.y))
    {
      node.parent_id = new_node.id;
      node.g = candidate_cost;
    }
  }

  return new_node;
}

}  // namespace rmp::path_planner
