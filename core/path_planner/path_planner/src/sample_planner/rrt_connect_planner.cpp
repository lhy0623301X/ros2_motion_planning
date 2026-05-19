/**
 * @file rrt_connect_planner.cpp
 * @brief RRT-Connect 双向采样规划器实现。
 */
#include "sample_planner/rrt_connect_planner.h"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

namespace {

bool isInsideMap(const RRTPathPlanner::Node & node, int size_x, int size_y)
{
  return node.x >= 0 && node.y >= 0 && node.x < size_x && node.y < size_y;
}

}  // namespace

RRTConnectPathPlanner::RRTConnectPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: RRTPathPlanner(std::move(costmap_ros))
{
}

bool RRTConnectPathPlanner::plan(
  const Point3d & start, const Point3d & goal,
  Points3d * path, Points3d * expand)
{
  // 步骤 1：输入预处理，转换为栅格坐标。
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheck(start.x, start.y, m_start_x, m_start_y) ||
      !validityCheck(goal.x, goal.y, m_goal_x, m_goal_y))
  {
    return false;
  }

  path->clear();
  expand->clear();
  std::unordered_map<int, Node> start_tree;
  std::unordered_map<int, Node> goal_tree;

  start_ = Node(
    static_cast<int>(m_start_x), static_cast<int>(m_start_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_start_x), static_cast<int>(m_start_y)), 0);
  start_.parent_id = start_.id;
  goal_ = Node(
    static_cast<int>(m_goal_x), static_cast<int>(m_goal_y),
    0.0, 0.0, grid2Index(static_cast<int>(m_goal_x), static_cast<int>(m_goal_y)), 0);
  goal_.parent_id = goal_.id;
  start_tree.insert({start_.id, start_});
  goal_tree.insert({goal_.id, goal_});
  expand->push_back({m_start_x, m_start_y, 0.0});
  expand->push_back({m_goal_x, m_goal_y, 0.0});

  // 步骤 2：每轮先扩展起点树，再让目标树朝新节点贪婪连接。
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
    if (start_tree.find(sample_node.id) != start_tree.end()) {
      continue;
    }

    Node start_new = findNearestAndSteer(start_tree, sample_node);
    if (start_new.id == -1 || start_tree.find(start_new.id) != start_tree.end()) {
      continue;
    }
    start_tree.insert({start_new.id, start_new});
    expand->push_back({
      static_cast<double>(start_new.x),
      static_cast<double>(start_new.y),
      static_cast<double>(start_new.parent_id)});

    // 步骤 3：目标树朝起点树的新节点连续推进。
    Node goal_new = findNearestAndSteer(goal_tree, start_new);
    while (goal_new.id != -1 && goal_tree.find(goal_new.id) == goal_tree.end()) {
      goal_tree.insert({goal_new.id, goal_new});
      expand->push_back({
        static_cast<double>(goal_new.x),
        static_cast<double>(goal_new.y),
        static_cast<double>(goal_new.parent_id)});

      // 步骤 4：两树已经到达同一栅格，回溯并拼接完整路径。
      if (goal_new == start_new) {
        const auto start_path = traceTreePath(start_tree, start_, start_new);
        const auto goal_path = traceTreePath(goal_tree, goal_, goal_new);
        if (start_path.empty() || goal_path.empty()) {
          fillBiTreeVisualization(start_tree, goal_tree, *expand);
          return false;
        }

        for (const auto & node : start_path) {
          double wx, wy;
          costmap_->mapToWorld(
            static_cast<unsigned int>(node.x),
            static_cast<unsigned int>(node.y), wx, wy);
          path->push_back({wx, wy, 0.0});
        }
        for (auto iter = goal_path.rbegin() + 1; iter != goal_path.rend(); ++iter) {
          double wx, wy;
          costmap_->mapToWorld(
            static_cast<unsigned int>(iter->x),
            static_cast<unsigned int>(iter->y), wx, wy);
          path->push_back({wx, wy, 0.0});
        }

        fillBiTreeVisualization(start_tree, goal_tree, *expand);
        return true;
      }

      const Node next = findNearestAndSteer(goal_tree, start_new);
      if (next.id == goal_new.id) {
        break;
      }
      goal_new = next;
    }
  }

  fillBiTreeVisualization(start_tree, goal_tree, *expand);
  return false;
}

std::vector<RRTPathPlanner::Node> RRTConnectPathPlanner::traceTreePath(
  const std::unordered_map<int, Node> & tree,
  const Node & root,
  const Node & leaf) const
{
  std::vector<Node> reversed_path;
  auto current = tree.find(leaf.id);
  while (current != tree.end()) {
    reversed_path.push_back(current->second);
    if (current->second == root) {
      break;
    }
    current = tree.find(current->second.parent_id);
  }
  if (reversed_path.empty() || !(reversed_path.back() == root)) {
    return {};
  }
  std::reverse(reversed_path.begin(), reversed_path.end());
  return reversed_path;
}

void RRTConnectPathPlanner::fillBiTreeVisualization(
  const std::unordered_map<int, Node> & start_tree,
  const std::unordered_map<int, Node> & goal_tree,
  const Points3d & expand)
{
  sample_list_.clear();
  sample_list_.insert(start_tree.begin(), start_tree.end());
  sample_list_.insert(goal_tree.begin(), goal_tree.end());
  fillSampleVisualization(expand);
}

}  // namespace rmp::path_planner
