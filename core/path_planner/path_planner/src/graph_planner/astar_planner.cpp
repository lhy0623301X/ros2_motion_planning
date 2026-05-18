/**
 * @file astar_planner.cpp
 * @brief A* global planner — uses both accumulated cost g and heuristic h.
 *
 * Compared to Dijkstra (h = 0), A* introduces an admissible heuristic
 * (Euclidean distance to the goal) so that the search is guided toward the
 * goal, typically expanding fewer nodes while still guaranteeing optimality.
 */
#include "graph_planner/astar_planner.h"

#include "common/util/log.h"

#include <cmath>
#include <queue>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

const std::vector<AStarPathPlanner::Node> AStarPathPlanner::motions_ = {
  {0, 1, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {-1, 0, 1.0},
  {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
  {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)},
};

AStarPathPlanner::AStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros))
{
}

double AStarPathPlanner::calcObstacleSigmoidCost(unsigned char cell_cost) const
{
  const double normalized_cost =
    static_cast<double>(cell_cost) / static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const double clamped_cost = std::max(0.0, std::min(1.0, normalized_cost));

  const double sigmoid =
    1.0 / (1.0 + std::exp(-config().obstacle_sigmoid_alpha *
    (clamped_cost - config().obstacle_sigmoid_center)));

  return config().obstacle_cost_weight * sigmoid;
}

void AStarPathPlanner::fillSearchedPointsDebugInfo(const Points3d & expand)
{
  auto & searched_points = mutableDebugInfo().searched_points;
  searched_points.clear();
  searched_points.reserve(expand.size());

  for (const auto & point : expand) {
    double wx;
    double wy;
    map2World(point.x, point.y, wx, wy);
    searched_points.push_back({wx, wy, point.theta});
  }
}

bool AStarPathPlanner::plan(
  const Point3d & start,
  const Point3d & goal,
  Points3d * path,
  Points3d * expand)
{
  // 步骤 1：将起点和终点从世界坐标转换到代价地图栅格坐标。
  // 如果任一点落在当前 costmap 范围外，则直接返回规划失败。
  double start_mx;
  double start_my;
  double goal_mx;
  double goal_my;
  if (!validityCheck(start.x, start.y, start_mx, start_my) ||
      !validityCheck(goal.x, goal.y, goal_mx, goal_my))
  {
    return false;
  }

  if (config().outline_map) {
    outlineMap();
  }

  // 步骤 2：在栅格空间中构造起点和终点搜索节点，
  // 并初始化它们在线性化地图中的索引 id，供图搜索使用。
  Node start_node(static_cast<int>(start_mx), static_cast<int>(start_my));
  Node goal_node(static_cast<int>(goal_mx), static_cast<int>(goal_my));
  start_node.id = grid2Index(start_node.x, start_node.y);
  goal_node.id = grid2Index(goal_node.x, goal_node.y);

  path->clear();
  expand->clear();

  // 步骤 3：初始化 open list 和 closed list。
  // open list 按 f = g + h 排序，其中
  //   g = 从起点到当前节点的累计路径代价
  //   h = 从当前节点到终点的欧几里得距离（启发式代价）
  std::priority_queue<Node, std::vector<Node>, Node::CompareCost> open_list;
  std::unordered_map<int, Node> closed_list;
  open_list.push(start_node);

  // 步骤 4：进入主搜索循环。
  // 每次从 open list 中取出 f 值最小的节点进行扩展。
  while (!open_list.empty()) {
    const auto current = open_list.top();
    open_list.pop();

    // 跳过已经在 closed list 中处理过的旧节点。
    if (closed_list.find(current.id) != closed_list.end()) {
      continue;
    }

    closed_list.insert(std::make_pair(current.id, current));
    expand->push_back(Point3d{static_cast<double>(current.x), static_cast<double>(current.y), 0.0});

    // 步骤 5：如果当前节点已经到达目标点，
    // 就沿父节点链回溯出整条路径，再转换回世界坐标后返回成功。
    if (current == goal_node) {
      fillSearchedPointsDebugInfo(*expand);
      const auto backtrace = convertClosedListToPath(closed_list, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        double wx;
        double wy;
        map2World(iter->x, iter->y, wx, wy);
        path->push_back(Point3d{wx, wy, 0.0});
      }
      return true;
    }

    // 步骤 6：扩展当前节点周围的 8 邻域。
    // 与 Dijkstra 的关键区别：A* 为每个邻接节点计算启发式 h，
    // 使得搜索能优先朝目标方向扩展，减少不必要的探索。
    for (const auto & motion : motions_) {
      auto next = current + motion;
      next.parent_id = current.id;

      if (next.x < 0 || next.y < 0 ||
        next.x >= getSizeInCellsX() || next.y >= getSizeInCellsY())
      {
        continue;
      }

      next.id = grid2Index(next.x, next.y);

      // 忽略已经完成扩展的节点。
      if (closed_list.find(next.id) != closed_list.end()) {
        continue;
      }

      // 丢弃障碍物栅格。
      const auto * char_map = getCostMap()->getCharMap();
      if (char_map[next.id] >= nav2_costmap_2d::LETHAL_OBSTACLE * config().obstacle_inflation_factor &&
          char_map[next.id] >= char_map[current.id])
      {
        continue;
      }

      // 步骤 6.0：在 costmap 膨胀之外，再叠加一层规划安全边界，
      // 给车体尺寸和控制误差预留余量。
      if (!isNodeCollisionFree(next.x, next.y)) {
        continue;
      }

      // 步骤 6.1：计算累计路径代价 g = 父节点 g + 运动代价 + 障碍物 Sigmoid 惩罚。
      const double obstacle_penalty = calcObstacleSigmoidCost(char_map[next.id]);
      next.g = current.g + motion.g + obstacle_penalty;

      // 步骤 6.2：计算启发式代价 h = 当前节点到目标的欧几里得距离。
      // 欧几里得距离是可接受的启发式（不会高估真实代价），
      // 因此 A* 在找到路径时能保证其最优性。
      next.h = std::hypot(
        next.x - goal_node.x,
        next.y - goal_node.y);

      open_list.push(next);
    }
  }

  // 步骤 7：如果 open list 已经耗尽，说明当前地图上不存在可行路径。
  fillSearchedPointsDebugInfo(*expand);
  return false;
}

}  // namespace rmp::path_planner
