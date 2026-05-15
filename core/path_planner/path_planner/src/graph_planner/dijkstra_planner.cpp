/**
 * @file dijkstra_planner.cpp
 * @brief Minimal Dijkstra global planner migrated first for phased validation.
 */
#include "graph_planner/dijkstra_planner.h"

#include <cmath>
#include <queue>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

const std::vector<DijkstraPathPlanner::Node> DijkstraPathPlanner::motions_ = {
  {0, 1, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {-1, 0, 1.0},
  {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
  {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)},
};

DijkstraPathPlanner::DijkstraPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros))
{
}

double DijkstraPathPlanner::calcObstacleSigmoidCost(unsigned char cell_cost) const
{
  // 将代价值归一化到 [0, 1]。
  // inflation layer 会让离障碍物越近的栅格拥有越高的 cost，
  // 因此这里可以把 cost 近似视为“离障碍物近”的程度。
  const double normalized_cost =
    static_cast<double>(cell_cost) / static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const double clamped_cost = std::max(0.0, std::min(1.0, normalized_cost));

  // 使用 Sigmoid 将障碍物邻近程度映射成平滑惩罚：
  // - 离障碍物远时，惩罚接近 0
  // - 接近 sigmoid_center 后，惩罚快速上升
  // - 离障碍物越近，惩罚越接近 obstacle_cost_weight
  const double sigmoid =
    1.0 / (1.0 + std::exp(-config().obstacle_sigmoid_alpha *
    (clamped_cost - config().obstacle_sigmoid_center)));

  return config().obstacle_cost_weight * sigmoid;
}

void DijkstraPathPlanner::fillSearchedPointsDebugInfo(const Points3d & expand)
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

bool DijkstraPathPlanner::plan(
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
  // open list 是按累计代价排序的优先队列；
  // closed list 用于保存已经完成扩展的节点，便于后续回溯路径。
  std::priority_queue<Node, std::vector<Node>, Node::CompareCost> open_list;
  std::unordered_map<int, Node> closed_list;
  open_list.push(start_node);

  // 步骤 4：进入主搜索循环。
  // 每次取出当前总代价最小的节点进行扩展，直到找到目标或 open list 为空。
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
    // Dijkstra 只使用累计路径代价 g，不引入启发式代价 h。
    for (const auto & motion : motions_) {
      auto next = current + motion;
      next.id = grid2Index(next.x, next.y);

      // 忽略已经完成扩展的节点。
      if (closed_list.find(next.id) != closed_list.end()) {
        continue;
      }

      next.parent_id = current.id;

      // 丢弃落在 costmap 边界之外的邻接节点。
      if (next.id < 0 || next.id >= getMapSize()) {
        continue;
      }

      // 丢弃障碍物栅格。
      // 第二个条件保留了原 ROS1 版本的行为：
      // 如果当前节点已经处于膨胀区域内，则允许一定程度上的继续扩展。
      const auto * char_map = getCostMap()->getCharMap();
      if (char_map[next.id] >= nav2_costmap_2d::LETHAL_OBSTACLE * config().obstacle_inflation_factor &&
          char_map[next.id] >= char_map[current.id])
      {
        continue;
      }

      // 步骤 6.1：在基础运动代价之外，引入“离障碍物越近代价越高”的惩罚项。
      // 这里不直接计算几何距离，而是利用 inflation layer 生成的 cost 近似反映
      // 障碍物邻近程度，再通过 Sigmoid 函数将其平滑映射为附加代价。
      const double obstacle_penalty = calcObstacleSigmoidCost(char_map[next.id]);
      next.g = current.g + motion.g + obstacle_penalty;

      // 合法邻接节点压入 open list，等待后续继续扩展。
      open_list.push(next);
    }
  }

  // 步骤 7：如果 open list 已经耗尽，说明当前地图上不存在可行路径。
  fillSearchedPointsDebugInfo(*expand);
  return false;
}

}  // namespace rmp::path_planner
