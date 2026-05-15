/**
 * @file hybrid_astar_planner.cpp
 * @brief Hybrid A* 路径规划器实现 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * =====================================================================
 * 算法总览
 * =====================================================================
 * Hybrid A* 将标准 A* 的离散 2D 网格搜索扩展到 3D 连续状态空间
 * (x, y, theta)。核心改进点：
 *
 * 1) 搜索空间：将朝向角量化为 N 个 bin，节点索引 = f(x, y, theta_bin)
 * 2) 邻居扩展：基于车辆运动学原语（直行/左转/右转），而非网格连通
 * 3) 双层启发式：
 *    - 障碍启发：忽略朝向，从目标反向 Dijkstra 构建 2D 代价场
 *    - 距离启发：Dubins 曲线长度（考虑最小转弯半径）
 *    取两者最大值作为 h(n)
 * 4) Analytic Expansion：当搜索前沿距目标足够近时，
 *    尝试用 Dubins 曲线直接连到目标，加速最后阶段收敛
 *
 * 迁移变更：
 *  - costmap_2d → nav2_costmap_2d（ROS2 Nav2）
 *  - protobuf 配置 → HybridAStarConfig 结构体 + ROS2 参数
 *  - Point3d 类型桥接：接口层使用 rmp::path_planner::Point3d（struct），
 *    内部使用 rmp::common::geometry::Point3d（模板类）用于曲线和碰撞检测
 *  - 接入 PlannerDebugInfo 可视化
 * =====================================================================
 */
#include "common/math/math_helper.h"
#include "graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"

using CPoint3d = rmp::common::geometry::Point3d;
using CPoints3d = rmp::common::geometry::Points3d;

namespace rmp::path_planner {

// ---- 静态成员初始化 ----
HybridAStarMotionTable HybridAStarPathPlanner::motion_table_;
std::vector<rmp::common::structure::Node<int>>
  HybridAStarPathPlanner::grid_motions_ = {
    {0, 1, 1.0},            {1, 0, 1.0},
    {0, -1, 1.0},           {-1, 0, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)},{-1, -1, std::sqrt(2.0)},
};

// ===================================================================
// 构造函数
// ===================================================================
HybridAStarPathPlanner::HybridAStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(costmap_ros)
{
}

/**
 * @brief 设置 Hybrid A* 专用参数并初始化运动原语表。
 *
 * 必须在第一次 plan() 调用之前由工厂调用。
 */
void HybridAStarPathPlanner::setHybridConfig(const HybridAStarConfig & cfg)
{
  hybrid_cfg_ = cfg;
  const double min_turn_px =
    hybrid_cfg_.minimum_turning_radius / costmap_->getResolution();
  motion_table_.initDubins(
    hybrid_cfg_.dim_3_size,
    min_turn_px,
    static_cast<int>(costmap_->getSizeInCellsX()),
    hybrid_cfg_.curve_sample_ratio,
    hybrid_cfg_.change_penalty,
    hybrid_cfg_.non_straight_penalty,
    hybrid_cfg_.reverse_penalty,
    hybrid_cfg_.retrospective_penalty);
}

// ===================================================================
// 3D 索引 ↔ 位姿 互转
// ===================================================================
CPoint3d HybridAStarPathPlanner::getPose(uint64_t index)
{
  return CPoint3d(
    static_cast<double>(
      (index / motion_table_.num_angle_quantization) % motion_table_.map_width),
    static_cast<double>(
      index / (static_cast<uint64_t>(motion_table_.num_angle_quantization) *
               motion_table_.map_width)),
    static_cast<double>(index % motion_table_.num_angle_quantization));
}

uint64_t HybridAStarPathPlanner::getIndex(const CPoint3d & pose)
{
  return static_cast<uint64_t>(pose.theta()) +
         static_cast<uint64_t>(pose.x()) *
           static_cast<uint64_t>(motion_table_.num_angle_quantization) +
         static_cast<uint64_t>(pose.y()) *
           static_cast<uint64_t>(motion_table_.map_width) *
           static_cast<uint64_t>(motion_table_.num_angle_quantization);
}

// ===================================================================
// plan() — 公共入口，处理坐标转换 + 路径复用 + 调用 createPath
// ===================================================================
bool HybridAStarPathPlanner::plan(
  const Point3d & start, const Point3d & goal,
  Points3d * path, Points3d * expand)
{
  // 步骤 1：将世界坐标转换为地图栅格坐标
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheck(start.x, start.y, m_start_x, m_start_y) ||
      !validityCheck(goal.x, goal.y, m_goal_x, m_goal_y))
  {
    return false;
  }
  path->clear();
  expand->clear();

  // 步骤 2：路径复用 — 如果目标不变且旧路径无碰撞，截取复用
  CPoint3d cgoal(goal.x, goal.y, goal.theta);
  if (!hybrid_last_path_.empty() && stored_goal_ == cgoal) {
    bool collision = false;
    CPoints3d::iterator closest_iter = hybrid_last_path_.begin();
    double min_dist = std::numeric_limits<double>::max();
    for (auto it = hybrid_last_path_.begin(); it != hybrid_last_path_.end(); ++it) {
      unsigned int mx, my;
      costmap_->worldToMap(it->x(), it->y(), mx, my);
      if (isCollision(CPoint3d(static_cast<double>(mx), static_cast<double>(my)))) {
        collision = true;
        break;
      }
      double dist = std::hypot(it->x() - start.x, it->y() - start.y);
      if (dist < min_dist) {
        min_dist = dist;
        closest_iter = it;
      }
    }
    if (!collision) {
      for (auto it = closest_iter; it != hybrid_last_path_.end(); ++it) {
        path->push_back({it->x(), it->y(), it->theta()});
      }
      hybrid_last_path_ = CPoints3d(closest_iter, hybrid_last_path_.end());
      return true;
    }
    hybrid_last_path_.clear();
  }

  // 步骤 3：量化起止点朝向，调用核心搜索
  stored_goal_ = cgoal;
  CPoints3d path_in_map;
  CPoint3d cstart_map(
    m_start_x, m_start_y,
    static_cast<double>(motion_table_.getOrientationBin(start.theta)));
  CPoint3d cgoal_map(
    m_goal_x, m_goal_y,
    static_cast<double>(motion_table_.getOrientationBin(goal.theta)));

  CPoints3d cexpand;
  if (createPath(cstart_map, cgoal_map, &path_in_map, &cexpand)) {
    // 步骤 4：将栅格路径转换回世界坐标（反序，因为 backtrace 是逆序的）
    for (auto it = path_in_map.rbegin(); it != path_in_map.rend(); ++it) {
      double wx, wy;
      costmap_->mapToWorld(
        static_cast<unsigned int>(it->x()),
        static_cast<unsigned int>(it->y()), wx, wy);
      path->push_back({wx, wy, it->theta()});
    }
    // 保存路径用于下次复用
    hybrid_last_path_.clear();
    for (const auto & pt : *path) {
      hybrid_last_path_.emplace_back(pt.x, pt.y, pt.theta);
    }

    // 步骤 5：填充可视化 — 将搜索过的节点转换为世界坐标
    auto & searched_pts = mutableDebugInfo().searched_points;
    searched_pts.clear();
    searched_pts.reserve(cexpand.size());
    for (const auto & ep : cexpand) {
      double wx, wy;
      costmap_->mapToWorld(
        static_cast<unsigned int>(ep.x()),
        static_cast<unsigned int>(ep.y()), wx, wy);
      searched_pts.push_back({wx, wy, ep.theta()});
    }
    return true;
  }

  return false;
}

// ===================================================================
// createPath() — Hybrid A* 核心搜索循环
// ===================================================================
bool HybridAStarPathPlanner::createPath(
  const CPoint3d & start, const CPoint3d & goal,
  CPoints3d * path, CPoints3d * expand)
{
  // 步骤 1：重置搜索图和优先队列
  clearGraph();
  clearQueue();
  best_heuristic_node_ = {std::numeric_limits<float>::max(), 0};
  auto start_node = addToGraph(getIndex(start));
  auto goal_node  = addToGraph(getIndex(goal));

  // 步骤 2：预计算障碍启发式 — 从目标反向 Dijkstra
  precomputeObstacleHeuristic(goal_node);

  // 步骤 3：将起点加入 Open 列表
  addToQueue(0.0, start_node);
  start_node->setAccumulatedCost(0.0);
  std::vector<NodeHybrid::NodePtr> neighbors;
  NodeHybrid::NodePtr neighbor = nullptr;

  // ===================== 主搜索循环 =====================
  int iterations = 0, approach_iterations = 0;
  while (iterations < hybrid_cfg_.max_iterations && !queue_.empty()) {
    // 步骤 4：取出 f 值最小的节点
    NodeHybrid::NodePtr current = queue_.top().second;
    queue_.pop();

    // 记录搜索过的节点用于可视化
    expand->emplace_back(
      current->pose().x(), current->pose().y(),
      motion_table_.getAngleFromBin(
        static_cast<int>(current->pose().theta())));

    // 如果已访问过（closed），跳过
    if (current->is_visited()) {
      continue;
    }
    iterations++;

    // 步骤 5：标记为已访问
    current->visited();

    // 步骤 6：尝试 Analytic Expansion（Dubins 曲线直连目标）
    NodeHybrid::NodePtr expansion_result =
      tryAnalyticExpansion(current, goal_node);
    if (expansion_result != nullptr) {
      current = expansion_result;
    }

    // 步骤 7：检查是否到达目标
    if (current == goal_node) {
      return backtracePath(current, path);
    } else if (best_heuristic_node_.first <
               static_cast<float>(hybrid_cfg_.goal_tolerance))
    {
      approach_iterations++;
      if (approach_iterations >= hybrid_cfg_.max_approach_iterations) {
        NodeHybrid::NodePtr node_ptr =
          &(graph_.at(best_heuristic_node_.second));
        return backtracePath(node_ptr, path);
      }
    }

    // 步骤 8：扩展邻居
    neighbors.clear();
    getNeighbors(current, neighbors);
    for (auto & nb : neighbors) {
      // 步骤 8.1：计算 g 代价
      double g_cost = current->accumulated_cost() +
                      current->getTraversalCost(nb, motion_table_);

      // 步骤 8.2：若找到更优路径则更新
      if (g_cost < nb->accumulated_cost()) {
        nb->setAccumulatedCost(g_cost);
        nb->parent = current;
        // 步骤 8.3：加入 Open 列表
        addToQueue(
          g_cost + hybrid_cfg_.lambda_h * getHeuristicCost(nb, goal_node),
          nb);
      }
    }
  }

  // 搜索耗尽 — 尝试返回最近的可达路径
  if (best_heuristic_node_.first <
      static_cast<float>(hybrid_cfg_.goal_tolerance))
  {
    NodeHybrid::NodePtr node_ptr = &(graph_.at(best_heuristic_node_.second));
    return backtracePath(node_ptr, path);
  }

  return false;
}

// ===================================================================
// 启发式函数
// ===================================================================

/**
 * @brief 综合启发式 = max(障碍启发, 距离启发)
 *
 * 两种启发函数各有侧重：
 *   - 障碍启发反映绕障代价，但忽略了运动学约束
 *   - 距离启发考虑最小转弯半径，但忽略了障碍物分布
 *   取 max 保证 h(n) 不高估（admissible）。
 */
double HybridAStarPathPlanner::getHeuristicCost(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  return std::max(getObstacleHeuristic(node), getDistanceHeuristic(node, goal));
}

/**
 * @brief 障碍启发 — 查询预计算的 2D Dijkstra 代价场
 */
double HybridAStarPathPlanner::getObstacleHeuristic(
  const NodeHybrid::NodePtr & node) const
{
  const int x = static_cast<int>(node->pose().x());
  const int y = static_cast<int>(node->pose().y());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int width  = static_cast<int>(costmap_->getSizeInCellsX());

  if (x < 0 || x >= width || y < 0 || y >= height) {
    R_ERROR << "Position at " << x << ", " << y << " is out of the map.";
    return std::numeric_limits<double>::max();
  }

  return obstacle_hmap_[y][x];
}

/**
 * @brief 距离启发 — 用 Dubins 曲线计算考虑转弯半径的最短路径长度
 */
double HybridAStarPathPlanner::getDistanceHeuristic(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  CPoints3d motion_path;
  CPoint3d from(
    node->pose().x(), node->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(node->pose().theta())));
  CPoint3d to(
    goal->pose().x(), goal->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(goal->pose().theta())));

  if (motion_table_.curve_gen->generation(from, to, motion_path)) {
    double dist = 0.0;
    for (size_t i = 0; i + 1 < motion_path.size(); ++i) {
      dist += std::hypot(
        motion_path[i].x() - motion_path[i + 1].x(),
        motion_path[i].y() - motion_path[i + 1].y());
    }
    return dist;
  }

  R_WARN << "Heuristic curve generation failed.";
  return -1.0;
}

// ===================================================================
// 预计算障碍启发式 — 从目标反向 Dijkstra
// ===================================================================
bool HybridAStarPathPlanner::precomputeObstacleHeuristic(
  const NodeHybrid::NodePtr & goal)
{
  const int goal_x = static_cast<int>(goal->pose().x());
  const int goal_y = static_cast<int>(goal->pose().y());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int width  = static_cast<int>(costmap_->getSizeInCellsX());

  if (isCollision(goal->pose())) {
    R_ERROR << "Goal not in free space";
    return false;
  }

  // 初始化代价场为无穷大，目标位置为 0
  obstacle_hmap_.clear();
  obstacle_hmap_.resize(height);
  for (auto & row : obstacle_hmap_) {
    row.resize(width, std::numeric_limits<double>::infinity());
  }
  obstacle_hmap_[goal_y][goal_x] = 0.0;

  // 标准 Dijkstra 反向扩展
  using QueueElement = std::pair<float, std::pair<int, int>>;
  std::priority_queue<QueueElement, std::vector<QueueElement>,
                      std::greater<>> queue;
  queue.emplace(0.0f, std::make_pair(goal_y, goal_x));

  while (!queue.empty()) {
    const auto node = queue.top();
    float curr_cost = node.first;
    int y = node.second.first;
    int x = node.second.second;
    queue.pop();

    for (const auto & motion : grid_motions_) {
      const int ny = y + motion.y();
      const int nx = x + motion.x();

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (isCollision(CPoint3d(static_cast<double>(nx), static_cast<double>(ny)))) {
        continue;
      }

      const double new_cost = curr_cost + motion.g();
      if (new_cost < obstacle_hmap_[ny][nx]) {
        obstacle_hmap_[ny][nx] = new_cost;
        queue.emplace(static_cast<float>(new_cost), std::make_pair(ny, nx));
      }
    }
  }

  // 将不可达位置标记为 -1
  for (auto & row : obstacle_hmap_) {
    for (auto & val : row) {
      if (std::isinf(val)) {
        val = -1.0;
      }
    }
  }
  return true;
}

// ===================================================================
// Analytic Expansion — 尝试 Dubins 曲线直连目标
// ===================================================================
NodeHybrid::NodePtr HybridAStarPathPlanner::tryAnalyticExpansion(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal)
{
  // 如果距目标太远，跳过（节省曲线计算开销）
  if (getHeuristicCost(node, goal) >
      hybrid_cfg_.analytic_expansion_max_length / costmap_->getResolution())
  {
    return nullptr;
  }

  CPoints3d motion_path;
  CPoint3d from(
    node->pose().x(), node->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(node->pose().theta())));
  CPoint3d to(
    goal->pose().x(), goal->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(goal->pose().theta())));

  if (motion_table_.curve_gen->generation(from, to, motion_path)) {
    expansions_node_.clear();
    NodeHybrid::NodePtr prev = node;

    // 沿 Dubins 曲线逐点检查碰撞
    for (size_t i = 1; i + 1 < motion_path.size(); i++) {
      auto & pose = motion_path[i];
      pose.setTheta(
        static_cast<double>(motion_table_.getOrientationBin(pose.theta())));
      if (isCollision(pose)) {
        return nullptr;
      }
      auto * n = new NodeHybrid(getIndex(pose));
      n->setPose(pose);
      n->parent = prev;
      n->visited();
      expansions_node_.push_back(n);
      prev = n;
    }
    goal->parent = prev;
    goal->visited();
    return goal;
  }

  return nullptr;
}

// ===================================================================
// 路径回溯
// ===================================================================
bool HybridAStarPathPlanner::backtracePath(
  NodeHybrid::NodePtr & node, CPoints3d * path)
{
  if (!node->parent) {
    return false;
  }

  NodeHybrid::NodePtr current = node;
  while (current->parent) {
    path->push_back(current->pose());
    path->back().setTheta(
      motion_table_.getAngleFromBin(
        static_cast<int>(path->back().theta())));
    current = current->parent;
  }

  // 加入起点
  path->push_back(current->pose());
  path->back().setTheta(
    motion_table_.getAngleFromBin(
      static_cast<int>(path->back().theta())));
  return true;
}

// ===================================================================
// 邻居扩展 — 基于运动原语
// ===================================================================
void HybridAStarPathPlanner::getNeighbors(
  const NodeHybrid::NodePtr & node,
  std::vector<NodeHybrid::NodePtr> & neighbors)
{
  const uint64_t max_index =
    static_cast<uint64_t>(costmap_->getSizeInCellsX()) *
    static_cast<uint64_t>(costmap_->getSizeInCellsY()) *
    static_cast<uint64_t>(hybrid_cfg_.dim_3_size);

  const auto & projections = motion_table_.getMotionPrimitives(node->pose());
  for (size_t i = 0; i < projections.size(); ++i) {
    CPoint3d new_pose(
      projections[i].x(), projections[i].y(), projections[i].theta());
    const uint64_t index = getIndex(new_pose);
    if (index < max_index) {
      NodeHybrid::NodePtr nb = addToGraph(index);
      if (!nb->is_visited()) {
        nb->setPose(new_pose);
        if (!isCollision(nb->pose())) {
          nb->setMotionPrimitiveIndex(
            static_cast<unsigned int>(i), projections[i].turn_dir());
          neighbors.push_back(nb);
        }
      }
    }
  }
}

// ===================================================================
// 碰撞检测
// ===================================================================
bool HybridAStarPathPlanner::isCollision(const CPoint3d & pose)
{
  const int x = static_cast<int>(pose.x());
  const int y = static_cast<int>(pose.y());
  const int width  = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());

  if (x < 0 || x >= width || y < 0 || y >= height) {
    return true;
  }

  const unsigned int index = costmap_->getIndex(
    static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  return costmap_->getCharMap()[index] >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

// ===================================================================
// 图管理与优先队列操作
// ===================================================================
NodeHybrid::NodePtr HybridAStarPathPlanner::addToGraph(uint64_t index)
{
  auto iter = graph_.find(index);
  if (iter != graph_.end()) {
    return &(iter->second);
  }
  auto node = NodeHybrid(index);
  node.setPose(getPose(index));
  return &(graph_.emplace(index, node).first->second);
}

void HybridAStarPathPlanner::clearGraph()
{
  Graph g;
  std::swap(graph_, g);
  graph_.reserve(static_cast<size_t>(hybrid_cfg_.default_graph_size));
}

void HybridAStarPathPlanner::addToQueue(
  double cost, NodeHybrid::NodePtr & node)
{
  queue_.emplace(QueueNode(cost, node));
}

void HybridAStarPathPlanner::clearQueue()
{
  Queue q;
  std::swap(queue_, q);
}

bool HybridAStarPathPlanner::isReachGoal(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  const double dx = node->pose().x() - goal->pose().x();
  const double dy = node->pose().y() - goal->pose().y();
  const double dtheta = node->pose().theta() - goal->pose().theta();
  return (dx * dx + dy * dy + dtheta * dtheta) < 1e-6;
}

}  // namespace rmp::path_planner
