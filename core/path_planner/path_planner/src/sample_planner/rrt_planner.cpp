/**
 * @file rrt_planner.cpp
 * @brief RRT（快速扩展随机树）路径规划器实现 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * =====================================================================
 * 算法总览
 * =====================================================================
 * RRT 是一种基于随机采样的路径规划算法，核心思路是：
 *   在自由空间中不断采样随机点，从已有搜索树中找到最近节点，
 *   沿方向扩展一步（受最大步长约束），并通过 Bresenham 直线碰撞
 *   检测验证新边的可行性。当新节点距目标足够近时，尝试直连目标。
 *
 * 与图搜索算法（A*、Dijkstra 等）的核心区别：
 *   - 图搜索在离散栅格上穷举邻居，保证最优但搜索量大
 *   - RRT 在连续空间中随机采样，不保证最优但能快速覆盖高维空间
 *
 * 迁移变更：
 *   - costmap_2d → nav2_costmap_2d（ROS2 Nav2）
 *   - protobuf 配置 → SamplePlannerConfig 结构体 + ROS2 参数
 *   - rmp::common::geometry::Point3d → rmp::path_planner::Point3d
 *   - 接入 PlannerDebugInfo 可视化（sampled_points + sampled_tree_edges）
 *   - 碰撞检测改为本地 Bresenham 实现，与 public-field Node 兼容
 *   - 随机数生成器改为成员变量，避免每次采样重新初始化
 * =====================================================================
 */
#include "sample_planner/rrt_planner.h"

#include <cmath>
#include <limits>

#include "util/log.h"
#include "geometry/line_collision_checker.h"

using LineChecker = rmp::common::geometry::LineCollisionChecker;

namespace rmp::path_planner {

// ===================================================================
// 构造函数
// ===================================================================
RRTPathPlanner::RRTPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(costmap_ros),
  rng_(std::random_device{}())
{
}

// ===================================================================
// 设置采样类专属参数
// ===================================================================
void RRTPathPlanner::setSampleConfig(const SamplePlannerConfig & cfg)
{
  sample_cfg_ = cfg;
}

// ===================================================================
// plan() — 公共入口
// ===================================================================
bool RRTPathPlanner::plan(
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
  sample_list_.clear();

  // 步骤 2：初始化起点和目标节点（栅格坐标）
  start_.x = static_cast<int>(m_start_x);
  start_.y = static_cast<int>(m_start_y);
  start_.id = grid2Index(start_.x, start_.y);
  start_.g = 0.0;
  start_.parent_id = start_.id;

  goal_.x = static_cast<int>(m_goal_x);
  goal_.y = static_cast<int>(m_goal_y);
  goal_.id = grid2Index(goal_.x, goal_.y);

  // 步骤 3：将起点作为树的根节点加入采样列表
  sample_list_.insert({start_.id, start_});
  expand->emplace_back(Point3d{m_start_x, m_start_y, 0.0});

  // ===================== 主采样循环 =====================
  int iteration = 0;
  while (iteration < sample_cfg_.sample_points) {
    // 步骤 4：生成随机采样点（以一定概率直接返回目标点）
    Node sample_node = generateRandomNode();

    // 步骤 5：跳过障碍物上的采样点
    if (costmap_->getCharMap()[sample_node.id] >=
        nav2_costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor)
    {
      continue;
    }

    // 步骤 6：跳过已在树中的采样点
    if (sample_list_.find(sample_node.id) != sample_list_.end()) {
      continue;
    }

    // 步骤 7：在树中找最近节点，并朝采样点方向以 max_distance 为步长扩展
    Node new_node = findNearestAndSteer(sample_list_, sample_node);

    // 步骤 8：碰撞检测失败则跳过
    if (new_node.id == -1) {
      continue;
    }

    // 步骤 9：将新节点加入树
    sample_list_.insert({new_node.id, new_node});

    // 记录扩展节点用于可视化
    expand->emplace_back(Point3d{
      static_cast<double>(new_node.x),
      static_cast<double>(new_node.y),
      static_cast<double>(new_node.parent_id)});

    // 步骤 10：检查新节点是否能直连目标
    if (checkGoalReachable(new_node)) {
      // 步骤 11：从目标回溯到起点，构建路径
      const auto & backtrace = convertClosedListToPath<Node>(sample_list_, start_, goal_);
      for (auto it = backtrace.rbegin(); it != backtrace.rend(); ++it) {
        double wx, wy;
        costmap_->mapToWorld(
          static_cast<unsigned int>(it->x),
          static_cast<unsigned int>(it->y), wx, wy);
        path->emplace_back(Point3d{wx, wy, 0.0});
      }

      // 步骤 12：填充可视化数据 — 采样点和树边
      fillSampleVisualization(*expand);
      return true;
    }

    iteration++;
  }

  // 采样次数耗尽，未找到路径
  fillSampleVisualization(*expand);
  return false;
}

// ===================================================================
// generateRandomNode() — 随机采样
// ===================================================================
RRTPathPlanner::Node RRTPathPlanner::generateRandomNode()
{
  std::uniform_real_distribution<float> prob(0.0f, 1.0f);

  // 以 optimization_sample_probability 的概率直接返回目标点（目标偏置采样）
  if (prob(rng_) > sample_cfg_.optimization_sample_probability) {
    // 在地图范围内均匀随机采样
    std::uniform_int_distribution<int> distr(0, getMapSize() - 1);
    const int id = distr(rng_);
    int x, y;
    index2Grid(id, x, y);
    return Node(x, y, 0.0, 0.0, id, 0);
  }

  // 目标偏置：直接返回目标点
  return Node(goal_.x, goal_.y, 0.0, 0.0, goal_.id, 0);
}

// ===================================================================
// findNearestAndSteer() — 找最近节点 + 步长截断 + 碰撞检测
// ===================================================================
RRTPathPlanner::Node RRTPathPlanner::findNearestAndSteer(
  std::unordered_map<int, Node> & tree, const Node & sample)
{
  Node nearest_node;
  Node new_node = sample;
  double min_dist = std::numeric_limits<double>::max();

  // 步骤 A：遍历树中所有节点，找到距采样点最近的节点
  for (const auto & [node_id, node] : tree) {
    const double dist = std::hypot(
      node.x - new_node.x, node.y - new_node.y);
    if (dist < min_dist) {
      nearest_node = node;
      new_node.parent_id = nearest_node.id;
      new_node.g = dist + node.g;
      min_dist = dist;
    }
  }

  // 步骤 B：如果距离超过最大步长，沿方向截断到 max_distance
  const double max_dist = sample_cfg_.sample_max_distance;
  if (min_dist > max_dist) {
    const double theta = std::atan2(
      new_node.y - nearest_node.y,
      new_node.x - nearest_node.x);
    new_node.x = nearest_node.x + static_cast<int>(max_dist * std::cos(theta));
    new_node.y = nearest_node.y + static_cast<int>(max_dist * std::sin(theta));
    new_node.id = grid2Index(new_node.x, new_node.y);
    new_node.g = max_dist + nearest_node.g;
  }

  // 步骤 C：Bresenham 直线碰撞检测（最近节点 → 新节点）
  if (isLineCollision(nearest_node.x, nearest_node.y, new_node.x, new_node.y)) {
    new_node.id = -1;
  }

  return new_node;
}

// ===================================================================
// checkGoalReachable() — 检查新节点是否能直连目标
// ===================================================================
bool RRTPathPlanner::checkGoalReachable(const Node & new_node)
{
  const double dist = std::hypot(
    new_node.x - goal_.x, new_node.y - goal_.y);

  // 距离目标超过最大步长，不尝试连接
  if (dist > sample_cfg_.sample_max_distance) {
    return false;
  }

  // Bresenham 碰撞检测：新节点 → 目标
  if (!isLineCollision(new_node.x, new_node.y, goal_.x, goal_.y)) {
    Node goal_node(
      goal_.x, goal_.y, dist + new_node.g, 0.0,
      grid2Index(goal_.x, goal_.y), new_node.id);
    sample_list_.insert({goal_node.id, goal_node});
    return true;
  }

  return false;
}

// ===================================================================
// isLineCollision() — 委托给 LineCollisionChecker 静态模块
// ===================================================================
bool RRTPathPlanner::isLineCollision(int x0, int y0, int x1, int y1) const
{
  return LineChecker::hasCollision(
    x0, y0, x1, y1,
    costmap_->getCharMap(),
    getSizeInCellsX(), getSizeInCellsY(),
    nav2_costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor);
}

// ===================================================================
// fillSampleVisualization() — 填充采样可视化数据
// ===================================================================
void RRTPathPlanner::fillSampleVisualization(const Points3d & expand)
{
  auto & dbg = mutableDebugInfo();

  // 采样点 → sampled_points（蓝色方块）
  dbg.sampled_points.clear();
  dbg.sampled_points.reserve(expand.size());
  for (const auto & ep : expand) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(ep.x),
      static_cast<unsigned int>(ep.y), wx, wy);
    dbg.sampled_points.push_back({wx, wy, 0.0});
  }

  // 树边 → sampled_tree_edges（灰白线段）
  dbg.sampled_tree_edges.clear();
  dbg.sampled_tree_edges.reserve(sample_list_.size());
  for (const auto & [node_id, node] : sample_list_) {
    if (node.id == start_.id) {
      continue;
    }
    auto parent_it = sample_list_.find(node.parent_id);
    if (parent_it == sample_list_.end()) {
      continue;
    }

    double wx1, wy1, wx2, wy2;
    costmap_->mapToWorld(
      static_cast<unsigned int>(node.x),
      static_cast<unsigned int>(node.y), wx1, wy1);
    costmap_->mapToWorld(
      static_cast<unsigned int>(parent_it->second.x),
      static_cast<unsigned int>(parent_it->second.y), wx2, wy2);

    dbg.sampled_tree_edges.push_back({
      {wx1, wy1, 0.0}, {wx2, wy2, 0.0}});
  }
}

}  // namespace rmp::path_planner
