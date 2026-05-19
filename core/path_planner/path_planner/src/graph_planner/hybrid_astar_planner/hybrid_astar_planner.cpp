/**
 * @file hybrid_astar_planner.cpp
 * @brief Hybrid A* global planner migrated from ROS1 to ROS2 Nav2.
 */
#include "graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "common/math/math_helper.h"
#include "common/util/log.h"
#include "nav2_costmap_2d/cost_values.hpp"

using CPoint3d = rmp::common::geometry::Point3d;
using CPoints3d = rmp::common::geometry::Points3d;

namespace rmp::path_planner {

HybridAStarMotionTable HybridAStarPathPlanner::motion_table_;

std::vector<rmp::common::structure::Node<int>> HybridAStarPathPlanner::grid_motions_ = {
  {0, 1, 1.0},
  {1, 0, 1.0},
  {0, -1, 1.0},
  {-1, 0, 1.0},
  {1, 1, std::sqrt(2.0)},
  {1, -1, std::sqrt(2.0)},
  {-1, 1, std::sqrt(2.0)},
  {-1, -1, std::sqrt(2.0)},
};

HybridAStarPathPlanner::HybridAStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros))
{
}

HybridAStarPathPlanner::~HybridAStarPathPlanner()
{
  clearAnalyticExpansionNodes();
}

void HybridAStarPathPlanner::setHybridConfig(const HybridAStarConfig & config)
{
  hybrid_cfg_ = config;
  AINFO << "[HybridA*] config staged: dim_3_size=" << hybrid_cfg_.dim_3_size
        << ", max_iterations=" << hybrid_cfg_.max_iterations
        << ", max_approach_iterations=" << hybrid_cfg_.max_approach_iterations
        << ", goal_tolerance=" << hybrid_cfg_.goal_tolerance
        << ", minimum_turning_radius=" << hybrid_cfg_.minimum_turning_radius
        << ", analytic_expansion_max_length=" << hybrid_cfg_.analytic_expansion_max_length
        << ", lambda_h=" << hybrid_cfg_.lambda_h;
}

CPoint3d HybridAStarPathPlanner::getPose(uint64_t index)
{
  return CPoint3d(
    static_cast<double>(
      (index / static_cast<uint64_t>(motion_table_.num_angle_quantization)) %
      static_cast<uint64_t>(motion_table_.map_width)),
    static_cast<double>(
      index /
      (static_cast<uint64_t>(motion_table_.num_angle_quantization) *
      static_cast<uint64_t>(motion_table_.map_width))),
    static_cast<double>(index % static_cast<uint64_t>(motion_table_.num_angle_quantization)));
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

bool HybridAStarPathPlanner::plan(
  const Point3d & start,
  const Point3d & goal,
  Points3d * path,
  Points3d * expand)
{
  if (!costmap_) {
    AERROR << "[HybridA*] plan failed: costmap is null.";
    return false;
  }

  AINFO << "[HybridA*] plan request world_start=("
        << start.x << ", " << start.y << ", " << start.theta << ")"
        << ", world_goal=(" << goal.x << ", " << goal.y << ", " << goal.theta << ")"
        << ", costmap_size=(" << costmap_->getSizeInCellsX() << ", "
        << costmap_->getSizeInCellsY() << ")"
        << ", resolution=" << costmap_->getResolution();

  if (!initializeMotionTableFromCostmap()) {
    return false;
  }

  // 步骤 1：将 Nav2 世界坐标转换为 costmap 栅格坐标，并保留起终点 yaw。
  double start_mx;
  double start_my;
  double goal_mx;
  double goal_my;
  if (!validityCheck(start.x, start.y, start_mx, start_my) ||
    !validityCheck(goal.x, goal.y, goal_mx, goal_my))
  {
    AERROR << "[HybridA*] plan failed: start or goal is outside costmap. "
           << "world_start=(" << start.x << ", " << start.y << ", " << start.theta << ")"
           << ", world_goal=(" << goal.x << ", " << goal.y << ", " << goal.theta << ")";
    return false;
  }

  AINFO << "[HybridA*] world to map converted: map_start=("
        << start_mx << ", " << start_my << ", yaw=" << start.theta << ")"
        << ", map_goal=(" << goal_mx << ", " << goal_my << ", yaw=" << goal.theta << ")";

  path->clear();
  expand->clear();

  // 步骤 2：保留 ROS1 版本的 Hybrid A* 局部路径复用逻辑。
  // 通用基类已经做了一层复用；这里用于处理 Hybrid A* 自己保存的连续朝向路径。
  const CPoint3d current_goal(goal.x, goal.y, goal.theta);
  if (!hybrid_last_path_.empty() && stored_goal_ == current_goal) {
    AINFO << "[HybridA*] checking cached path reuse: cached_points="
          << hybrid_last_path_.size();
    bool collision = false;
    auto closest_iter = hybrid_last_path_.begin();
    double min_dist = std::numeric_limits<double>::max();

    for (auto iter = hybrid_last_path_.begin(); iter != hybrid_last_path_.end(); ++iter) {
      unsigned int mx;
      unsigned int my;
      if (!costmap_->worldToMap(iter->x(), iter->y(), mx, my) ||
        isCollision(CPoint3d(static_cast<double>(mx), static_cast<double>(my), iter->theta())))
      {
        AWARN << "[HybridA*] cached path reuse rejected: collision or out of map at world=("
              << iter->x() << ", " << iter->y() << ", " << iter->theta() << ")";
        collision = true;
        break;
      }

      const double dist = std::hypot(iter->x() - start.x, iter->y() - start.y);
      if (dist < min_dist) {
        min_dist = dist;
        closest_iter = iter;
      }
    }

    if (!collision) {
      for (auto iter = closest_iter; iter != hybrid_last_path_.end(); ++iter) {
        path->push_back({iter->x(), iter->y(), iter->theta()});
      }
      hybrid_last_path_ = CPoints3d(closest_iter, hybrid_last_path_.end());
      AINFO << "[HybridA*] cached path reused: output_points=" << path->size()
            << ", nearest_distance=" << min_dist;
      return true;
    }

    AWARN << "[HybridA*] cached path cleared, replanning from scratch.";
    hybrid_last_path_.clear();
  }

  // 步骤 3：将 yaw 量化为 theta bin，进入 Hybrid A* 三维搜索空间。
  stored_goal_ = current_goal;
  CPoints3d path_in_map;
  CPoints3d expand_in_map;
  const CPoint3d start_map(
    start_mx,
    start_my,
    static_cast<double>(motion_table_.getOrientationBin(start.theta)));
  const CPoint3d goal_map(
    goal_mx,
    goal_my,
    static_cast<double>(motion_table_.getOrientationBin(goal.theta)));

  AINFO << "[HybridA*] quantized poses: start=("
        << start_map.x() << ", " << start_map.y() << ", theta_bin=" << start_map.theta()
        << "), goal=(" << goal_map.x() << ", " << goal_map.y()
        << ", theta_bin=" << goal_map.theta() << ")";

  if (!createPath(start_map, goal_map, &path_in_map, &expand_in_map)) {
    fillSearchedPointsDebugInfo(expand_in_map);
    AERROR << "[HybridA*] plan failed: createPath returned false. expanded_points="
           << expand_in_map.size() << ", map_start=(" << start_map.x() << ", "
           << start_map.y() << ", " << start_map.theta() << ")"
           << ", map_goal=(" << goal_map.x() << ", " << goal_map.y() << ", "
           << goal_map.theta() << ")";
    return false;
  }

  // 步骤 4：回溯结果是从目标到起点，反向输出并转换回世界坐标。
  for (auto iter = path_in_map.rbegin(); iter != path_in_map.rend(); ++iter) {
    // Hybrid A* 的节点索引、启发式地图和碰撞检测都按 costmap 栅格节点解释。
    // 输出到世界坐标时应使用栅格中心点，即 origin + (map + 0.5) * resolution，
    // 与 Costmap2D::mapToWorld 和其他 2D planner 的路径坐标语义保持一致。
    double wx;
    double wy;
    map2World(iter->x(), iter->y(), wx, wy);
    path->push_back({wx, wy, iter->theta()});
  }

  // 步骤 5：保存路径供下一次规划复用，并填充统一的调试可视化数据。
  hybrid_last_path_.clear();
  for (const auto & point : *path) {
    hybrid_last_path_.emplace_back(point.x, point.y, point.theta);
  }
  fillSearchedPointsDebugInfo(expand_in_map);
  if (!path->empty()) {
    const auto & first = path->front();
    const auto & last = path->back();
    AINFO << "[HybridA*] plan succeeded: path_points=" << path->size()
          << ", expanded_points=" << expand_in_map.size()
          << ", first=(" << first.x << ", " << first.y << ", " << first.theta << ")"
          << ", last=(" << last.x << ", " << last.y << ", " << last.theta << ")";
  } else {
    AWARN << "[HybridA*] createPath succeeded but output path is empty.";
  }
  return true;
}

bool HybridAStarPathPlanner::createPath(
  const CPoint3d & start,
  const CPoint3d & goal,
  CPoints3d * path,
  CPoints3d * expand)
{
  // 步骤 1：清理上一轮搜索状态，创建起点和目标节点。
  clearGraph();
  clearQueue();
  clearAnalyticExpansionNodes();
  best_heuristic_node_ = {std::numeric_limits<float>::max(), 0};
  auto start_node = addToGraph(getIndex(start));
  auto goal_node = addToGraph(getIndex(goal));

  AINFO << "[HybridA*] createPath begin: start_map=("
        << start.x() << ", " << start.y() << ", theta_bin=" << start.theta() << ")"
        << ", goal_map=(" << goal.x() << ", " << goal.y()
        << ", theta_bin=" << goal.theta() << ")"
        << ", start_index=" << getIndex(start)
        << ", goal_index=" << getIndex(goal);

  // 步骤 2：从目标反向做一次 2D Dijkstra，预计算绕障启发式代价场。
  if (!precomputeObstacleHeuristic(goal_node)) {
    AERROR << "[HybridA*] createPath failed: obstacle heuristic precomputation failed.";
    return false;
  }

  // 步骤 3：初始化 open list。
  addToQueue(0.0, start_node);
  start_node->setAccumulatedCost(0.0);

  std::vector<NodeHybrid::NodePtr> neighbors;
  int iterations = 0;
  int approach_iterations = 0;

  // 步骤 4：Hybrid A* 主循环，每次取 f=g+lambda*h 最小的 3D 节点扩展。
  while (iterations < hybrid_cfg_.max_iterations && !queue_.empty()) {
    NodeHybrid::NodePtr current = queue_.top().second;
    queue_.pop();

    expand->emplace_back(
      current->pose().x(),
      current->pose().y(),
      motion_table_.getAngleFromBin(static_cast<int>(current->pose().theta())));

    if (current->is_visited()) {
      continue;
    }
    ++iterations;

    const double heuristic = getHeuristicCost(current, goal_node);
    if (heuristic >= 0.0 && heuristic < best_heuristic_node_.first) {
      best_heuristic_node_ = {static_cast<float>(heuristic), getIndex(current->pose())};
    }

    AINFO_EVERY(500) << "[HybridA*] searching: iteration=" << iterations
                     << ", queue_size=" << queue_.size()
                     << ", graph_size=" << graph_.size()
                     << ", current_map=(" << current->pose().x() << ", "
                     << current->pose().y() << ", theta_bin="
                     << current->pose().theta() << ")"
                     << ", g=" << current->accumulated_cost()
                     << ", h=" << heuristic
                     << ", best_h=" << best_heuristic_node_.first;

    // 步骤 5：标记 closed，并在足够靠近目标时尝试 Dubins 曲线直接连接。
    current->visited();
    if (auto expansion_result = tryAnalyticExpansion(current, goal_node)) {
      current = expansion_result;
      AINFO << "[HybridA*] analytic expansion connected to goal at iteration="
            << iterations;
    }

    // 步骤 6：若到达目标或已足够接近目标，则回溯路径。
    if (current == goal_node || isReachGoal(current, goal_node)) {
      const bool traced = backtracePath(current, path);
      AINFO << "[HybridA*] goal reached: iteration=" << iterations
            << ", traced=" << traced
            << ", backtrace_points=" << path->size()
            << ", expanded_points=" << expand->size();
      return traced;
    }

    if (best_heuristic_node_.first < static_cast<float>(hybrid_cfg_.goal_tolerance)) {
      ++approach_iterations;
      if (approach_iterations >= hybrid_cfg_.max_approach_iterations) {
        NodeHybrid::NodePtr node_ptr = &(graph_.at(best_heuristic_node_.second));
        const bool traced = backtracePath(node_ptr, path);
        AWARN << "[HybridA*] approach iteration limit reached: iterations=" << iterations
              << ", approach_iterations=" << approach_iterations
              << ", best_h=" << best_heuristic_node_.first
              << ", traced=" << traced
              << ", backtrace_points=" << path->size();
        return traced;
      }
    }

    // 步骤 7：按运动原语扩展直行、左转、右转候选，并更新更优父节点。
    neighbors.clear();
    getNeighbors(current, neighbors);
    for (auto & neighbor : neighbors) {
      const double g_cost =
        current->accumulated_cost() + current->getTraversalCost(neighbor, motion_table_);
      if (g_cost < neighbor->accumulated_cost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current;
        addToQueue(g_cost + hybrid_cfg_.lambda_h * getHeuristicCost(neighbor, goal_node), neighbor);
      }
    }
  }

  // 步骤 8：搜索耗尽时，如果存在足够接近目标的节点，则返回该近似路径。
  if (best_heuristic_node_.first < static_cast<float>(hybrid_cfg_.goal_tolerance)) {
    NodeHybrid::NodePtr node_ptr = &(graph_.at(best_heuristic_node_.second));
    const bool traced = backtracePath(node_ptr, path);
    AWARN << "[HybridA*] search ended, returning closest path: best_h="
          << best_heuristic_node_.first
          << ", graph_size=" << graph_.size()
          << ", expanded_points=" << expand->size()
          << ", traced=" << traced
          << ", backtrace_points=" << path->size();
    return traced;
  }

  AERROR << "[HybridA*] search failed: iterations=" << iterations
         << ", queue_empty=" << queue_.empty()
         << ", queue_size=" << queue_.size()
         << ", graph_size=" << graph_.size()
         << ", expanded_points=" << expand->size()
         << ", best_h=" << best_heuristic_node_.first
         << ", goal_tolerance=" << hybrid_cfg_.goal_tolerance;
  return false;
}

double HybridAStarPathPlanner::getHeuristicCost(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  return std::max(getObstacleHeuristic(node), getDistanceHeuristic(node, goal));
}

double HybridAStarPathPlanner::getObstacleHeuristic(const NodeHybrid::NodePtr & node) const
{
  const int x = static_cast<int>(node->pose().x());
  const int y = static_cast<int>(node->pose().y());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int width = static_cast<int>(costmap_->getSizeInCellsX());

  if (x < 0 || x >= width || y < 0 || y >= height) {
    AWARN << "[HybridA*] obstacle heuristic query out of map: pose=("
          << x << ", " << y << "), map_size=(" << width << ", " << height << ")";
    return std::numeric_limits<double>::max();
  }

  return obstacle_hmap_[y][x];
}

double HybridAStarPathPlanner::getDistanceHeuristic(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  CPoints3d motion_path;
  const CPoint3d from(
    node->pose().x(),
    node->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(node->pose().theta())));
  const CPoint3d to(
    goal->pose().x(),
    goal->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(goal->pose().theta())));

  if (!motion_table_.curve_gen || !motion_table_.curve_gen->generation(from, to, motion_path)) {
    AWARN << "[HybridA*] distance heuristic curve generation failed: from=("
          << from.x() << ", " << from.y() << ", " << from.theta() << ")"
          << ", to=(" << to.x() << ", " << to.y() << ", " << to.theta() << ")";
    return -1.0;
  }

  double dist = 0.0;
  for (std::size_t i = 0; i + 1 < motion_path.size(); ++i) {
    dist += std::hypot(
      motion_path[i].x() - motion_path[i + 1].x(),
      motion_path[i].y() - motion_path[i + 1].y());
  }
  return dist;
}

bool HybridAStarPathPlanner::precomputeObstacleHeuristic(const NodeHybrid::NodePtr & goal)
{
  const int goal_x = static_cast<int>(goal->pose().x());
  const int goal_y = static_cast<int>(goal->pose().y());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const int width = static_cast<int>(costmap_->getSizeInCellsX());

  if (isCollision(goal->pose())) {
    AERROR << "[HybridA*] Goal not in free space: goal_map=("
           << goal_x << ", " << goal_y << ", theta_bin=" << goal->pose().theta()
           << "), costmap_size=(" << width << ", " << height << ")";
    return false;
  }

  obstacle_hmap_.clear();
  obstacle_hmap_.resize(height);
  for (auto & row : obstacle_hmap_) {
    row.resize(width, std::numeric_limits<double>::infinity());
  }
  obstacle_hmap_[goal_y][goal_x] = 0.0;

  using QueueElement = std::pair<float, std::pair<int, int>>;
  std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> queue;
  queue.emplace(0.0f, std::make_pair(goal_y, goal_x));

  while (!queue.empty()) {
    const auto node = queue.top();
    const float current_cost = node.first;
    const int y = node.second.first;
    const int x = node.second.second;
    queue.pop();

    for (const auto & motion : grid_motions_) {
      const int ny = y + motion.y();
      const int nx = x + motion.x();

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (isCollision(CPoint3d(static_cast<double>(nx), static_cast<double>(ny), 0.0))) {
        continue;
      }

      const double new_cost = current_cost + motion.g();
      if (new_cost < obstacle_hmap_[ny][nx]) {
        obstacle_hmap_[ny][nx] = new_cost;
        queue.emplace(static_cast<float>(new_cost), std::make_pair(ny, nx));
      }
    }
  }

  std::size_t reachable_cells = 0;
  std::size_t unreachable_cells = 0;

  for (auto & row : obstacle_hmap_) {
    for (auto & value : row) {
      if (std::isinf(value)) {
        value = -1.0;
        ++unreachable_cells;
      } else {
        ++reachable_cells;
      }
    }
  }

  AINFO << "[HybridA*] obstacle heuristic ready: goal_map=(" << goal_x << ", "
        << goal_y << "), reachable_cells=" << reachable_cells
        << ", unreachable_cells=" << unreachable_cells
        << ", map_cells=" << static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  return true;
}

NodeHybrid::NodePtr HybridAStarPathPlanner::tryAnalyticExpansion(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal)
{
  if (getHeuristicCost(node, goal) >
    hybrid_cfg_.analytic_expansion_max_length / costmap_->getResolution())
  {
    return nullptr;
  }

  CPoints3d motion_path;
  const CPoint3d from(
    node->pose().x(),
    node->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(node->pose().theta())));
  const CPoint3d to(
    goal->pose().x(),
    goal->pose().y(),
    motion_table_.getAngleFromBin(static_cast<int>(goal->pose().theta())));

  if (!motion_table_.curve_gen || !motion_table_.curve_gen->generation(from, to, motion_path)) {
    AWARN << "[HybridA*] analytic expansion curve generation failed: from=("
          << from.x() << ", " << from.y() << ", " << from.theta() << ")"
          << ", to=(" << to.x() << ", " << to.y() << ", " << to.theta() << ")";
    return nullptr;
  }

  clearAnalyticExpansionNodes();
  NodeHybrid::NodePtr previous = node;
  for (std::size_t i = 1; i + 1 < motion_path.size(); ++i) {
    auto & pose = motion_path[i];
    pose.setTheta(static_cast<double>(motion_table_.getOrientationBin(pose.theta())));
    if (isCollision(pose)) {
      AWARN << "[HybridA*] analytic expansion rejected by collision: map_pose=("
            << pose.x() << ", " << pose.y() << ", theta_bin=" << pose.theta() << ")";
      return nullptr;
    }

    auto * next = new NodeHybrid(getIndex(pose));
    next->setPose(pose);
    next->parent = previous;
    next->visited();
    expansions_node_.push_back(next);
    previous = next;
  }

  goal->parent = previous;
  goal->visited();
  AINFO << "[HybridA*] analytic expansion succeeded: intermediate_nodes="
        << expansions_node_.size() << ", sampled_points=" << motion_path.size();
  return goal;
}

bool HybridAStarPathPlanner::backtracePath(NodeHybrid::NodePtr & node, CPoints3d * path)
{
  if (!node->parent) {
    AERROR << "[HybridA*] backtrace failed: final node has no parent. node_map=("
           << node->pose().x() << ", " << node->pose().y()
           << ", theta_bin=" << node->pose().theta() << ")";
    return false;
  }

  NodeHybrid::NodePtr current = node;
  while (current->parent) {
    path->push_back(current->pose());
    path->back().setTheta(
      motion_table_.getAngleFromBin(static_cast<int>(path->back().theta())));
    current = current->parent;
  }

  path->push_back(current->pose());
  path->back().setTheta(
    motion_table_.getAngleFromBin(static_cast<int>(path->back().theta())));
  AINFO << "[HybridA*] backtrace completed: map_path_points=" << path->size();
  return true;
}

void HybridAStarPathPlanner::getNeighbors(
  const NodeHybrid::NodePtr & node,
  std::vector<NodeHybrid::NodePtr> & neighbors)
{
  const uint64_t max_index =
    static_cast<uint64_t>(costmap_->getSizeInCellsX()) *
    static_cast<uint64_t>(costmap_->getSizeInCellsY()) *
    static_cast<uint64_t>(hybrid_cfg_.dim_3_size);

  const auto projections = motion_table_.getMotionPrimitives(node->pose());
  for (std::size_t i = 0; i < projections.size(); ++i) {
    const CPoint3d new_pose(projections[i].x(), projections[i].y(), projections[i].theta());
    const uint64_t index = getIndex(new_pose);
    if (index >= max_index) {
      continue;
    }

    auto neighbor = addToGraph(index);
    if (neighbor->is_visited()) {
      continue;
    }

    neighbor->setPose(new_pose);
    if (!isCollision(neighbor->pose())) {
      neighbor->setMotionPrimitiveIndex(
        static_cast<unsigned int>(i), projections[i].turn_dir());
      neighbors.push_back(neighbor);
    }
  }
}

bool HybridAStarPathPlanner::isCollision(const CPoint3d & pose) const
{
  const int x = static_cast<int>(pose.x());
  const int y = static_cast<int>(pose.y());
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());

  if (x < 0 || x >= width || y < 0 || y >= height) {
    return true;
  }

  return !isNodeCollisionFree(x, y);
}

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
  Graph graph;
  std::swap(graph_, graph);
  graph_.reserve(static_cast<std::size_t>(hybrid_cfg_.default_graph_size));
}

void HybridAStarPathPlanner::addToQueue(double cost, NodeHybrid::NodePtr & node)
{
  queue_.emplace(QueueNode(cost, node));
}

void HybridAStarPathPlanner::clearQueue()
{
  Queue queue;
  std::swap(queue_, queue);
}

bool HybridAStarPathPlanner::isReachGoal(
  const NodeHybrid::NodePtr & node,
  const NodeHybrid::NodePtr & goal) const
{
  const double dx = node->pose().x() - goal->pose().x();
  const double dy = node->pose().y() - goal->pose().y();
  const double dtheta = node->pose().theta() - goal->pose().theta();
  return (dx * dx + dy * dy + dtheta * dtheta) < rmp::common::math::kMathEpsilon;
}

bool HybridAStarPathPlanner::initializeMotionTableFromCostmap()
{
  if (!costmap_) {
    AERROR << "[HybridA*] motion table init failed: costmap is null.";
    return false;
  }

  const auto size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const auto size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const double resolution = costmap_->getResolution();
  if (size_x <= 0 || size_y <= 0 || resolution <= 0.0) {
    AERROR << "[HybridA*] motion table init failed: invalid costmap metadata. "
           << "size=(" << size_x << ", " << size_y << "), resolution=" << resolution;
    return false;
  }

  const double minimum_turning_radius_in_cells =
    hybrid_cfg_.minimum_turning_radius / resolution;
  motion_table_.initDubins(
    hybrid_cfg_.dim_3_size,
    minimum_turning_radius_in_cells,
    size_x,
    hybrid_cfg_.curve_sample_ratio,
    hybrid_cfg_.change_penalty,
    hybrid_cfg_.non_straight_penalty,
    hybrid_cfg_.reverse_penalty,
    hybrid_cfg_.retrospective_penalty);

  AINFO << "[HybridA*] motion table initialized from current costmap: map_size=("
        << size_x << ", " << size_y << ")"
        << ", map_width=" << motion_table_.map_width
        << ", resolution=" << resolution
        << ", minimum_turning_radius_in_cells=" << minimum_turning_radius_in_cells
        << ", dim_3_size=" << motion_table_.num_angle_quantization;
  return true;
}

void HybridAStarPathPlanner::clearAnalyticExpansionNodes()
{
  for (auto * node : expansions_node_) {
    delete node;
  }
  expansions_node_.clear();
}

void HybridAStarPathPlanner::fillSearchedPointsDebugInfo(const CPoints3d & expand)
{
  auto & searched_points = mutableDebugInfo().searched_points;
  searched_points.clear();
  searched_points.reserve(expand.size());

  for (const auto & point : expand) {
    double wx;
    double wy;
    map2World(point.x(), point.y(), wx, wy);
    searched_points.push_back({wx, wy, point.theta()});
  }
}

}  // namespace rmp::path_planner
