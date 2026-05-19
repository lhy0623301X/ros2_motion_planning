/**
 * @file dstar_lite_planner.cpp
 * @brief D* Lite 增量式全局规划器 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * D* Lite 是 D* 的简化版本，基于 LPA* 思想：
 *   - 从目标向起点反向搜索（与 LPA* 方向相反）
 *   - 使用 rhs 值和双键（key = min(g, rhs) + h + km）进行优先级排序
 *   - 当代价地图变化时，仅对受影响区域进行增量修复
 *   - km 修正项用于补偿机器人移动带来的启发式偏差
 */
#include "graph_planner/dstar_lite_planner.h"

#include "util/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

DStarLitePathPlanner::DStarLitePathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros)),
  curr_global_costmap_(nullptr),
  last_global_costmap_(nullptr),
  map_(nullptr),
  start_ptr_(nullptr),
  goal_ptr_(nullptr),
  last_ptr_(nullptr),
  km_(0.0),
  nx_(0), ny_(0), map_size_(0),
  initialized_(false)
{
  start_.x = std::numeric_limits<int>::max();
  start_.y = std::numeric_limits<int>::max();
  goal_.x = std::numeric_limits<int>::max();
  goal_.y = std::numeric_limits<int>::max();
}

DStarLitePathPlanner::~DStarLitePathPlanner()
{
  freeMap();
}

/**
 * @brief 释放节点网格和代价地图快照的所有动态内存。
 */
void DStarLitePathPlanner::freeMap()
{
  delete[] curr_global_costmap_;
  curr_global_costmap_ = nullptr;

  delete[] last_global_costmap_;
  last_global_costmap_ = nullptr;

  if (map_) {
    for (int i = 0; i < nx_; ++i) {
      for (int j = 0; j < ny_; ++j) {
        delete map_[i][j];
      }
      delete[] map_[i];
    }
    delete[] map_;
    map_ = nullptr;
  }
}

/**
 * @brief 初始化节点网格：为每个栅格单元创建 LNode，
 *        open_it 指向 open_list_ 的 end() 表示尚未入队。
 */
void DStarLitePathPlanner::initMap()
{
  map_ = new LNodePtr *[nx_];
  for (int i = 0; i < nx_; ++i) {
    map_[i] = new LNodePtr[ny_];
    for (int j = 0; j < ny_; ++j) {
      map_[i][j] = new LNode(i, j, INF, INF, grid2Index(i, j), -1, INF, INF);
      map_[i][j]->open_it = open_list_.end();
    }
  }
}

/**
 * @brief 完全重置搜索状态：清空 open list 并释放旧的节点网格后重建。
 */
void DStarLitePathPlanner::reset()
{
  open_list_.clear();

  if (map_) {
    for (int i = 0; i < nx_; ++i) {
      for (int j = 0; j < ny_; ++j) {
        delete map_[i][j];
      }
      delete[] map_[i];
    }
    delete[] map_;
    map_ = nullptr;
  }

  initMap();
}

/**
 * @brief 延迟初始化 — 确保地图资源已分配，仅在首次 plan() 时执行。
 */
void DStarLitePathPlanner::ensureInitialized()
{
  if (initialized_) {
    return;
  }

  nx_ = getSizeInCellsX();
  ny_ = getSizeInCellsY();
  map_size_ = getMapSize();

  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  std::memcpy(
    curr_global_costmap_, getCostMap()->getCharMap(),
    static_cast<std::size_t>(map_size_));
  std::memcpy(
    last_global_costmap_, curr_global_costmap_,
    static_cast<std::size_t>(map_size_));

  initMap();
  initialized_ = true;
}

/**
 * @brief 启发式函数：两节点之间的欧几里得距离。
 */
double DStarLitePathPlanner::getH(LNodePtr n1, LNodePtr n2) const
{
  return std::hypot(n1->x - n2->x, n1->y - n2->y);
}

/**
 * @brief 计算节点优先级键值：key = min(g, rhs) + h(s, start) + km。
 *
 * D* Lite 反向搜索，启发式方向指向 start（当前机器人位置）。
 * km 是累计的启发式修正项，补偿机器人移动导致的键值偏移。
 */
double DStarLitePathPlanner::calculateKey(LNodePtr s) const
{
  return std::min(s->g, s->rhs) + 0.9 * getH(s, start_ptr_) + km_;
}

/**
 * @brief 检测两相邻节点之间是否存在障碍物碰撞。
 */
bool DStarLitePathPlanner::isCollision(LNodePtr n1, LNodePtr n2) const
{
  if (n1->id < 0 || n1->id >= map_size_ ||
      n2->id < 0 || n2->id >= map_size_)
  {
    return true;
  }

  const double lethal = nav2_costmap_2d::LETHAL_OBSTACLE *
    config().obstacle_inflation_factor;

  if (curr_global_costmap_[n1->id] >= lethal &&
      curr_global_costmap_[n1->id] >= curr_global_costmap_[n2->id])
  {
    return true;
  }

  if (curr_global_costmap_[n2->id] >= lethal &&
      curr_global_costmap_[n2->id] >= curr_global_costmap_[n1->id])
  {
    return true;
  }

  return false;
}

/**
 * @brief 获取节点的 8 邻域有效邻居（不越界、不碰撞）。
 */
void DStarLitePathPlanner::getNeighbours(
  LNodePtr u, std::vector<LNodePtr> & neighbours) const
{
  neighbours.clear();
  static const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
  static const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

  for (int i = 0; i < 8; ++i) {
    int nbx = u->x + dx[i];
    int nby = u->y + dy[i];
    if (nbx < 0 || nbx >= nx_ || nby < 0 || nby >= ny_) {
      continue;
    }
    LNodePtr nb = map_[nbx][nby];
    if (!isCollision(u, nb)) {
      neighbours.push_back(nb);
    }
  }
}

/**
 * @brief 将 costmap 代价值映射为障碍物邻近惩罚。
 *
 * inflation layer 会让靠近障碍物的栅格拥有更高的代价值，
 * 这里复用与 A* / D* 一致的 sigmoid 映射，让路径更倾向于远离障碍物边缘。
 */
double DStarLitePathPlanner::calculateObstacleCost(unsigned char cell_cost) const
{
  const double normalized_cost =
    static_cast<double>(cell_cost) / static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const double clamped_cost = std::clamp(normalized_cost, 0.0, 1.0);
  const double sigmoid =
    1.0 / (1.0 + std::exp(-config().obstacle_sigmoid_alpha *
    (clamped_cost - config().obstacle_sigmoid_center)));

  return config().obstacle_cost_weight * sigmoid;
}

/**
 * @brief 计算两相邻节点之间的移动代价（欧几里得距离 + 障碍物邻近惩罚）。
 */
double DStarLitePathPlanner::getCost(LNodePtr n1, LNodePtr n2) const
{
  if (isCollision(n1, n2)) {
    return INF;
  }

  const double motion_cost = std::hypot(n1->x - n2->x, n1->y - n2->y);
  const double obstacle_cost = calculateObstacleCost(curr_global_costmap_[n2->id]);
  return motion_cost + obstacle_cost;
}

/**
 * @brief 更新节点的 rhs 值与 open list 状态。
 *
 * D* Lite 反向搜索：goal 节点的 rhs 始终为 0（搜索源），不参与更新。
 */
void DStarLitePathPlanner::updateVertex(LNodePtr u)
{
  // 步骤 1：对非目标节点，从所有后继中选取最小 rhs 值。
  if (u->x != goal_.x || u->y != goal_.y) {
    u->rhs = INF;
    std::vector<LNodePtr> neighbours;
    getNeighbours(u, neighbours);
    for (const auto & s : neighbours) {
      const double new_rhs = s->g + getCost(s, u);
      if (new_rhs < u->rhs) {
        u->rhs = new_rhs;
      }
    }
  }

  // 步骤 2：如果节点已在 open list 中，先移除旧条目。
  if (u->open_it != open_list_.end()) {
    open_list_.erase(u->open_it);
    u->open_it = open_list_.end();
  }

  // 步骤 3：若 g ≠ rhs（局部不一致），重新插入 open list。
  if (u->g != u->rhs) {
    u->key = calculateKey(u);
    u->open_it = open_list_.insert(std::make_pair(u->key, u));
  }
}

/**
 * @brief 增量计算最短路径。
 *
 * 反复处理 open list 中键值最小的节点，直到：
 *   - start 节点局部一致（g == rhs）且其键值不小于 open list 最小键值
 *   - 或 open list 为空
 */
void DStarLitePathPlanner::computeShortestPath()
{
  while (!open_list_.empty()) {
    // 步骤 1：取出 open list 中键值最小的节点。
    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->open_it = open_list_.end();

    expand_.push_back(
      Point3d{static_cast<double>(u->x), static_cast<double>(u->y), 0.0});

    // 步骤 2：检查终止条件 — start 局部一致且当前键值 ≥ start 键值。
    if (u->key >= calculateKey(start_ptr_) &&
        start_ptr_->rhs == start_ptr_->g)
    {
      break;
    }

    // 步骤 3：根据局部一致性关系处理节点。
    if (u->g > u->rhs) {
      // 步骤 3a：过一致（overconsistent）— 降低 g 值使其一致。
      u->g = u->rhs;
    } else {
      // 步骤 3b：欠一致（underconsistent）— 将 g 设为 INF 触发重传播。
      u->g = INF;
      updateVertex(u);
    }

    // 步骤 4：传播变化到所有邻居节点。
    std::vector<LNodePtr> neighbours;
    getNeighbours(u, neighbours);
    for (auto & s : neighbours) {
      updateVertex(s);
    }
  }
}

/**
 * @brief 从 start 贪心追踪到 goal 提取路径。
 *
 * D* Lite 反向搜索完成后，start 处的 g 值有效。
 * 从 start 出发，每步选择 g 值最小的邻居，直到到达 goal。
 */
bool DStarLitePathPlanner::extractPath(
  const LNode & start, const LNode & goal)
{
  Points3d path_temp;
  LNodePtr node_ptr = map_[start.x][start.y];
  int count = 0;

  while (!(node_ptr->x == goal.x && node_ptr->y == goal.y)) {
    double wx, wy;
    map2World(node_ptr->x, node_ptr->y, wx, wy);
    path_temp.push_back(Point3d{wx, wy, 0.0});

    std::vector<LNodePtr> neighbours;
    getNeighbours(node_ptr, neighbours);

    LNodePtr best = nullptr;
    double min_g = INF;
    for (auto & nb : neighbours) {
      if (nb->g < min_g) {
        min_g = nb->g;
        best = nb;
      }
    }

    if (!best || ++count > map_size_) {
      return false;
    }
    node_ptr = best;
  }

  double wx, wy;
  map2World(goal.x, goal.y, wx, wy);
  path_temp.push_back(Point3d{wx, wy, 0.0});

  path_ = path_temp;
  return true;
}

/**
 * @brief 获取给定坐标处的节点状态。
 */
DStarLitePathPlanner::LNode DStarLitePathPlanner::getState(
  const LNode & current) const
{
  return *map_[current.x][current.y];
}

void DStarLitePathPlanner::fillSearchedPointsDebugInfo(
  const Points3d & expand)
{
  auto & searched_points = mutableDebugInfo().searched_points;
  searched_points.clear();
  searched_points.reserve(expand.size());

  for (const auto & point : expand) {
    double wx, wy;
    map2World(point.x, point.y, wx, wy);
    searched_points.push_back({wx, wy, point.theta});
  }
}

/**
 * @brief 主规划入口。
 *
 * 首次调用时执行完整初始化和搜索。后续调用：
 *   - 如果目标改变 → 完全重置并重新搜索
 *   - 如果仅地图代价变化 → 增量更新受影响节点并重新计算
 *   - km_ 随机器人移动累加，补偿启发式偏差
 */
bool DStarLitePathPlanner::plan(
  const Point3d & start,
  const Point3d & goal,
  Points3d * path,
  Points3d * expand)
{
  // 步骤 1：将起点和终点从世界坐标转换到代价地图栅格坐标。
  double start_mx, start_my, goal_mx, goal_my;
  if (!validityCheck(start.x, start.y, start_mx, start_my) ||
      !validityCheck(goal.x, goal.y, goal_mx, goal_my))
  {
    return false;
  }

  if (config().outline_map) {
    outlineMap();
  }

  const int s_x = static_cast<int>(start_mx);
  const int s_y = static_cast<int>(start_my);
  const int g_x = static_cast<int>(goal_mx);
  const int g_y = static_cast<int>(goal_my);

  // 步骤 2：延迟初始化。
  ensureInitialized();

  // 步骤 3：更新当前代价地图快照。
  std::memcpy(
    curr_global_costmap_, getCostMap()->getCharMap(),
    static_cast<std::size_t>(map_size_));

  const LNode new_start(s_x, s_y, INF, INF, grid2Index(s_x, s_y), -1, INF, INF);
  const LNode new_goal(g_x, g_y, INF, INF, grid2Index(g_x, g_y), -1, INF, INF);

  // 步骤 4：如果目标改变，完全重置。
  if (!(new_goal == goal_)) {
    reset();
    km_ = 0.0;

    start_ = new_start;
    goal_ = new_goal;
    start_ptr_ = map_[start_.x][start_.y];
    goal_ptr_ = map_[goal_.x][goal_.y];
    last_ptr_ = start_ptr_;

    // D* Lite 反向搜索：goal 的 rhs = 0（搜索源）。
    goal_ptr_->rhs = 0.0;
    goal_ptr_->key = calculateKey(goal_ptr_);
    goal_ptr_->open_it = open_list_.insert(std::make_pair(goal_ptr_->key, goal_ptr_));

    expand_.clear();
    computeShortestPath();

    if (!extractPath(start_, goal_)) {
      fillSearchedPointsDebugInfo(expand_);
      return false;
    }

    *path = path_;
    *expand = expand_;

    std::memcpy(
      last_global_costmap_, curr_global_costmap_,
      static_cast<std::size_t>(map_size_));

    fillSearchedPointsDebugInfo(*expand);
    return true;
  }

  // 步骤 5：目标未变 — 增量更新。
  start_ = new_start;
  start_ptr_ = map_[start_.x][start_.y];

  // 步骤 5a：计算 km 修正项 — 补偿机器人从 last_ptr_ 移动到 start_ptr_ 的距离。
  km_ += getH(last_ptr_, start_ptr_);
  last_ptr_ = start_ptr_;

  expand_.clear();

  // 步骤 5b：在 start 周围的局部窗口内扫描代价地图变化。
  const int cx = start_.x;
  const int cy = start_.y;
  const int half_win = kWindowSize / 2;

  for (int i = std::max(0, cx - half_win); i < std::min(nx_, cx + half_win); ++i) {
    for (int j = std::max(0, cy - half_win); j < std::min(ny_, cy + half_win); ++j) {
      const int idx = grid2Index(i, j);
      if (curr_global_costmap_[idx] != last_global_costmap_[idx]) {
        LNodePtr changed = map_[i][j];
        updateVertex(changed);

        std::vector<LNodePtr> neighbours;
        getNeighbours(changed, neighbours);
        for (auto & nb : neighbours) {
          updateVertex(nb);
        }
      }
    }
  }

  // 步骤 5c：增量重新计算最短路径。
  computeShortestPath();

  if (!extractPath(start_, goal_)) {
    fillSearchedPointsDebugInfo(expand_);
    return false;
  }

  // 步骤 5d：裁剪已走过的路段。
  LNode state = getState(new_start);
  double min_dist = INF;
  int trim_index = 0;
  for (int i = 0; i < static_cast<int>(path_.size()); ++i) {
    double px, py;
    map2World(state.x, state.y, px, py);
    const double dist = std::hypot(path_[i].x - px, path_[i].y - py);
    if (dist < min_dist) {
      min_dist = dist;
      trim_index = i;
    }
  }

  Points3d trimmed_path(path_.begin() + trim_index, path_.end());
  *path = trimmed_path;
  *expand = expand_;

  std::memcpy(
    last_global_costmap_, curr_global_costmap_,
    static_cast<std::size_t>(map_size_));

  fillSearchedPointsDebugInfo(*expand);
  return true;
}

}  // namespace rmp::path_planner
