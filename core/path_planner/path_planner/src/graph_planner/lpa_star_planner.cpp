/**
 * @file lpa_star_planner.cpp
 * @brief LPA* (Lifelong Planning A*) incremental global planner.
 *
 * LPA* 是一种增量搜索算法，在地图代价发生局部变化后，仅重新计算受影响的
 * 节点，避免从头执行完整 A*。与 D* Lite 不同，LPA* 搜索方向为正向
 * (start → goal)，因此当机器人移动（起点改变）时必须完全重置。
 */
#include "graph_planner/lpa_star_planner.h"

#include "common/util/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

LPAStarPathPlanner::LPAStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros)),
  initialized_(false),
  curr_global_costmap_(nullptr),
  last_global_costmap_(nullptr),
  map_(nullptr),
  start_ptr_(nullptr),
  goal_ptr_(nullptr),
  last_ptr_(nullptr),
  nx_(0),
  ny_(0),
  map_size_(0)
{
  start_.x = std::numeric_limits<int>::max();
  start_.y = std::numeric_limits<int>::max();
  goal_.x = std::numeric_limits<int>::max();
  goal_.y = std::numeric_limits<int>::max();
}

LPAStarPathPlanner::~LPAStarPathPlanner()
{
  delete[] curr_global_costmap_;
  delete[] last_global_costmap_;

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
void LPAStarPathPlanner::initMap()
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
void LPAStarPathPlanner::reset()
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
 * @brief 启发式函数：两节点之间的欧几里得距离。
 */
double LPAStarPathPlanner::getH(const LNodePtr n1, const LNodePtr n2) const
{
  return std::hypot(n1->x - n2->x, n1->y - n2->y);
}

/**
 * @brief 计算节点优先级键值。
 *
 * 与 D* Lite 的关键区别：LPA* 正向搜索，因此启发式方向指向 goal
 * （D* Lite 使用到 start 的启发式 + km_ 修正）。
 */
double LPAStarPathPlanner::calculateKey(const LNodePtr s) const
{
  return std::min(s->g, s->rhs) + 0.9 * getH(s, goal_ptr_);
}

/**
 * @brief 检测两相邻节点之间是否存在障碍物碰撞。
 */
bool LPAStarPathPlanner::isCollision(const LNodePtr n1, const LNodePtr n2) const
{
  if (n1->id < 0 || n1->id >= map_size_ ||
      n2->id < 0 || n2->id >= map_size_)
  {
    return true;
  }

  const auto * char_map = getCostMap()->getCharMap();

  if (char_map[n1->id] >= nav2_costmap_2d::LETHAL_OBSTACLE *
      config().obstacle_inflation_factor &&
      char_map[n1->id] >= char_map[n2->id])
  {
    return true;
  }

  if (char_map[n2->id] >= nav2_costmap_2d::LETHAL_OBSTACLE *
      config().obstacle_inflation_factor &&
      char_map[n2->id] >= char_map[n1->id])
  {
    return true;
  }

  return false;
}

/**
 * @brief 获取节点的 8 邻域有效邻居（不越界、不碰撞）。
 */
void LPAStarPathPlanner::getNeighbours(
  const LNodePtr node, std::vector<LNodePtr> & neighbours) const
{
  neighbours.clear();
  static const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
  static const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

  for (int i = 0; i < 8; ++i) {
    const int nx = node->x + dx[i];
    const int ny = node->y + dy[i];
    if (nx < 0 || nx >= nx_ || ny < 0 || ny >= ny_) {
      continue;
    }

    LNodePtr nb = map_[nx][ny];
    if (!isCollision(node, nb)) {
      neighbours.push_back(nb);
    }
  }
}

/**
 * @brief 将 costmap 代价值映射为障碍物邻近惩罚。
 *
 * 复用与 A* / D* / D* Lite 一致的 sigmoid 映射，让 LPA* 在增量重规划时
 * 也能保持“尽量远离障碍物边缘”的路径偏好。
 */
double LPAStarPathPlanner::calculateObstacleCost(unsigned char cell_cost) const
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
double LPAStarPathPlanner::getCost(const LNodePtr n1, const LNodePtr n2) const
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
 * 与 D* Lite 的关键区别：LPA* 跳过 start 节点（start.rhs 始终为 0），
 * 而 D* Lite 跳过 goal 节点。
 */
void LPAStarPathPlanner::updateVertex(LNodePtr u)
{
  // 步骤 1：对非起点节点，从所有前驱中计算最小 rhs 值。
  // 起点的 rhs 固定为 0（搜索源），不参与更新。
  if (u->x != start_.x || u->y != start_.y) {
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
 *   - goal 节点局部一致（g == rhs）且其键值不小于 open list 最小键值
 *   - 或 open list 为空
 *
 * 与 D* Lite 的关键区别：
 *   - 终止条件检查的是 goal（D* Lite 检查 start）
 *   - 初始化时设置 start.rhs = 0（D* Lite 设置 goal.rhs = 0）
 */
void LPAStarPathPlanner::computeShortestPath()
{
  while (!open_list_.empty()) {
    // 步骤 1：取出 open list 中键值最小的节点。
    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->open_it = open_list_.end();

    expand_.push_back(
      Point3d{static_cast<double>(u->x), static_cast<double>(u->y), 0.0});

    // 步骤 2：检查终止条件 — goal 局部一致且当前键值 ≥ goal 键值。
    if (u->key >= calculateKey(goal_ptr_) &&
        goal_ptr_->rhs == goal_ptr_->g)
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

    // 步骤 4：传播变化到所有后继节点。
    std::vector<LNodePtr> neighbours;
    getNeighbours(u, neighbours);
    for (auto & s : neighbours) {
      updateVertex(s);
    }
  }
}

/**
 * @brief 从 goal 回溯到 start 提取路径（贪心选取 g 值最小的邻居）。
 *
 * 与 D* Lite 的关键区别：LPA* 正向搜索后路径从 goal 回溯到 start，
 * 调用方需要在返回前反转路径。
 */
bool LPAStarPathPlanner::extractPath(const LNode & start, const LNode & goal)
{
  Points3d path_temp;
  LNodePtr node_ptr = map_[goal.x][goal.y];
  int count = 0;

  while (!(node_ptr->x == start.x && node_ptr->y == start.y)) {
    double wx, wy;
    map2World(node_ptr->x, node_ptr->y, wx, wy);
    path_temp.push_back(Point3d{wx, wy, 0.0});

    // 在邻居中选取 g 值最小的作为下一步（贪心回溯）。
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

    if (!best || ++count > getMapSize()) {
      return false;
    }
    node_ptr = best;
  }

  double wx, wy;
  map2World(start.x, start.y, wx, wy);
  path_temp.push_back(Point3d{wx, wy, 0.0});

  path_ = path_temp;
  return true;
}

/**
 * @brief 获取给定坐标处的节点状态（从节点网格中查找）。
 */
LPAStarPathPlanner::LNode LPAStarPathPlanner::getState(const LNode & node) const
{
  return *map_[node.x][node.y];
}

void LPAStarPathPlanner::fillSearchedPointsDebugInfo(const Points3d & expand)
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

/**
 * @brief 主规划入口。
 *
 * 首次调用时执行完整初始化和搜索。后续调用如果起点/终点改变则完全重置；
 * 如果仅地图代价变化则增量更新受影响节点后重新计算。
 */
bool LPAStarPathPlanner::plan(
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

  // 步骤 2：延迟初始化 — 首次 plan() 调用时分配地图资源。
  if (!initialized_) {
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

  // 步骤 3：更新当前代价地图快照。
  std::memcpy(
    curr_global_costmap_, getCostMap()->getCharMap(),
    static_cast<std::size_t>(map_size_));

  const LNode new_start(s_x, s_y, INF, INF, grid2Index(s_x, s_y), -1, INF, INF);
  const LNode new_goal(g_x, g_y, INF, INF, grid2Index(g_x, g_y), -1, INF, INF);

  // 步骤 4：判断是否需要完全重置。
  // LPA* 仅能处理固定起点下的边代价增量变化。
  // 起点或终点改变时必须从头开始。
  if (!(new_start == start_) || !(new_goal == goal_)) {
    // 步骤 4a：起点或终点已改变 — 完全重置后执行首次搜索。
    reset();

    start_ = new_start;
    goal_ = new_goal;

    start_ptr_ = map_[start_.x][start_.y];
    goal_ptr_ = map_[goal_.x][goal_.y];

    // LPA* 正向搜索：起点 rhs = 0（搜索源）。
    start_ptr_->rhs = 0.0;

    start_ptr_->key = calculateKey(start_ptr_);
    start_ptr_->open_it = open_list_.insert(std::make_pair(start_ptr_->key, start_ptr_));

    expand_.clear();
    computeShortestPath();

    if (!extractPath(start_, goal_)) {
      fillSearchedPointsDebugInfo(expand_);
      return false;
    }

    // 步骤 4b：路径是从 goal 回溯到 start 的，需要反转。
    std::reverse(path_.begin(), path_.end());
    *path = path_;
    *expand = expand_;

    std::memcpy(
      last_global_costmap_, curr_global_costmap_,
      static_cast<std::size_t>(map_size_));

    fillSearchedPointsDebugInfo(*expand);
    return true;
  }

  // 步骤 5：起点/终点未变 — 检测代价地图变化并增量更新。
  expand_.clear();

  // 步骤 5a：在 start 周围的局部窗口内扫描代价地图变化。
  // 只处理窗口内的变化以提高效率。
  const int cx = start_.x;
  const int cy = start_.y;
  const int half_win = kWindowSize / 2;

  for (int i = std::max(0, cx - half_win); i < std::min(nx_, cx + half_win); ++i) {
    for (int j = std::max(0, cy - half_win); j < std::min(ny_, cy + half_win); ++j) {
      const int idx = grid2Index(i, j);
      if (curr_global_costmap_[idx] != last_global_costmap_[idx]) {
        // 步骤 5b：此栅格代价已变化 — 更新该节点及其所有邻居。
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

  // 步骤 5d：反转路径（goal → start 变为 start → goal）。
  std::reverse(path_.begin(), path_.end());

  // 步骤 5e：找到机器人当前位置在路径上最近的点，裁剪已走过的路段。
  LNode state = getState(new_start);
  last_ptr_ = map_[state.x][state.y];

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
