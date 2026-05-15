/**
 * @file dstar_planner.cpp
 * @brief D* 增量式全局规划器 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * D* 是经典的增量式路径搜索算法：
 *   1. 首次搜索从目标（goal）向起点（start）执行反向 BFS 扩展
 *   2. 机器人沿路径行进，如果代价地图发生变化，通过 RAISE/LOWER 状态传播
 *      仅修复受影响的局部区域，不必重新搜索整个地图
 *   3. 每个节点有三种标记：NEW（未访问）、OPEN（在开放列表）、CLOSED（已处理）
 */
#include "graph_planner/dstar_planner.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner {

DStarPathPlanner::DStarPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros)),
  initialized_(false),
  curr_global_costmap_(nullptr),
  last_global_costmap_(nullptr),
  map_(nullptr),
  nx_(0), ny_(0), map_size_(0)
{
  goal_.x = std::numeric_limits<int>::max();
  goal_.y = std::numeric_limits<int>::max();
}

DStarPathPlanner::~DStarPathPlanner()
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
 * @brief 初始化节点网格：为每个栅格单元创建 DNode。
 */
void DStarPathPlanner::initMap()
{
  map_ = new DNodePtr *[nx_];
  for (int i = 0; i < nx_; ++i) {
    map_[i] = new DNodePtr[ny_];
    for (int j = 0; j < ny_; ++j) {
      map_[i][j] = new DNode(
        i, j, INF, INF, grid2Index(i, j), -1, DNode::NEW, INF);
    }
  }
}

/**
 * @brief 完全重置搜索状态：清空 open list 并重新分配节点网格。
 */
void DStarPathPlanner::reset()
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
 * @brief 将节点插入或重新插入 open list，同时更新其键值 k。
 *
 * D* 的 insert 规则：
 *   - NEW 节点：k = h_new（首次发现时的代价估计）
 *   - OPEN 节点：k = min(原 k, h_new)（保留更小的优先级）
 *   - CLOSED 节点：k = min(原 h, h_new)（重新激活时取最小代价）
 *
 * @param node_ptr 要插入的节点指针
 * @param h_new 新的 h 代价值
 */
void DStarPathPlanner::insert(DNodePtr node_ptr, double h_new)
{
  if (node_ptr->tag == DNode::NEW) {
    node_ptr->k = h_new;
  } else if (node_ptr->tag == DNode::OPEN) {
    node_ptr->k = std::min(node_ptr->k, h_new);
  } else {
    node_ptr->k = std::min(node_ptr->h, h_new);
  }
  node_ptr->h = h_new;
  node_ptr->tag = DNode::OPEN;
  open_list_.insert(std::make_pair(node_ptr->k, node_ptr));
}

/**
 * @brief 检测两相邻节点之间是否存在障碍物碰撞。
 */
bool DStarPathPlanner::isCollision(DNodePtr n1, DNodePtr n2) const
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
void DStarPathPlanner::getNeighbours(
  DNodePtr node_ptr, std::vector<DNodePtr> & neighbours) const
{
  neighbours.clear();
  static const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
  static const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

  for (int i = 0; i < 8; ++i) {
    int nx = node_ptr->x + dx[i];
    int ny = node_ptr->y + dy[i];
    if (nx < 0 || nx >= nx_ || ny < 0 || ny >= ny_) {
      continue;
    }
    DNodePtr nb = map_[nx][ny];
    if (!isCollision(node_ptr, nb)) {
      neighbours.push_back(nb);
    }
  }
}

/**
 * @brief 计算两相邻节点之间的移动代价（欧几里得距离）。
 */
double DStarPathPlanner::getCost(DNodePtr n1, DNodePtr n2) const
{
  return isCollision(n1, n2) ? INF : std::hypot(n1->x - n2->x, n1->y - n2->y);
}

/**
 * @brief D* 核心函数 — 处理 open list 中键值最小的节点。
 *
 * 这是 D* 区别于 A* 的关键部分：
 *   - 如果 k_old < h（RAISE 状态）：节点代价增大，需要从邻居中寻找更好路径
 *   - 如果 k_old == h（LOWER 状态）：节点代价减小或首次计算，正常传播
 *
 * @return 当前处理的最小 k 值；如果 open list 为空则返回 -1。
 */
double DStarPathPlanner::processState()
{
  if (open_list_.empty()) {
    return -1.0;
  }

  // 步骤 1：取出 open list 中键值最小的节点 X。
  double k_old = open_list_.begin()->first;
  DNodePtr x = open_list_.begin()->second;
  open_list_.erase(open_list_.begin());
  x->tag = DNode::CLOSED;

  expand_.push_back(
    Point3d{static_cast<double>(x->x), static_cast<double>(x->y), 0.0});

  std::vector<DNodePtr> neighbours;
  getNeighbours(x, neighbours);

  // 步骤 2：如果 k_old < h（RAISE 状态） — 节点代价可能因障碍物出现而增大。
  if (k_old < x->h) {
    for (auto & y : neighbours) {
      if (y->tag != DNode::NEW &&
          y->h <= k_old &&
          x->h > y->h + getCost(x, y))
      {
        x->parent_id = y->id;
        x->h = y->h + getCost(x, y);
      }
    }
  }

  // 步骤 3：如果 k_old == h（LOWER 状态） — 正常代价传播。
  if (k_old == x->h) {
    for (auto & y : neighbours) {
      if (y->tag == DNode::NEW ||
          (y->parent_id == x->id && y->h != x->h + getCost(x, y)) ||
          (y->parent_id != x->id && y->h > x->h + getCost(x, y)))
      {
        y->parent_id = x->id;
        insert(y, x->h + getCost(x, y));
      }
    }
  } else {
    // 步骤 4：RAISE 后的特殊传播 — 处理需要降低或重定向的邻居。
    for (auto & y : neighbours) {
      if (y->tag == DNode::NEW ||
          (y->parent_id == x->id && y->h != x->h + getCost(x, y)))
      {
        y->parent_id = x->id;
        insert(y, x->h + getCost(x, y));
      } else if (y->parent_id != x->id && y->h > x->h + getCost(x, y)) {
        insert(x, x->h);
      } else if (y->parent_id != x->id &&
                 x->h > y->h + getCost(x, y) &&
                 y->tag == DNode::CLOSED &&
                 y->h > k_old)
      {
        insert(y, y->h);
      }
    }
  }

  return open_list_.empty() ? -1.0 : open_list_.begin()->first;
}

/**
 * @brief 从起点沿 parent 链回溯到目标，提取路径。
 */
void DStarPathPlanner::extractPath(
  const DNode & start_node, const DNode & goal_node)
{
  path_.clear();
  DNodePtr node_ptr = map_[start_node.x][start_node.y];
  int count = 0;

  while (!(node_ptr->x == goal_node.x && node_ptr->y == goal_node.y)) {
    double wx, wy;
    map2World(node_ptr->x, node_ptr->y, wx, wy);
    path_.push_back(Point3d{wx, wy, 0.0});

    int par_x, par_y;
    index2Grid(node_ptr->parent_id, par_x, par_y);
    if (par_x < 0 || par_x >= nx_ || par_y < 0 || par_y >= ny_ ||
        ++count > map_size_)
    {
      return;
    }
    node_ptr = map_[par_x][par_y];
  }

  double wx, wy;
  map2World(goal_node.x, goal_node.y, wx, wy);
  path_.push_back(Point3d{wx, wy, 0.0});
}

/**
 * @brief 获取给定坐标处的节点状态。
 */
DStarPathPlanner::DNode DStarPathPlanner::getState(const DNode & current) const
{
  return *map_[current.x][current.y];
}

/**
 * @brief 修正因代价地图变化而受影响的节点。
 *
 * D* 的增量修复核心：对代价发生变化的节点 X，
 * 检查其邻居并重新 insert，然后持续调用 processState() 直到传播完成。
 */
void DStarPathPlanner::modify(DNodePtr x)
{
  if (x->tag == DNode::CLOSED) {
    insert(x, x->h);
  }
}

void DStarPathPlanner::fillSearchedPointsDebugInfo(const Points3d & expand_data)
{
  auto & searched_points = mutableDebugInfo().searched_points;
  searched_points.clear();
  searched_points.reserve(expand_data.size());

  for (const auto & point : expand_data) {
    double wx, wy;
    map2World(point.x, point.y, wx, wy);
    searched_points.push_back({wx, wy, point.theta});
  }
}

/**
 * @brief 主规划入口。
 *
 * 首次调用：从目标反向扩展直到到达起点。
 * 后续调用：
 *   - 如果目标改变 → 完全重置
 *   - 如果仅代价地图变化 → 增量修复受影响节点
 */
bool DStarPathPlanner::plan(
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

  const DNode new_goal(g_x, g_y, INF, INF, grid2Index(g_x, g_y));

  // 步骤 4：如果目标改变，则完全重置。
  if (!(new_goal == goal_)) {
    reset();
    goal_ = new_goal;

    DNodePtr goal_ptr = map_[goal_.x][goal_.y];
    goal_ptr->h = 0.0;
    insert(goal_ptr, 0.0);

    DNode start_node(s_x, s_y, INF, INF, grid2Index(s_x, s_y));
    expand_.clear();

    // 步骤 4a：主搜索循环 — 反复处理 open list 直到起点被找到。
    while (true) {
      double min_k = processState();
      if (min_k < 0) {
        fillSearchedPointsDebugInfo(expand_);
        return false;
      }
      DNode state = getState(start_node);
      if (state.tag == DNode::CLOSED) {
        break;
      }
    }

    // 步骤 4b：提取路径（start → goal 方向）。
    extractPath(start_node, goal_);
    *path = path_;
    *expand = expand_;

    std::memcpy(
      last_global_costmap_, curr_global_costmap_,
      static_cast<std::size_t>(map_size_));

    fillSearchedPointsDebugInfo(*expand);
    return !path_.empty();
  }

  // 步骤 5：目标未变 — 检测代价地图变化并增量修复。
  expand_.clear();

  const int half_win = kWindowSize / 2;
  for (int i = std::max(0, s_x - half_win); i < std::min(nx_, s_x + half_win); ++i) {
    for (int j = std::max(0, s_y - half_win); j < std::min(ny_, s_y + half_win); ++j) {
      const int idx = grid2Index(i, j);
      if (curr_global_costmap_[idx] != last_global_costmap_[idx]) {
        // 步骤 5a：标记代价变化的节点和邻居，触发增量传播。
        DNodePtr changed = map_[i][j];
        modify(changed);

        std::vector<DNodePtr> neighbours;
        getNeighbours(changed, neighbours);
        for (auto & nb : neighbours) {
          modify(nb);
        }
      }
    }
  }

  // 步骤 5b：持续处理 open list 直到起点闭合或无解。
  DNode start_node(s_x, s_y, INF, INF, grid2Index(s_x, s_y));

  while (true) {
    double min_k = processState();
    if (min_k < 0) {
      break;
    }
    DNode state = getState(start_node);
    if (state.tag == DNode::CLOSED) {
      break;
    }
  }

  extractPath(start_node, goal_);
  *path = path_;
  *expand = expand_;

  std::memcpy(
    last_global_costmap_, curr_global_costmap_,
    static_cast<std::size_t>(map_size_));

  fillSearchedPointsDebugInfo(*expand);
  return !path_.empty();
}

}  // namespace rmp::path_planner
