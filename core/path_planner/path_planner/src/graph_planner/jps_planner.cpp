/**
 * @file jps_planner.cpp
 * @brief JPS（跳点搜索）全局路径规划器 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * JPS 是 A* 的加速变体：在均匀代价网格上，通过"跳跃"跳过对称路径，
 * 仅在强制邻居（forced neighbor）处生成后继节点，大幅减少 open list 规模。
 */
#include "graph_planner/jps_planner.h"

#include "util/log.h"

#include <cmath>
#include <queue>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rmp::path_planner {

JPSPathPlanner::JPSPathPlanner(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: PathPlanner(std::move(costmap_ros))
{
}

void JPSPathPlanner::resetDebugCounters()
{
  blocked_reject_count_ = 0;
  safety_reject_count_ = 0;
  forced_neighbor_safety_reject_count_ = 0;
}

bool JPSPathPlanner::isSafeFreeCellByIndex(int index) const
{
  if (index < 0 || index >= map_size_) {
    return false;
  }

  int x, y;
  index2Grid(index, x, y);
  return isNodeCollisionFree(x, y);
}

Points3d JPSPathPlanner::densifyPathInWorld(const Points3d & jump_points) const
{
  if (jump_points.size() < 2) {
    return jump_points;
  }

  constexpr double kInterpolationStep = 0.05;
  Points3d dense_path;
  dense_path.reserve(jump_points.size() * 4);
  dense_path.push_back(jump_points.front());

  for (std::size_t i = 1; i < jump_points.size(); ++i) {
    const auto & prev = jump_points[i - 1];
    const auto & curr = jump_points[i];
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    const double segment_length = std::hypot(dx, dy);

    if (segment_length <= 1e-6) {
      continue;
    }

    const int num_segments = std::max(1, static_cast<int>(std::ceil(segment_length / kInterpolationStep)));
    for (int step = 1; step <= num_segments; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(num_segments);
      dense_path.push_back({
        prev.x + dx * t,
        prev.y + dy * t,
        std::atan2(dy, dx)
      });
    }
  }

  return dense_path;
}

void JPSPathPlanner::fillSearchedPointsDebugInfo(const Points3d & expand)
{
  auto & searched_points = mutableDebugInfo().searched_points;
  searched_points.clear();
  searched_points.reserve(expand.size());
  for (const auto & pt : expand) {
    double wx, wy;
    map2World(pt.x, pt.y, wx, wy);
    searched_points.push_back({wx, wy, pt.theta});
  }
}

bool JPSPathPlanner::plan(
  const Point3d & start,
  const Point3d & goal,
  Points3d * path,
  Points3d * expand)
{
  auto logger = rclcpp::get_logger("path_planner.jps");

  // 步骤 1：将起点和终点从世界坐标转换到代价地图栅格坐标。
  double start_mx, start_my, goal_mx, goal_my;
  if (!validityCheck(start.x, start.y, start_mx, start_my) ||
      !validityCheck(goal.x, goal.y, goal_mx, goal_my))
  {
    RCLCPP_WARN(
      logger,
      "JPS 规划失败: 起点或终点未通过 validityCheck。start=(%.3f, %.3f), goal=(%.3f, %.3f)",
      start.x, start.y, goal.x, goal.y);
    return false;
  }

  if (config().outline_map) {
    outlineMap();
  }

  // 步骤 2：缓存地图尺寸，并根据 nx_ 计算八方向偏移量与 forced neighbor 检测映射。
  nx_ = getSizeInCellsX();
  ny_ = getSizeInCellsY();
  map_size_ = getMapSize();

  dirs_ = {1, -1, nx_, -nx_, nx_ + 1, -nx_ - 1, nx_ - 1, -nx_ + 1};
  dir_to_obs_id_.clear();
  dir_to_obs_id_ = {
    {-nx_,     {0, 1}},   // bottom   → 检测 left, right
    {nx_,      {0, 1}},   // top      → 检测 left, right
    {-1,       {2, 3}},   // right    → 检测 top, bottom
    {1,        {2, 3}},   // left     → 检测 top, bottom
    {-nx_ - 1, {0, 2}},   // right-bottom → 检测 left, top
    {nx_ + 1,  {1, 3}},   // left-top     → 检测 right, bottom
    {-nx_ + 1, {1, 2}},   // left-bottom  → 检测 right, top
    {nx_ - 1,  {0, 3}},   // right-top    → 检测 left, bottom
  };

  // 步骤 3：构造起点/终点搜索节点。
  start_ = JNode(
    static_cast<int>(start_mx), static_cast<int>(start_my));
  start_.id = grid2Index(start_.x, start_.y);
  goal_ = JNode(
    static_cast<int>(goal_mx), static_cast<int>(goal_my));
  goal_.id = grid2Index(goal_.x, goal_.y);
  resetDebugCounters();

  path->clear();
  expand->clear();

  if (!isNodeCollisionFree(start_.x, start_.y)) {
    RCLCPP_WARN(
      logger,
      "JPS 规划失败: 起点被 planning_safety_margin 过滤。start_grid=(%d, %d), margin=%.3f m",
      start_.x, start_.y, config().planning_safety_margin);
    return false;
  }

  if (!isNodeCollisionFree(goal_.x, goal_.y)) {
    RCLCPP_WARN(
      logger,
      "JPS 规划失败: 终点被 planning_safety_margin 过滤。goal_grid=(%d, %d), margin=%.3f m",
      goal_.x, goal_.y, config().planning_safety_margin);
    return false;
  }

  // 步骤 4：初始化 open list 和 closed list，从起点向四个对角方向发起探测。
  OpenList open_list = OpenList();
  std::unordered_map<int, JNode> closed_list;

  for (int i = 4; i < 6; ++i) {
    checkSlashLine(dirs_[i], start_, open_list);
  }
  for (int i = 6; i < 8; ++i) {
    checkSlashLine(dirs_[i], start_, open_list, false);
  }
  closed_list.insert(std::make_pair(start_.id, start_));

  // 步骤 5：主搜索循环 — 逐个处理 open list 中代价最小的跳点。
  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (closed_list.find(current.id) != closed_list.end()) {
      continue;
    }

    closed_list.insert(std::make_pair(current.id, current));
    expand->push_back(
      Point3d{static_cast<double>(current.x), static_cast<double>(current.y), 0.0});

    // 步骤 6：如果当前节点到达目标，回溯路径并返回。
    if (current == goal_) {
      while (!open_list.empty()) {
        auto n = open_list.top();
        open_list.pop();
        closed_list.insert(std::make_pair(n.id, n));
        expand->push_back(
          Point3d{static_cast<double>(n.x), static_cast<double>(n.y), 0.0});
      }

      const auto backtrace = convertClosedListToPath(closed_list, start_, goal_);
      Points3d jump_points_world;
      jump_points_world.reserve(backtrace.size());
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        double wx, wy;
        map2World(iter->x, iter->y, wx, wy);
        jump_points_world.push_back(Point3d{wx, wy, 0.0});
      }
      *path = densifyPathInWorld(jump_points_world);
      fillSearchedPointsDebugInfo(*expand);
      return true;
    }

    // 步骤 7：从当前跳点出发，沿到达方向和强制邻居方向继续跳跃检测。
    jump(current, open_list);
  }

  // 步骤 8：open list 耗尽，无可行路径。
  fillSearchedPointsDebugInfo(*expand);
  RCLCPP_WARN(
    logger,
    "JPS 搜索失败: open list 耗尽。expanded=%zu, blocked_rejects=%d, safety_rejects=%d, "
    "forced_neighbor_safety_rejects=%d, margin=%.3f m",
    expand->size(),
    blocked_reject_count_,
    safety_reject_count_,
    forced_neighbor_safety_reject_count_,
    config().planning_safety_margin);
  return false;
}

/**
 * @brief 从当前跳点出发，根据到达方向继续探测新的跳点。
 *
 * 先沿原来的到达方向（直线或对角线）检测，
 * 然后如果存在强制邻居（fid != -1），沿强制邻居方向额外发起一次对角线检测。
 */
void JPSPathPlanner::jump(const JNode & node, OpenList & open_list)
{
  // 根据当前节点和父节点位置还原到达方向偏移量。
  auto calcDir = [&](int cur_id, int par_id) -> int {
    int cur_x, cur_y, par_x, par_y;
    index2Grid(cur_id, cur_x, cur_y);
    index2Grid(par_id, par_x, par_y);
    int dx = cur_x - par_x;
    int dy = cur_y - par_y;
    dx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    dy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    return grid2Index(dx, dy);
  };

  int dir = calcDir(node.id, node.parent_id);

  if (dir == 1 || dir == -1 || dir == nx_ || dir == -nx_) {
    checkStraightLine(dir, node, open_list);
  } else {
    checkSlashLine(dir, node, open_list);
  }

  if (node.fid != -1) {
    int f_dir = calcDir(node.fid, node.id);
    checkSlashLine(f_dir, node, open_list, false);
  }
}

/**
 * @brief 在当前位置检测强制邻居。
 *
 * 当沿某方向行进时，如果侧面存在障碍物，而障碍物对角方向是自由的，
 * 则该对角方向位置就是"强制邻居"— 需要产生新的跳点来覆盖此路径。
 */
bool JPSPathPlanner::forceNeighborDetect(
  int dir, int cur_id, std::vector<int> & fn_id)
{
  fn_id.clear();
  const auto it = dir_to_obs_id_.find(dir);
  if (it == dir_to_obs_id_.end()) {
    return false;
  }

  std::array<int, 2> delta_obs = {dirs_[it->second.first], dirs_[it->second.second]};
  const bool current_is_safe = isSafeFreeCellByIndex(cur_id);

  // JPS 安全边界优化：
  // 这里不再仅把“真实障碍物”视为 forced neighbor 的触发来源，
  // 而是把“在 planning_safety_margin 语义下不可安全通行的相邻格”
  // 也等价看作障碍边界。这样可以在不破坏 JPS 跳跃结构的前提下，
  // 让跳点骨架天然远离墙体，而不是等最终路径生成后再被动补救。
  if (!current_is_safe) {
    ++safety_reject_count_;
    return false;
  }

  // 本次优化标记：
  // 直线方向（水平/垂直）下，只要侧面格“不安全”、而对角前方“安全”，
  // 就认为当前位置出现了由安全边界诱导出的 forced neighbor。
  if (dir == 1 || dir == -1 || dir == nx_ || dir == -nx_) {
    for (int i = 0; i < 2; ++i) {
      const int obs_id = cur_id + delta_obs[i];
      const int fn = cur_id + delta_obs[i] + dir;
      if (obs_id >= 0 && obs_id < map_size_ && fn >= 0 && fn < map_size_) {
        const bool obs_safe = isSafeFreeCellByIndex(obs_id);
        const bool fn_safe = isSafeFreeCellByIndex(fn);
        if (!obs_safe && fn_safe) {
          fn_id.push_back(fn);
        } else if (!obs_safe && !fn_safe) {
          ++forced_neighbor_safety_reject_count_;
        }
      }
    }
  } else {
    // 本次优化标记：
    // 对角方向下沿用同样的安全语义，把安全边界造成的不对称性
    // 也当作 forced neighbor 的触发条件。
    for (int i = 0; i < 2; ++i) {
      const int obs_id = cur_id + delta_obs[i];
      const int fn = cur_id + 2 * delta_obs[i] + dir;
      if (obs_id >= 0 && obs_id < map_size_ && fn >= 0 && fn < map_size_) {
        const bool obs_safe = isSafeFreeCellByIndex(obs_id);
        const bool fn_safe = isSafeFreeCellByIndex(fn);
        if (!obs_safe && fn_safe) {
          fn_id.push_back(fn);
        } else if (!obs_safe && !fn_safe) {
          ++forced_neighbor_safety_reject_count_;
        }
      }
    }
  }

  return !fn_id.empty();
}

/**
 * @brief 沿直线方向（上/下/左/右）逐格检测跳点。
 *
 * 逐步前进，遇到以下情况之一时停止：
 *   - 遇到障碍物 → 无跳点
 *   - 到达目标 → 目标即跳点
 *   - 发现强制邻居 → 当前位置即跳点
 */
bool JPSPathPlanner::checkStraightLine(
  int dir, const JNode & node, OpenList & open_list)
{
  int pt = node.id;
  std::vector<int> force_neighbors;
  const auto * char_map = getCostMap()->getCharMap();
  const double lethal = nav2_costmap_2d::LETHAL_OBSTACLE * config().obstacle_inflation_factor;

  while (true) {
    pt += dir;

    if (pt < 0 || pt >= map_size_) {
      return false;
    }

    if (char_map[pt] >= lethal) {
      ++blocked_reject_count_;
      return false;
    }

    int pt_x, pt_y;
    index2Grid(pt, pt_x, pt_y);

    // 直线跳跃过程中，如果当前位置已经贴近障碍物到不满足额外安全边界，
    // 则不把它继续作为候选跳点使用。
    if (!isNodeCollisionFree(pt_x, pt_y)) {
      ++safety_reject_count_;
      return false;
    }

    if (pt == goal_.id) {
      open_list.emplace(
        goal_.x, goal_.y,
        node.g + std::hypot(node.x - goal_.x, node.y - goal_.y),
        0.0, pt, node.id, -1);
      return true;
    }

    if (forceNeighborDetect(dir, pt, force_neighbors)) {
      for (int fn : force_neighbors) {
        if (fn < 0 || fn >= map_size_) {
          continue;
        }
        open_list.emplace(
          pt_x, pt_y,
          node.g + std::hypot(node.x - pt_x, node.y - pt_y),
          std::hypot(pt_x - goal_.x, pt_y - goal_.y),
          pt, node.id, fn);
      }
      return true;
    }
  }
}

/**
 * @brief 沿对角方向检测跳点，同时在每个对角步内递归检查水平和垂直方向。
 *
 * 对角线搜索的核心思路：
 *   1. 如果 from_cur == true，先从当前节点沿水平和垂直方向各做一次直线检测。
 *   2. 然后逐步沿对角方向前进，在每一步中：
 *      - 遇到障碍则停止
 *      - 到达目标则记录跳点
 *      - 发现强制邻居则记录跳点
 *      - 否则从新位置沿水平和垂直方向做直线检测，若找到跳点则当前位置也是跳点
 */
bool JPSPathPlanner::checkSlashLine(
  int dir, const JNode & node, OpenList & open_list, bool from_cur)
{
  // 将对角方向分解为水平分量和垂直分量。
  auto dirDecompose = [&](int d, int & x_dir, int & y_dir) {
    if (std::abs(d) == 1) {
      x_dir = d;
      y_dir = 0;
    } else if (std::abs(d) == nx_) {
      x_dir = 0;
      y_dir = d;
    } else if (d > 0) {
      x_dir = d > nx_ ? 1 : -1;
      y_dir = nx_;
    } else {
      x_dir = std::abs(d) > nx_ ? -1 : 1;
      y_dir = -nx_;
    }
  };

  int x_dir, y_dir;
  bool find_jp = false;
  dirDecompose(dir, x_dir, y_dir);

  if (from_cur) {
    if (checkStraightLine(x_dir, node, open_list) ||
        checkStraightLine(y_dir, node, open_list))
    {
      find_jp = true;
    }
  }

  int pt = node.id;
  std::vector<int> force_neighbors;
  const auto * char_map = getCostMap()->getCharMap();
  const double lethal = nav2_costmap_2d::LETHAL_OBSTACLE * config().obstacle_inflation_factor;

  while (true) {
    pt += dir;

    if (pt < 0 || pt >= map_size_) {
      return find_jp;
    }

    if (char_map[pt] >= lethal) {
      ++blocked_reject_count_;
      return find_jp;
    }

    int pt_x, pt_y;
    index2Grid(pt, pt_x, pt_y);

    // 对角跳跃过程中，同样要求当前位置满足额外安全边界。
    if (!isNodeCollisionFree(pt_x, pt_y)) {
      ++safety_reject_count_;
      return find_jp;
    }

    if (pt == goal_.id) {
      open_list.emplace(
        goal_.x, goal_.y,
        node.g + std::hypot(node.x - goal_.x, node.y - goal_.y),
        0.0, pt, node.id, -1);
      return true;
    }

    if (forceNeighborDetect(dir, pt, force_neighbors)) {
      for (int fn : force_neighbors) {
        if (fn < 0 || fn >= map_size_) {
          continue;
        }
        open_list.emplace(
          pt_x, pt_y,
          node.g + std::hypot(node.x - pt_x, node.y - pt_y),
          std::hypot(pt_x - goal_.x, pt_y - goal_.y),
          pt, node.id, fn);
      }
      return true;
    }

    JNode temp(
      pt_x, pt_y,
      node.g + std::hypot(node.x - pt_x, node.y - pt_y),
      std::hypot(pt_x - goal_.x, pt_y - goal_.y),
      pt, node.id, -1);

    if (checkStraightLine(x_dir, temp, open_list) ||
        checkStraightLine(y_dir, temp, open_list))
    {
      if (!(temp.id < 0 || temp.id >= map_size_)) {
        open_list.emplace(temp);
      }
      return true;
    }
  }
}

}  // namespace rmp::path_planner
