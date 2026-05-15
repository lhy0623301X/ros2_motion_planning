/**
 * @file path_replanning_utils.cpp
 * @brief Utilities for reusing the previous global path before replanning.
 */
#include "utils/path_replanning_utils.h"

#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace rmp::path_planner::utils {

namespace {

double calcDistance(const Point3d & lhs, const Point3d & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool isPointBlocked(
  const Point3d & point,
  nav2_costmap_2d::Costmap2D * costmap,
  const PathPlannerConfig & config)
{
  if (!costmap) {
    return true;
  }

  if (point.x < costmap->getOriginX() || point.y < costmap->getOriginY()) {
    return true;
  }

  const double mx = (point.x - costmap->getOriginX()) / costmap->getResolution();
  const double my = (point.y - costmap->getOriginY()) / costmap->getResolution();

  if (mx < 0.0 || my < 0.0 ||
    mx >= static_cast<double>(costmap->getSizeInCellsX()) ||
    my >= static_cast<double>(costmap->getSizeInCellsY()))
  {
    return true;
  }

  const int ix = static_cast<int>(mx);
  const int iy = static_cast<int>(my);
  const unsigned int index = static_cast<unsigned int>(ix + iy * costmap->getSizeInCellsX());
  const auto * char_map = costmap->getCharMap();

  return char_map[index] >=
         nav2_costmap_2d::LETHAL_OBSTACLE * config.obstacle_inflation_factor;
}

}  // namespace

ReplanningDecision shouldReplan(
  const Point3d & current_pose,
  const Points3d & last_path,
  nav2_costmap_2d::Costmap2D * costmap,
  const PathPlannerConfig & config)
{
  ReplanningDecision decision;
  if (last_path.empty() || !costmap) {
    return decision;
  }

  // 步骤 1：在上一帧路径上找到距离当前位置最近的路径点。
  double min_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < last_path.size(); ++i) {
    const double distance = calcDistance(current_pose, last_path[i]);
    if (distance < min_distance) {
      min_distance = distance;
      decision.nearest_index = i;
    }
  }

  // 步骤 2：如果当前位置离最近路径点已经偏得较远，则直接触发重规划。
  if (min_distance >= config.replanning_distance) {
    return decision;
  }

  // 步骤 3：从最近路径点往前检查剩余路径是否被阻挡。
  // 只要剩余路径上任一点落入障碍物或 costmap 外，就触发重规划。
  for (std::size_t i = decision.nearest_index; i < last_path.size(); ++i) {
    if (isPointBlocked(last_path[i], costmap, config)) {
      return decision;
    }
  }

  // 步骤 4：若距离足够近且剩余路径畅通，则复用旧路径，不进行重规划。
  decision.need_replanning = false;
  return decision;
}

}  // namespace rmp::path_planner::utils
