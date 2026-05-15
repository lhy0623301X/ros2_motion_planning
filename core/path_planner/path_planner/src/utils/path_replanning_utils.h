/**
 * @file path_replanning_utils.h
 * @brief Utilities for reusing the previous global path before replanning.
 */
#ifndef RMP_PATH_PLANNER_UTILS_PATH_REPLANNING_UTILS_H_
#define RMP_PATH_PLANNER_UTILS_PATH_REPLANNING_UTILS_H_

#include <cstddef>

#include "nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp"
#include "path_planner.h"

namespace rmp::path_planner::utils {

struct ReplanningDecision
{
  bool need_replanning{true};
  std::size_t nearest_index{0};
};

ReplanningDecision shouldReplan(
  const Point3d & current_pose,
  const Points3d & last_path,
  nav2_costmap_2d::Costmap2D * costmap,
  const PathPlannerConfig & config);

}  // namespace rmp::path_planner::utils

#endif  // RMP_PATH_PLANNER_UTILS_PATH_REPLANNING_UTILS_H_
