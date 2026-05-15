/**
 * @file path_planner.cpp
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#include "path_planner.h"

namespace rmp::path_planner {

PathPlanner::PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(std::move(costmap_ros)),
  costmap_(costmap_ros_ ? costmap_ros_->getCostmap() : nullptr)
{
}

}  // namespace rmp::path_planner
