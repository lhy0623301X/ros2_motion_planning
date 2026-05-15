/**
 * @file path_planner_factory.h
 * @brief Factory shell for phased planner migration.
 */
#ifndef RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "path_planner.h"

namespace rmp::path_planner {

enum PlannerType
{
  kNoPlanner = 0,
  kGraphPlanner = 1,
};

class PathPlannerFactory
{
public:
  struct PlannerProps
  {
    PlannerType planner_type{kNoPlanner};
    std::string planner_name;
    std::shared_ptr<PathPlanner> planner_ptr;
  };

  static bool createPlanner(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    PlannerProps & planner_props);
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_
