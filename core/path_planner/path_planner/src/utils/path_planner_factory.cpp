/**
 * @file path_planner_factory.cpp
 * @brief Factory shell for phased planner migration.
 */
#include "utils/path_planner_factory.h"

namespace rmp::path_planner {

bool PathPlannerFactory::createPlanner(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /* costmap_ros */,
  PlannerProps & planner_props)
{
  planner_props = PlannerProps{};
  planner_props.planner_name = node->declare_parameter<std::string>(
    plugin_name + ".planner_name", "");

  if (!planner_props.planner_name.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Planner '%s' is not migrated yet. The path_planner package is currently using a "
      "minimal shell without algorithm implementations.",
      planner_props.planner_name.c_str());
  } else {
    RCLCPP_INFO(
      node->get_logger(),
      "No concrete planner is configured for '%s' yet. Keeping the minimal path_planner shell.",
      plugin_name.c_str());
  }

  return false;
}

}  // namespace rmp::path_planner
