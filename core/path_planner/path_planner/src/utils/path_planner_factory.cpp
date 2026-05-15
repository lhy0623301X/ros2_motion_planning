/**
 * @file path_planner_factory.cpp
 * @brief Factory shell for phased planner migration.
 */
#include "utils/path_planner_factory.h"

#include "graph_planner/dijkstra_planner.h"

namespace rmp::path_planner {

bool PathPlannerFactory::createPlanner(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  PlannerProps & planner_props)
{
  PathPlannerConfig config;
  planner_props = PlannerProps{};
  planner_props.planner_name = node->declare_parameter<std::string>(
    plugin_name + ".planner_name", "");
  config.default_tolerance = node->declare_parameter<double>(
    plugin_name + ".tolerance", config.default_tolerance);
  config.obstacle_inflation_factor = node->declare_parameter<double>(
    plugin_name + ".obstacle_inflation_factor", config.obstacle_inflation_factor);
  config.obstacle_cost_weight = node->declare_parameter<double>(
    plugin_name + ".obstacle_cost_weight", config.obstacle_cost_weight);
  config.obstacle_sigmoid_alpha = node->declare_parameter<double>(
    plugin_name + ".obstacle_sigmoid_alpha", config.obstacle_sigmoid_alpha);
  config.obstacle_sigmoid_center = node->declare_parameter<double>(
    plugin_name + ".obstacle_sigmoid_center", config.obstacle_sigmoid_center);
  config.replanning_distance = node->declare_parameter<double>(
    plugin_name + ".replanning_distance", config.replanning_distance);
  config.enable_path_reuse = node->declare_parameter<bool>(
    plugin_name + ".enable_path_reuse", config.enable_path_reuse);
  config.enable_debug_visualization = node->declare_parameter<bool>(
    "enable_debug_visualization", config.enable_debug_visualization);
  config.outline_map = node->declare_parameter<bool>(
    plugin_name + ".outline_map", config.outline_map);

  if (planner_props.planner_name == "dijkstra") {
    planner_props.planner_ptr = std::make_shared<DijkstraPathPlanner>(std::move(costmap_ros));
    planner_props.planner_ptr->setConfig(config);
    planner_props.planner_type = kGraphPlanner;
    RCLCPP_INFO(node->get_logger(), "Using migrated planner '%s'.", planner_props.planner_name.c_str());
    return true;
  }

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
