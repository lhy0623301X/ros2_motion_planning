/**
 * @file path_planner_factory.cpp
 * @brief Factory shell for phased planner migration.
 */
#include "utils/path_planner_factory.h"

#include "graph_planner/astar_planner.h"
#include "graph_planner/dijkstra_planner.h"
#include "graph_planner/dstar_planner.h"
#include "graph_planner/dstar_lite_planner.h"
#include "graph_planner/gbfs_planner.h"
#include "graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"
#include "graph_planner/jps_planner.h"
#include "graph_planner/lpa_star_planner.h"

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
  config.planning_safety_margin = node->declare_parameter<double>(
    plugin_name + ".planning_safety_margin", config.planning_safety_margin);
  config.replanning_distance = node->declare_parameter<double>(
    plugin_name + ".replanning_distance", config.replanning_distance);
  config.goal_reuse_tolerance = node->declare_parameter<double>(
    plugin_name + ".goal_reuse_tolerance", config.goal_reuse_tolerance);
  config.enable_path_reuse = node->declare_parameter<bool>(
    plugin_name + ".enable_path_reuse", config.enable_path_reuse);
  config.enable_path_smoother = node->declare_parameter<bool>(
    plugin_name + ".enable_path_smoother", config.enable_path_smoother);
  config.path_smoother_type = node->declare_parameter<std::string>(
    plugin_name + ".path_smoother_type", config.path_smoother_type);
  config.path_smoother_downsample_factor = node->declare_parameter<double>(
    plugin_name + ".path_smoother_downsample_factor", config.path_smoother_downsample_factor);
  config.enable_debug_visualization = node->declare_parameter<bool>(
    "enable_debug_visualization", config.enable_debug_visualization);
  config.enable_log = node->declare_parameter<bool>(
    plugin_name + ".enable_log", config.enable_log);
  config.outline_map = node->declare_parameter<bool>(
    plugin_name + ".outline_map", config.outline_map);

  std::shared_ptr<PathPlanner> planner;

  if (planner_props.planner_name == "dijkstra") {
    planner = std::make_shared<DijkstraPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "A*") {
    planner = std::make_shared<AStarPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "GBFS") {
    planner = std::make_shared<GBFSPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "JPS") {
    planner = std::make_shared<JPSPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "D*") {
    planner = std::make_shared<DStarPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "D* Lite") {
    planner = std::make_shared<DStarLitePathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "LPA*") {
    planner = std::make_shared<LPAStarPathPlanner>(costmap_ros);
  } else if (planner_props.planner_name == "hybrid A*") {
    auto hybrid = std::make_shared<HybridAStarPathPlanner>(costmap_ros);
    HybridAStarConfig hybrid_config;
    hybrid_config.dim_3_size = node->declare_parameter<int>(
      plugin_name + ".hybrid_astar.dim_3_size", hybrid_config.dim_3_size);
    hybrid_config.max_iterations = node->declare_parameter<int>(
      plugin_name + ".hybrid_astar.max_iterations", hybrid_config.max_iterations);
    hybrid_config.max_approach_iterations = node->declare_parameter<int>(
      plugin_name + ".hybrid_astar.max_approach_iterations",
      hybrid_config.max_approach_iterations);
    hybrid_config.goal_tolerance = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.goal_tolerance", hybrid_config.goal_tolerance);
    hybrid_config.minimum_turning_radius = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.minimum_turning_radius",
      hybrid_config.minimum_turning_radius);
    hybrid_config.curve_sample_ratio = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.curve_sample_ratio", hybrid_config.curve_sample_ratio);
    hybrid_config.non_straight_penalty = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.non_straight_penalty", hybrid_config.non_straight_penalty);
    hybrid_config.change_penalty = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.change_penalty", hybrid_config.change_penalty);
    hybrid_config.reverse_penalty = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.reverse_penalty", hybrid_config.reverse_penalty);
    hybrid_config.retrospective_penalty = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.retrospective_penalty",
      hybrid_config.retrospective_penalty);
    hybrid_config.analytic_expansion_max_length = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.analytic_expansion_max_length",
      hybrid_config.analytic_expansion_max_length);
    hybrid_config.lambda_h = node->declare_parameter<double>(
      plugin_name + ".hybrid_astar.lambda_h", hybrid_config.lambda_h);
    hybrid_config.default_graph_size = node->declare_parameter<int>(
      plugin_name + ".hybrid_astar.default_graph_size", hybrid_config.default_graph_size);
    hybrid->setHybridConfig(hybrid_config);
    planner = hybrid;
  }

  if (planner) {
    planner->setConfig(config);
    planner_props.planner_ptr = planner;
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
