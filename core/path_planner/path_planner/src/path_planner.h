/**
 * @file path_planner.h
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#ifndef RMP_PATH_PLANNER_PATH_PLANNER_H_
#define RMP_PATH_PLANNER_PATH_PLANNER_H_

#include <memory>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "util/planner_visualization.h"

namespace rmp::path_planner {

struct Point3d
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

using Points3d = std::vector<Point3d>;

struct PathPlannerConfig
{
  double default_tolerance{2.0};
  double obstacle_inflation_factor{1.0};
  double obstacle_cost_weight{3.0};
  double obstacle_sigmoid_alpha{10.0};
  double obstacle_sigmoid_center{0.35};
  double planning_safety_margin{0.0};
  double replanning_distance{0.5};
  double goal_reuse_tolerance{0.2};
  bool enable_path_reuse{true};
  bool enable_path_smoother{false};
  std::string path_smoother_type{"BSplineCurve"};
  bool enable_debug_visualization{true};
  bool enable_log{false};
  bool outline_map{false};
};

class PathPlanner
{
public:
  explicit PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  virtual ~PathPlanner() = default;

  virtual bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) = 0;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  const PathPlannerConfig & config() const;
  void setConfig(const PathPlannerConfig & config);
  const common::util::PlannerDebugInfo & debugInfo() const;
  nav2_costmap_2d::Costmap2D * getCostMap() const;
  int getMapSize() const;
  int grid2Index(int x, int y) const;
  void index2Grid(int i, int & x, int & y) const;
  bool world2Map(double wx, double wy, double & mx, double & my) const;
  void map2World(double mx, double my, double & wx, double & wy) const;
  void outlineMap();
  bool validityCheck(double wx, double wy, double & mx, double & my) const;
  bool isNodeCollisionFree(int mx, int my) const;

protected:
  template<typename NodeT>
  std::vector<NodeT> convertClosedListToPath(
    const std::unordered_map<int, NodeT> & closed_list,
    const NodeT & start,
    const NodeT & goal) const
  {
    std::vector<NodeT> path;
    auto current = closed_list.find(goal.id);
    while (current != closed_list.end() && !(current->second == start)) {
      path.emplace_back(current->second.x, current->second.y);
      const auto parent = closed_list.find(current->second.parent_id);
      if (parent == closed_list.end()) {
        return {};
      }
      current = parent;
    }

    if (current == closed_list.end()) {
      return {};
    }

    path.push_back(start);
    return path;
  }

  nav_msgs::msg::Path toNavPath(
    const Points3d & path,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  int getSizeInCellsX() const;
  int getSizeInCellsY() const;
  bool isPathCollisionFree(const Points3d & path) const;
  common::util::PlannerDebugInfo & mutableDebugInfo();
  void clearDebugInfo();

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  PathPlannerConfig config_;
  common::util::PlannerDebugInfo debug_info_;
  Points3d last_path_;
  Point3d last_goal_;
  bool has_last_goal_{false};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_PATH_PLANNER_H_
