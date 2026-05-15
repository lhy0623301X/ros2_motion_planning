/**
 * @file path_planner.h
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#ifndef RMP_PATH_PLANNER_PATH_PLANNER_H_
#define RMP_PATH_PLANNER_PATH_PLANNER_H_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"

namespace rmp::path_planner {

class PathPlanner
{
public:
  explicit PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  virtual ~PathPlanner() = default;

  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_PATH_PLANNER_H_
