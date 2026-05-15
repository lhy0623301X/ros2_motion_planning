/**
 * @file path_planner_node.h
 * @brief Minimal Nav2 global planner plugin shell for phased migration.
 */
#ifndef RMP_PATH_PLANNER_PATH_PLANNER_NODE_H_
#define RMP_PATH_PLANNER_PATH_PLANNER_NODE_H_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "path_planner.h"
#include "utils/path_planner_factory.h"

namespace rmp::path_planner {

class PathPlannerNode : public nav2_core::GlobalPlanner
{
public:
  PathPlannerNode() = default;
  ~PathPlannerNode() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav_msgs::msg::Path makeEmptyPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<PathPlanner> planner_;
  std::string name_;
  std::string global_frame_;
  PlannerType planner_type_{kNoPlanner};
  rclcpp::Logger logger_{rclcpp::get_logger("path_planner")};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_PATH_PLANNER_NODE_H_
