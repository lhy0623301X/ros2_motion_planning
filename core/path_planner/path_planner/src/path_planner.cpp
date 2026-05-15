/**
 * @file path_planner.cpp
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#include "path_planner.h"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace rmp::path_planner {

PathPlanner::PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(std::move(costmap_ros)),
  costmap_(costmap_ros_ ? costmap_ros_->getCostmap() : nullptr)
{
}

nav_msgs::msg::Path PathPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  Points3d path;
  Points3d expand;
  const bool found = plan(
    {start.pose.position.x, start.pose.position.y, 0.0},
    {goal.pose.position.x, goal.pose.position.y, 0.0},
    &path, &expand);

  if (!found) {
    return nav_msgs::msg::Path{};
  }

  return toNavPath(path, goal.header.frame_id, goal.header.stamp);
}

const PathPlannerConfig & PathPlanner::config() const
{
  return config_;
}

void PathPlanner::setConfig(const PathPlannerConfig & config)
{
  config_ = config;
}

nav2_costmap_2d::Costmap2D * PathPlanner::getCostMap() const
{
  return costmap_;
}

int PathPlanner::getMapSize() const
{
  return getSizeInCellsX() * getSizeInCellsY();
}

int PathPlanner::grid2Index(int x, int y) const
{
  return x + getSizeInCellsX() * y;
}

void PathPlanner::index2Grid(int i, int & x, int & y) const
{
  const int size_x = getSizeInCellsX();
  x = i % size_x;
  y = i / size_x;
}

bool PathPlanner::world2Map(double wx, double wy, double & mx, double & my) const
{
  auto logger = rclcpp::get_logger("path_planner");

  if (!costmap_) {
    RCLCPP_WARN(logger, "world2Map 失败: costmap_ 为空。输入 world=(%.3f, %.3f)", wx, wy);
    return false;
  }

  RCLCPP_INFO(
    logger,
    "world2Map 输入: world=(%.3f, %.3f), origin=(%.3f, %.3f), resolution=%.3f, size=(%d, %d)",
    wx, wy, costmap_->getOriginX(), costmap_->getOriginY(),
    costmap_->getResolution(), getSizeInCellsX(), getSizeInCellsY());

  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    RCLCPP_WARN(
      logger,
      "world2Map 失败: world 坐标小于 origin。world=(%.3f, %.3f), origin=(%.3f, %.3f)",
      wx, wy, costmap_->getOriginX(), costmap_->getOriginY());
    return false;
  }

  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();
  const bool in_bounds = mx < getSizeInCellsX() && my < getSizeInCellsY();

  RCLCPP_INFO(
    logger,
    "world2Map 结果: map=(%.3f, %.3f), in_bounds=%s",
    mx, my, in_bounds ? "true" : "false");

  if (!in_bounds) {
    RCLCPP_WARN(
      logger,
      "world2Map 失败: map 坐标超出边界。map=(%.3f, %.3f), size=(%d, %d)",
      mx, my, getSizeInCellsX(), getSizeInCellsY());
  }

  return in_bounds;
}

void PathPlanner::map2World(double mx, double my, double & wx, double & wy) const
{
  wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}

void PathPlanner::outlineMap()
{
  if (!costmap_) {
    return;
  }

  const int size_x = getSizeInCellsX();
  const int size_y = getSizeInCellsY();
  auto * pc = costmap_->getCharMap();
  for (int i = 0; i < size_x; ++i) {
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap() + (size_y - 1) * size_x;
  for (int i = 0; i < size_x; ++i) {
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap();
  for (int i = 0; i < size_y; ++i, pc += size_x) {
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap() + size_x - 1;
  for (int i = 0; i < size_y; ++i, pc += size_x) {
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
}

bool PathPlanner::validityCheck(double wx, double wy, double & mx, double & my) const
{
  auto logger = rclcpp::get_logger("path_planner");
  const bool valid = world2Map(wx, wy, mx, my);

  RCLCPP_INFO(
    logger,
    "validityCheck: world=(%.3f, %.3f) -> map=(%.3f, %.3f), valid=%s",
    wx, wy, mx, my, valid ? "true" : "false");

  return valid;
}

nav_msgs::msg::Path PathPlanner::toNavPath(
  const Points3d & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path nav_path;
  nav_path.header.frame_id = frame_id;
  nav_path.header.stamp = stamp;

  for (const auto & pt : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = nav_path.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    nav_path.poses.push_back(pose);
  }

  return nav_path;
}

int PathPlanner::getSizeInCellsX() const
{
  return costmap_ ? static_cast<int>(costmap_->getSizeInCellsX()) : 0;
}

int PathPlanner::getSizeInCellsY() const
{
  return costmap_ ? static_cast<int>(costmap_->getSizeInCellsY()) : 0;
}

}  // namespace rmp::path_planner
