/**
 * @file path_planner.cpp
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#include "path_planner.h"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace rmp::path_planner {

PathPlanner::PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(std::move(costmap_ros)),
  costmap_(costmap_ros_ ? costmap_ros_->getCostmap() : nullptr),
  nx_(costmap_ ? static_cast<int>(costmap_->getSizeInCellsX()) : 0),
  ny_(costmap_ ? static_cast<int>(costmap_->getSizeInCellsY()) : 0),
  map_size_(nx_ * ny_)
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

nav2_costmap_2d::Costmap2D * PathPlanner::getCostMap() const
{
  return costmap_;
}

int PathPlanner::getMapSize() const
{
  return map_size_;
}

int PathPlanner::grid2Index(int x, int y) const
{
  return x + nx_ * y;
}

void PathPlanner::index2Grid(int i, int & x, int & y) const
{
  x = i % nx_;
  y = i / nx_;
}

bool PathPlanner::world2Map(double wx, double wy, double & mx, double & my) const
{
  if (!costmap_ || wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();
  return mx < nx_ && my < ny_;
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

  auto * pc = costmap_->getCharMap();
  for (int i = 0; i < nx_; ++i) {
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap() + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; ++i) {
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny_; ++i, pc += nx_) {
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  pc = costmap_->getCharMap() + nx_ - 1;
  for (int i = 0; i < ny_; ++i, pc += nx_) {
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  }
}

bool PathPlanner::validityCheck(double wx, double wy, double & mx, double & my) const
{
  return world2Map(wx, wy, mx, my);
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

}  // namespace rmp::path_planner
