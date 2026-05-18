/**
 * @file path_planner.cpp
 * @brief Minimal abstract planner shell used during phased ROS 2 migration.
 */
#include "path_planner.h"

#include <algorithm>
#include <cmath>

#include "common/util/log.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "utils/path_replanning_utils.h"

namespace rmp::path_planner {

namespace {

double calcPlanarDistance(const Point3d & lhs, const Point3d & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace

PathPlanner::PathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(std::move(costmap_ros)),
  costmap_(costmap_ros_ ? costmap_ros_->getCostmap() : nullptr)
{
}

nav_msgs::msg::Path PathPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  clearDebugInfo();
  const Point3d current_start{start.pose.position.x, start.pose.position.y, 0.0};
  const Point3d current_goal{goal.pose.position.x, goal.pose.position.y, 0.0};

  // 步骤 1：如果上一帧路径存在，先判断当前目标是否与上一帧目标一致。
  // 只有目标未明显变化时，旧路径复用才有意义；一旦换了目标，就必须重规划。
  const bool goal_matches_last_plan =
    has_last_goal_ &&
    calcPlanarDistance(current_goal, last_goal_) < config_.goal_reuse_tolerance;

  // 步骤 2：只有在“路径复用开关开启 + 上一帧路径存在 + 目标未变化”时，
  // 才进一步判断能否直接复用上一帧路径。
  if (config_.enable_path_reuse && !last_path_.empty() && goal_matches_last_plan) {
    const auto replanning_decision = utils::shouldReplan(
      current_start,
      last_path_, costmap_, config_);

    // 步骤 3：若当前位置仍贴近上一帧路径，且剩余路径未被阻挡，
    // 就从最近点开始截取剩余路径，直接作为当前规划结果输出。
    if (!replanning_decision.need_replanning) {
      Points3d reused_path(
        last_path_.begin() + static_cast<std::ptrdiff_t>(replanning_decision.nearest_index),
        last_path_.end());
      last_path_ = reused_path;
      return toNavPath(reused_path, goal.header.frame_id, goal.header.stamp);
    }
  }

  // 步骤 4：若不存在可复用路径，则调用具体规划算法重新搜索新路径。
  Points3d path;
  Points3d expand;
  const bool found = plan(
    current_start,
    current_goal,
    &path, &expand);

  if (!found) {
    last_path_.clear();
    has_last_goal_ = false;
    return nav_msgs::msg::Path{};
  }

  // 步骤 5：保存本次新生成的路径和对应目标，供下一帧判断是否需要重规划。
  last_path_ = path;
  last_goal_ = current_goal;
  has_last_goal_ = true;
  return toNavPath(path, goal.header.frame_id, goal.header.stamp);
}

const PathPlannerConfig & PathPlanner::config() const
{
  return config_;
}

void PathPlanner::setConfig(const PathPlannerConfig & config)
{
  config_ = config;
  ::apollo::common::util::SetLogEnabled(config_.enable_log);
}

const common::util::PlannerDebugInfo & PathPlanner::debugInfo() const
{
  return debug_info_;
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

  return valid;
}

bool PathPlanner::isNodeCollisionFree(int mx, int my) const
{
  if (!costmap_) {
    return false;
  }

  if (mx < 0 || my < 0 || mx >= getSizeInCellsX() || my >= getSizeInCellsY()) {
    return false;
  }

  const int margin_cells = static_cast<int>(
    std::ceil(std::max(0.0, config_.planning_safety_margin) / costmap_->getResolution()));
  const double lethal_threshold =
    nav2_costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor;
  const auto * char_map = costmap_->getCharMap();

  for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
    for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
      const int nx = mx + dx;
      const int ny = my + dy;

      if (nx < 0 || ny < 0 || nx >= getSizeInCellsX() || ny >= getSizeInCellsY()) {
        return false;
      }

      const int index = grid2Index(nx, ny);
      if (char_map[index] >= lethal_threshold) {
        return false;
      }
    }
  }

  return true;
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

common::util::PlannerDebugInfo & PathPlanner::mutableDebugInfo()
{
  return debug_info_;
}

void PathPlanner::clearDebugInfo()
{
  debug_info_.clear();
}

}  // namespace rmp::path_planner
