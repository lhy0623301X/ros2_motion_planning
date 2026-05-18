/**
 * @file planner_visualization.cpp
 * @brief Reusable RViz debug visualization for planning algorithms.
 *
 * Visual conventions:
 *   - searched_points: occupancy grid overlay (explored cells in search space)
 *   - sampled_points:  blue cubes (random/methodical samples)
 *   - tree_edges:      grey thin lines (sampling tree connectivity)
 *   - trajectories:    cyan thicker lines (candidate motion trajectories)
 */
#include "util/planner_visualization.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rmp::common::util {

namespace {

// 将二维栅格索引打包成一个 64 位 key，用于快速去重。
// 这里按 resolution 量化坐标，保证落在同一 costmap cell 的点只会被记录一次。
std::int64_t makeGridKey(double x, double y, double resolution)
{
  const auto grid_x = static_cast<std::int32_t>(std::llround(x / resolution));
  const auto grid_y = static_cast<std::int32_t>(std::llround(y / resolution));
  return (static_cast<std::int64_t>(grid_x) << 32) |
         static_cast<std::uint32_t>(grid_y);
}

geometry_msgs::msg::Point toGeometryPoint(const DebugPoint3d & point)
{
  geometry_msgs::msg::Point geometry_point;
  geometry_point.x = point.x;
  geometry_point.y = point.y;
  geometry_point.z = 0.0;
  return geometry_point;
}

}  // namespace

PlannerVisualizer::PlannerVisualizer(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & topic_name)
{
  // 图搜索扩展区域单独发布为 OccupancyGrid，便于在 RViz 中以连续栅格层显示。
  searched_points_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name + "/searched_grid", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, 1);
}

void PlannerVisualizer::publish(
  const PlannerDebugInfo & debug_info,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double resolution,
  unsigned int width,
  unsigned int height,
  double origin_x,
  double origin_y)
{
  // 无订阅者时跳过计算，节省资源
  if (marker_pub_->get_subscription_count() == 0 &&
      searched_points_pub_->get_subscription_count() == 0)
  {
    return;
  }

  // searched_points 采用整张 costmap 尺寸的 OccupancyGrid，
  // 行为上对齐 ros_motion_planning 原项目的 expand zone 可视化。
  searched_points_pub_->publish(
    makeSearchedPointsGrid(
      debug_info.searched_points, frame_id, stamp, resolution, width, height, origin_x, origin_y));

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(
    makeSampledPointsMarker(debug_info.sampled_points, frame_id, stamp, resolution));
  marker_array.markers.push_back(makeSampledTreeMarker(debug_info.sampled_tree_edges, frame_id, stamp));
  marker_array.markers.push_back(
    makeSampledTrajectoryMarker(debug_info.sampled_trajectories, frame_id, stamp));
  marker_pub_->publish(marker_array);
}

nav_msgs::msg::OccupancyGrid PlannerVisualizer::makeSearchedPointsGrid(
  const DebugPoints3d & searched_points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double resolution,
  unsigned int width,
  unsigned int height,
  double origin_x,
  double origin_y) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.header.stamp = stamp;
  grid.info.map_load_time = stamp;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width = width;
  grid.info.height = height;
  // OccupancyGrid 的 origin 语义是左下角 cell 的外边界原点，
  // 而 costmap 的 origin 对应 cell 中心，因此这里减去半个 resolution 做对齐。
  grid.info.origin.position.x = origin_x - resolution / 2.0;
  grid.info.origin.position.y = origin_y - resolution / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  if (searched_points.empty() || width == 0 || height == 0) {
    return grid;
  }

  std::unordered_set<std::int64_t> visited_cells;
  visited_cells.reserve(searched_points.size());

  for (const auto & point : searched_points) {
    // searched_points 保存的是世界坐标，这里先转换到当前 costmap 原点下的局部坐标。
    const auto local_x = point.x - origin_x;
    const auto local_y = point.y - origin_y;
    const auto key = makeGridKey(local_x, local_y, resolution);
    if (!visited_cells.insert(key).second) {
      continue;
    }

    // 将世界坐标重新量化为 costmap 栅格索引，并写入整张 OccupancyGrid。
    const auto grid_x = static_cast<int>(std::llround(local_x / resolution));
    const auto grid_y = static_cast<int>(std::llround(local_y / resolution));
    if (grid_x < 0 || grid_y < 0 ||
        grid_x >= static_cast<int>(width) || grid_y >= static_cast<int>(height))
    {
      continue;
    }

    const auto index =
      static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(grid_x);
    grid.data[index] = 50;
  }

  return grid;
}

visualization_msgs::msg::Marker PlannerVisualizer::makeSampledPointsMarker(
  const DebugPoints3d & sampled_points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double resolution) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sampled_points";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  // 采样点仍用 Marker，和 searched grid 在视觉语义上区分开。
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution * 0.2;
  // 蓝色半透明 — 采样点，与搜索节点区分
  marker.color.r = 0.2F;
  marker.color.g = 0.4F;
  marker.color.b = 0.9F;
  marker.color.a = 0.3F;

  if (sampled_points.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & point : sampled_points) {
      marker.points.push_back(toGeometryPoint(point));
    }
  }

  return marker;
}

visualization_msgs::msg::Marker PlannerVisualizer::makeSampledTreeMarker(
  const std::vector<DebugLineSegment> & sampled_tree_edges,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sampled_tree_edges";
  marker.id = 2;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.02;  // 线宽 2cm，比之前的 1cm 更可见
  // 灰白色半透明 — 采样树的边，不喧宾夺主
  marker.color.r = 0.7F;
  marker.color.g = 0.7F;
  marker.color.b = 0.7F;
  marker.color.a = 0.4F;

  if (sampled_tree_edges.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & edge : sampled_tree_edges) {
      marker.points.push_back(toGeometryPoint(edge.start));
      marker.points.push_back(toGeometryPoint(edge.end));
    }
  }

  return marker;
}

visualization_msgs::msg::Marker PlannerVisualizer::makeSampledTrajectoryMarker(
  const std::vector<DebugPoints3d> & sampled_trajectories,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sampled_trajectories";
  marker.id = 3;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.03;  // 轨迹线最粗，因为是最重要的信息
  // 青色 — 轨迹，醒目且有动感
  marker.color.r = 0.0F;
  marker.color.g = 0.8F;
  marker.color.b = 0.9F;
  marker.color.a = 0.6F;

  if (sampled_trajectories.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & trajectory : sampled_trajectories) {
      for (std::size_t i = 1; i < trajectory.size(); ++i) {
        marker.points.push_back(toGeometryPoint(trajectory[i - 1]));
        marker.points.push_back(toGeometryPoint(trajectory[i]));
      }
    }
  }

  return marker;
}

}  // namespace rmp::common::util
