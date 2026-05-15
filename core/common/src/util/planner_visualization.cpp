/**
 * @file planner_visualization.cpp
 * @brief Reusable RViz debug visualization for planning algorithms.
 *
 * Visual conventions:
 *   - searched_points: green cubes (explored nodes in search space)
 *   - sampled_points:  blue cubes (random/methodical samples)
 *   - tree_edges:      grey thin lines (sampling tree connectivity)
 *   - trajectories:    cyan thicker lines (candidate motion trajectories)
 */
#include "util/planner_visualization.h"

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rmp::common::util {

namespace {

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
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, 1);
}

void PlannerVisualizer::publish(
  const PlannerDebugInfo & debug_info,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double resolution)
{
  // 无订阅者时跳过计算，节省资源
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(
    makeSearchedPointsMarker(debug_info.searched_points, frame_id, stamp, resolution));
  marker_array.markers.push_back(
    makeSampledPointsMarker(debug_info.sampled_points, frame_id, stamp, resolution));
  marker_array.markers.push_back(
    makeSampledTreeMarker(debug_info.sampled_tree_edges, frame_id, stamp));
  marker_array.markers.push_back(
    makeSampledTrajectoryMarker(debug_info.sampled_trajectories, frame_id, stamp));
  marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker PlannerVisualizer::makeSearchedPointsMarker(
  const DebugPoints3d & searched_points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double resolution) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "searched_points";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.scale.x = resolution ;   // 略大于栅格，相邻重叠消除网格线
  marker.scale.y = resolution ;
  marker.scale.z = resolution * 0.2;
  // 深绿色半透明 — 探索过的节点，低调不遮挡地图
  marker.color.r = 0.0F;
  marker.color.g = 0.5F;
  marker.color.b = 0.0F;
  marker.color.a = 0.2F;

  if (searched_points.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & point : searched_points) {
      marker.points.push_back(toGeometryPoint(point));
    }
  }

  return marker;
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
  marker.scale.x = resolution ;   // 略大于栅格，相邻重叠消除网格线
  marker.scale.y = resolution ;
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
