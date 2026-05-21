/**
 * @file dwa_visualizer.cpp
 * @brief RViz visualization helper for DWA sampled and selected trajectories.
 *
 * Visual conventions:
 *   - sampled trajectories: grey thin lines
 *   - best trajectory:      red thick line
 */
#include "dwa_controller/dwa_visualizer.h"

#include <cstddef>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rmp::controller {

namespace {

geometry_msgs::msg::Point toGeometryPoint(const DWATrajectoryPoint & point, double z)
{
  geometry_msgs::msg::Point geometry_point;
  geometry_point.x = point.x;
  geometry_point.y = point.y;
  geometry_point.z = z;
  return geometry_point;
}

}  // namespace

DWAVisualizer::DWAVisualizer(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & topic_name)
{
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, 1);
}

void DWAVisualizer::publish(
  const std::vector<DWATrajectory> & sampled_trajectories,
  const DWATrajectory & best_trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(
    makeSampledTrajectoriesMarker(sampled_trajectories, frame_id, stamp));
  marker_array.markers.push_back(makeBestTrajectoryMarker(best_trajectory, frame_id, stamp));
  marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker DWAVisualizer::makeSampledTrajectoriesMarker(
  const std::vector<DWATrajectory> & sampled_trajectories,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "dwa_sampled_trajectories";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.015;
  // 候选轨迹用低饱和灰色，只表达采样分布，不抢最优轨迹的视觉焦点。
  marker.color.r = 0.55F;
  marker.color.g = 0.55F;
  marker.color.b = 0.55F;
  marker.color.a = 0.45F;

  bool has_segments = false;
  for (const auto & trajectory : sampled_trajectories) {
    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
      marker.points.push_back(toGeometryPoint(trajectory.points[i - 1], 0.02));
      marker.points.push_back(toGeometryPoint(trajectory.points[i], 0.02));
      has_segments = true;
    }
  }

  marker.action = has_segments ?
    visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  return marker;
}

visualization_msgs::msg::Marker DWAVisualizer::makeBestTrajectoryMarker(
  const DWATrajectory & best_trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "dwa_best_trajectory";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.020;
  // 最优轨迹用红色粗线，便于在 RViz 中一眼看到 DWA 最终选择。
  marker.color.r = 1.0F;
  marker.color.g = 0.0F;
  marker.color.b = 0.0F;
  marker.color.a = 0.95F;

  if (!best_trajectory.legal || best_trajectory.points.size() < 2) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & point : best_trajectory.points) {
    marker.points.push_back(toGeometryPoint(point, 0.08));
  }
  return marker;
}

}  // namespace rmp::controller
