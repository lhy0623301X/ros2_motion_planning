/**
 * @file teb_visualizer.cpp
 * @brief Visualization helpers for the TEB controller.
 */
#include "teb_controller/teb_visualizer.h"

#include <iomanip>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/point.hpp"

namespace rmp::controller {

namespace {

geometry_msgs::msg::Point toPoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace

TEBVisualizer::TEBVisualizer(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: node_(node)
{
  // TEB 更适合看“初值 band 与优化后 band 的差异”，因此这里统一发布 marker array。
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "teb_trajectory", rclcpp::QoS(1));
}

void TEBVisualizer::publishTrajectory(
  const TEBTrajectory & initial_trajectory,
  const TEBTrajectory & optimized_trajectory,
  const TEBOptimizationSummary & summary,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(makeLineStripMarker(
    initial_trajectory, frame_id, stamp, 0, "teb_initial", 0.020F, 0.55F, 0.55F, 0.55F, 0.75F));
  marker_array.markers.push_back(makeLineStripMarker(
    optimized_trajectory, frame_id, stamp, 1, "teb_optimized", 0.035F, 1.00F, 0.10F, 0.10F,
    0.95F));
  marker_array.markers.push_back(makePosePointMarker(optimized_trajectory, frame_id, stamp));
  marker_array.markers.push_back(makeHeadingMarker(optimized_trajectory, frame_id, stamp));
  marker_array.markers.push_back(makeSummaryMarker(
    optimized_trajectory, summary, frame_id, stamp));
  marker_pub_->publish(std::move(marker_array));
}

visualization_msgs::msg::Marker TEBVisualizer::makeLineStripMarker(
  const TEBTrajectory & trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  int id,
  const std::string & ns,
  float width,
  float r,
  float g,
  float b,
  float a) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.action = trajectory.states.size() >= 2U ?
    visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;

  for (const auto & state : trajectory.states) {
    marker.points.push_back(toPoint(state.pose.x, state.pose.y, 0.02));
  }
  return marker;
}

visualization_msgs::msg::Marker TEBVisualizer::makePosePointMarker(
  const TEBTrajectory & trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "teb_vertices";
  marker.id = 2;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.10F;
  marker.color.g = 0.55F;
  marker.color.b = 0.95F;
  marker.color.a = 0.85F;
  marker.action = trajectory.states.empty() ?
    visualization_msgs::msg::Marker::DELETE :
    visualization_msgs::msg::Marker::ADD;

  for (const auto & state : trajectory.states) {
    marker.points.push_back(toPoint(state.pose.x, state.pose.y, 0.05));
  }
  return marker;
}

visualization_msgs::msg::Marker TEBVisualizer::makeHeadingMarker(
  const TEBTrajectory & trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "teb_heading";
  marker.id = 3;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.012;
  marker.color.r = 0.15F;
  marker.color.g = 0.35F;
  marker.color.b = 0.95F;
  marker.color.a = 0.85F;
  marker.action = trajectory.states.empty() ?
    visualization_msgs::msg::Marker::DELETE :
    visualization_msgs::msg::Marker::ADD;

  for (const auto & state : trajectory.states) {
    const double arrow_length = 0.12;
    marker.points.push_back(toPoint(state.pose.x, state.pose.y, 0.08));
    marker.points.push_back(toPoint(
      state.pose.x + arrow_length * std::cos(state.pose.theta),
      state.pose.y + arrow_length * std::sin(state.pose.theta),
      0.08));
  }
  return marker;
}

visualization_msgs::msg::Marker TEBVisualizer::makeSummaryMarker(
  const TEBTrajectory & trajectory,
  const TEBOptimizationSummary & summary,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "teb_summary";
  marker.id = 4;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.scale.z = 0.16;
  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.color.a = 0.95F;
  marker.action = trajectory.states.empty() ?
    visualization_msgs::msg::Marker::DELETE :
    visualization_msgs::msg::Marker::ADD;

  if (!trajectory.states.empty()) {
    const auto & tail = trajectory.states.back().pose;
    marker.pose.position.x = tail.x;
    marker.pose.position.y = tail.y;
    marker.pose.position.z = 0.25;
  }

  std::ostringstream text;
  text << "iter=" << summary.iterations
       << " cost=" << std::fixed << std::setprecision(2) << summary.total_cost
       << " feasible=" << (summary.feasible ? "yes" : "no");
  marker.text = text.str();
  return marker;
}

}  // namespace rmp::controller
