/**
 * @file planner_visualization.h
 * @brief Reusable RViz debug visualization for planning algorithms.
 */
#ifndef RMP_COMMON_UTIL_PLANNER_VISUALIZATION_H_
#define RMP_COMMON_UTIL_PLANNER_VISUALIZATION_H_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rmp::common::util {

struct DebugPoint3d
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

using DebugPoints3d = std::vector<DebugPoint3d>;

struct DebugLineSegment
{
  DebugPoint3d start;
  DebugPoint3d end;
};

struct PlannerDebugInfo
{
  DebugPoints3d searched_points;
  DebugPoints3d sampled_points;
  std::vector<DebugLineSegment> sampled_tree_edges;
  std::vector<DebugPoints3d> sampled_trajectories;

  void clear()
  {
    searched_points.clear();
    sampled_points.clear();
    sampled_tree_edges.clear();
    sampled_trajectories.clear();
  }
};

class PlannerVisualizer
{
public:
  PlannerVisualizer(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic_name);

  void publish(
    const PlannerDebugInfo & debug_info,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    double resolution);

private:
  visualization_msgs::msg::Marker makeSearchedPointsMarker(
    const DebugPoints3d & searched_points,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    double resolution) const;

  visualization_msgs::msg::Marker makeSampledPointsMarker(
    const DebugPoints3d & sampled_points,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    double resolution) const;

  visualization_msgs::msg::Marker makeSampledTreeMarker(
    const std::vector<DebugLineSegment> & sampled_tree_edges,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  visualization_msgs::msg::Marker makeSampledTrajectoryMarker(
    const std::vector<DebugPoints3d> & sampled_trajectories,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rmp::common::util

#endif  // RMP_COMMON_UTIL_PLANNER_VISUALIZATION_H_
