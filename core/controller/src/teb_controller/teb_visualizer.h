/**
 * @file teb_visualizer.h
 * @brief Visualization helpers for the TEB controller.
 */
#ifndef RMP_CONTROLLER_TEB_VISUALIZER_H_
#define RMP_CONTROLLER_TEB_VISUALIZER_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "teb_controller/teb_types.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rmp::controller {

class TEBVisualizer
{
public:
  explicit TEBVisualizer(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  void publishTrajectory(
    const TEBTrajectory & initial_trajectory,
    const TEBTrajectory & optimized_trajectory,
    const TEBOptimizationSummary & summary,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

private:
  visualization_msgs::msg::Marker makeLineStripMarker(
    const TEBTrajectory & trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    int id,
    const std::string & ns,
    float width,
    float r,
    float g,
    float b,
    float a) const;
  visualization_msgs::msg::Marker makePosePointMarker(
    const TEBTrajectory & trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  visualization_msgs::msg::Marker makeHeadingMarker(
    const TEBTrajectory & trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  visualization_msgs::msg::Marker makeSummaryMarker(
    const TEBTrajectory & trajectory,
    const TEBOptimizationSummary & summary,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_TEB_VISUALIZER_H_
