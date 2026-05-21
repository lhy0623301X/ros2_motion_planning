/**
 * @file dwa_visualizer.h
 * @brief RViz visualization helper for DWA sampled and selected trajectories.
 */
#ifndef RMP_CONTROLLER_DWA_VISUALIZER_H_
#define RMP_CONTROLLER_DWA_VISUALIZER_H_

#include <string>
#include <vector>

#include "dwa_controller/dwa_types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rmp::controller {

class DWAVisualizer
{
public:
  DWAVisualizer(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic_name);

  void publish(
    const std::vector<DWATrajectory> & sampled_trajectories,
    const DWATrajectory & best_trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

private:
  visualization_msgs::msg::Marker makeSampledTrajectoriesMarker(
    const std::vector<DWATrajectory> & sampled_trajectories,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  visualization_msgs::msg::Marker makeBestTrajectoryMarker(
    const DWATrajectory & best_trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_DWA_VISUALIZER_H_
