/**
 * @file lookahead_point_publisher.h
 * @brief RViz marker publisher for controller lookahead points.
 */
#ifndef RMP_CONTROLLER_UTILS_LOOKAHEAD_POINT_PUBLISHER_H_
#define RMP_CONTROLLER_UTILS_LOOKAHEAD_POINT_PUBLISHER_H_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rmp::controller::utils {

class LookaheadPointPublisher
{
public:
  LookaheadPointPublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic_name);

  void publish(
    double x,
    double y,
    double theta,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace rmp::controller::utils

#endif  // RMP_CONTROLLER_UTILS_LOOKAHEAD_POINT_PUBLISHER_H_
