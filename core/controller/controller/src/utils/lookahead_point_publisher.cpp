/**
 * @file lookahead_point_publisher.cpp
 * @brief RViz marker publisher for controller lookahead points.
 */
#include "utils/lookahead_point_publisher.h"

#include <cmath>

#include "visualization_msgs/msg/marker.hpp"

namespace rmp::controller::utils {

LookaheadPointPublisher::LookaheadPointPublisher(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & topic_name)
{
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, 1);
}

void LookaheadPointPublisher::publish(
  double x,
  double y,
  double theta,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "lookahead_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.z = std::sin(theta * 0.5);
  marker.pose.orientation.w = std::cos(theta * 0.5);
  marker.scale.x = 0.18;
  marker.scale.y = 0.18;
  marker.scale.z = 0.18;
  marker.color.r = 1.0F;
  marker.color.g = 0.0F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  marker_pub_->publish(marker);
}

}  // namespace rmp::controller::utils
