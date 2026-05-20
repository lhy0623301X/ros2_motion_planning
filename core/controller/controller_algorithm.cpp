/**
 * @file controller_algorithm.cpp
 * @brief Common helpers for controller algorithms.
 */
#include "controller_algorithm.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "util/log.h"

namespace rmp::controller {

geometry_msgs::msg::PoseStamped ControllerAlgorithm::transformPoseToPathFrame(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path,
  const std::shared_ptr<tf2_ros::Buffer> & tf) const
{
  const auto & path_frame = path.header.frame_id;
  if (path_frame.empty() || pose.header.frame_id.empty() || pose.header.frame_id == path_frame) {
    return pose;
  }

  if (!tf) {
    AWARN << "[ControllerAlgorithm] cannot transform robot pose from "
          << pose.header.frame_id << " to " << path_frame
          << ": tf buffer is null. Falling back to the original pose.";
    return pose;
  }

  try {
    return tf->transform(pose, path_frame, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    AWARN << "[ControllerAlgorithm] failed to transform robot pose from "
          << pose.header.frame_id << " to " << path_frame
          << ": " << ex.what()
          << ". Falling back to the original pose; tracking may have frame error.";
    return pose;
  }
}

ControllerAlgorithm::LookaheadPoint ControllerAlgorithm::selectLookaheadPoint(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_dist) const
{
  if (path.poses.empty()) {
    throw std::invalid_argument("Cannot select lookahead point from an empty path.");
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  auto target_it = std::find_if(
    path.poses.begin(), path.poses.end(),
    [&](const auto & plan_pose) {
      return std::hypot(plan_pose.pose.position.x - rx, plan_pose.pose.position.y - ry) >=
             lookahead_dist;
    });

  if (target_it == path.poses.end()) {
    target_it = std::prev(path.poses.end());
  }

  LookaheadPoint target;
  target.x = target_it->pose.position.x;
  target.y = target_it->pose.position.y;

  if (target_it + 1 != path.poses.end()) {
    const auto & next = *(target_it + 1);
    target.theta = std::atan2(
      next.pose.position.y - target.y,
      next.pose.position.x - target.x);
  } else {
    target.theta = tf2::getYaw(target_it->pose.orientation);
    if (!std::isfinite(target.theta)) {
      target.theta = std::atan2(target.y - ry, target.x - rx);
    }
  }

  return target;
}

}  // namespace rmp::controller
