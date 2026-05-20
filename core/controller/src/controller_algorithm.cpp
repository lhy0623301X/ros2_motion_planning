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

double ControllerAlgorithm::limitLinearSpeedByGoalDistance(
  double desired_v,
  double distance_to_goal,
  const GoalSpeedLimitConfig & config) const
{
  if (!config.enabled || distance_to_goal < 0.0) {
    return desired_v;
  }

  const double max_linear_velocity = std::max(0.0, config.max_linear_velocity);
  const double max_decel = std::max(1e-6, config.max_decel);
  const double direction = desired_v < 0.0 ? -1.0 : 1.0;
  const double desired_speed = std::fabs(desired_v);
  const double nominal_slowdown_distance =
    max_linear_velocity * max_linear_velocity / (2.0 * max_decel);
  const double slowdown_distance =
    nominal_slowdown_distance * std::max(1.0, config.brake_distance_scale) +
    std::max(0.0, config.brake_distance_buffer);

  if (distance_to_goal > slowdown_distance) {
    return desired_v;
  }

  const double distance_speed_limit = std::sqrt(2.0 * max_decel * distance_to_goal);
  double limited_speed = std::min(desired_speed, distance_speed_limit);

  if (limited_speed < config.min_linear_velocity) {
    limited_speed = 0.0;
  }

  const double limited_v = direction * limited_speed;
  AINFO_EVERY(20) << "[ControllerAlgorithm] goal speed limit: distance_to_goal="
                  << distance_to_goal
                  << ", slowdown_distance=" << slowdown_distance
                  << ", nominal_slowdown_distance=" << nominal_slowdown_distance
                  << ", desired_v=" << desired_v
                  << ", max_linear_velocity=" << max_linear_velocity
                  << ", distance_speed_limit=" << distance_speed_limit
                  << ", limited_v=" << limited_v;
  return limited_v;
}

}  // namespace rmp::controller
