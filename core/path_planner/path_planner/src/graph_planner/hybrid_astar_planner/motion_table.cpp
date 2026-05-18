/**
 * @file motion_table.cpp
 * @brief Hybrid A* motion primitive table migrated from ROS1.
 */
#include "graph_planner/hybrid_astar_planner/motion_table.h"

#include <cmath>

#include "common/geometry/curve/dubins_curve.h"

using rmp::common::geometry::DubinsCurve;
using rmp::common::geometry::Point3d;

namespace rmp::path_planner {

MotionPoses HybridAStarMotionTable::getMotionPrimitives(const Point3d & pose) const
{
  MotionPoses projections_query;
  const double x = pose.x();
  const double y = pose.y();
  const int theta_bin = static_cast<int>(pose.theta());

  for (std::size_t i = 0; i < projections.size(); ++i) {
    int new_theta = theta_bin + static_cast<int>(projections[i].theta());
    new_theta =
      (new_theta % num_angle_quantization + num_angle_quantization) % num_angle_quantization;

    const double delta_x = delta_xs[i][theta_bin];
    const double delta_y = delta_ys[i][theta_bin];
    projections_query.emplace_back(
      x + delta_x, y + delta_y, static_cast<double>(new_theta), projections[i].turn_dir());
  }

  return projections_query;
}

double HybridAStarMotionTable::getAngleFromBin(int bin_idx) const
{
  return bin_idx * bin_size;
}

int HybridAStarMotionTable::getOrientationBin(double theta) const
{
  const double local_bin_size = 2.0 * M_PI / num_angle_quantization;
  int orientation_bin = static_cast<int>(std::round(theta / local_bin_size));
  orientation_bin %= num_angle_quantization;
  if (orientation_bin < 0) {
    orientation_bin += num_angle_quantization;
  }
  return orientation_bin;
}

void HybridAStarMotionTable::initDubins(
  int num_angle_quantization_,
  double min_turning_radius_,
  int map_width_,
  double curve_sample_ratio_,
  double change_penalty_,
  double non_straight_penalty_,
  double reverse_penalty_,
  double retrospective_penalty_)
{
  num_angle_quantization = num_angle_quantization_;
  min_turning_radius = min_turning_radius_;
  map_width = map_width_;
  change_penalty = change_penalty_;
  non_straight_penalty = non_straight_penalty_;
  reverse_penalty = reverse_penalty_;
  travel_distance_reward = 1.0 - retrospective_penalty_;
  curve_gen = std::make_unique<DubinsCurve>(curve_sample_ratio_, 1.0 / min_turning_radius);

  double angle = 2.0 * std::asin(std::sqrt(2.0) / (2.0 * min_turning_radius));
  bin_size = 2.0 * M_PI / num_angle_quantization;
  double increments = (angle < bin_size) ? 1.0 : std::ceil(angle / bin_size);
  angle = increments * bin_size;

  const double dx = min_turning_radius * std::sin(angle);
  const double dy = min_turning_radius - (min_turning_radius * std::cos(angle));
  const double dist = std::hypot(dx, dy);
  projections = {
    MotionPose(dist, 0.0, 0.0, TurnDirection::FORWARD),
    MotionPose(dx, dy, increments, TurnDirection::LEFT),
    MotionPose(dx, -dy, -increments, TurnDirection::RIGHT),
  };

  initMotionPrimitives(angle, dist);
}

void HybridAStarMotionTable::initMotionPrimitives(double /* angle */, double d_dist)
{
  trig_values.resize(num_angle_quantization);
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());

  for (std::size_t i = 0; i < projections.size(); ++i) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (int j = 0; j < num_angle_quantization; ++j) {
      const double theta = bin_size * j;
      const double cos_theta = std::cos(theta);
      const double sin_theta = std::sin(theta);

      if (i == 0) {
        trig_values[j] = {cos_theta, sin_theta};
      }

      delta_xs[i][j] = projections[i].x() * cos_theta - projections[i].y() * sin_theta;
      delta_ys[i][j] = projections[i].x() * sin_theta + projections[i].y() * cos_theta;
    }
  }

  travel_costs.resize(projections.size());
  for (std::size_t i = 0; i < projections.size(); ++i) {
    if (projections[i].turn_dir() != TurnDirection::FORWARD &&
      projections[i].turn_dir() != TurnDirection::REVERSE)
    {
      const double arc_angle = projections[i].theta() * bin_size;
      const double turning_radius = 0.5 * d_dist / std::sin(arc_angle * 0.5);
      travel_costs[i] = turning_radius * arc_angle;
    } else {
      travel_costs[i] = d_dist;
    }
  }
}

}  // namespace rmp::path_planner
