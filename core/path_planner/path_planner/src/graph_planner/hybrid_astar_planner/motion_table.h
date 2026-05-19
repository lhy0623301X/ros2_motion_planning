/**
 * @file motion_table.h
 * @brief Motion primitive lookup table for Hybrid A*.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTION_TABLE_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTION_TABLE_H_

#include <memory>
#include <cmath>
#include <utility>
#include <vector>

#include "geometry/curve/curve.h"
#include "graph_planner/hybrid_astar_planner/motions.h"

namespace rmp::path_planner {

class HybridAStarMotionTable
{
public:
  HybridAStarMotionTable() = default;
  ~HybridAStarMotionTable() = default;

  MotionPoses getMotionPrimitives(const rmp::common::geometry::Point3d & pose) const;
  double getAngleFromBin(int bin_idx) const;
  int getOrientationBin(double theta) const;

  void initDubins(
    int num_angle_quantization,
    double min_turning_radius,
    int map_width,
    double curve_sample_ratio,
    double change_penalty,
    double non_straight_penalty,
    double reverse_penalty,
    double retrospective_penalty);

private:
  void initMotionPrimitives(double angle, double d_dist);

public:
  int num_angle_quantization{72};
  double min_turning_radius{1.0};
  int map_width{0};

  double change_penalty{0.0};
  double non_straight_penalty{1.2};
  double reverse_penalty{2.0};
  double travel_distance_reward{1.0};

  double bin_size{2.0 * M_PI / 72.0};
  MotionPoses projections;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<std::pair<double, double>> trig_values;
  std::vector<double> travel_costs;

  std::unique_ptr<rmp::common::geometry::Curve> curve_gen;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTION_TABLE_H_
