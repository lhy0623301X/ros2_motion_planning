/**
 * @file motions.h
 * @brief Motion primitive types used by Hybrid A*.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTIONS_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTIONS_H_

#include <vector>

namespace rmp::path_planner {

enum class TurnDirection
{
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

class MotionPose
{
public:
  MotionPose() = default;

  MotionPose(double x, double y, double theta, TurnDirection turn_dir)
  : x_(x), y_(y), theta_(theta), turn_dir_(turn_dir)
  {
  }

  MotionPose operator-(const MotionPose & other) const
  {
    return MotionPose(
      x_ - other.x_, y_ - other.y_, theta_ - other.theta_, TurnDirection::UNKNOWN);
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  TurnDirection turn_dir() const { return turn_dir_; }

private:
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  TurnDirection turn_dir_{TurnDirection::UNKNOWN};
};

using MotionPoses = std::vector<MotionPose>;

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTIONS_H_
