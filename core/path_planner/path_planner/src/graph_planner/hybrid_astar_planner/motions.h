/**
 * @file motions.h
 * @brief Hybrid A* 运动模型相关类型定义 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * 包含转向方向枚举 TurnDirection 和运动原语位姿 MotionPose。
 */
#ifndef RMP_PATH_PLANNER_HYBRID_ASTAR_MOTIONS_H_
#define RMP_PATH_PLANNER_HYBRID_ASTAR_MOTIONS_H_

#include <vector>

namespace rmp::path_planner {

/**
 * @brief 运动原语的转向方向。
 *
 * Dubins 曲线只使用 FORWARD / LEFT / RIGHT 三种；
 * Reeds-Shepp 曲线额外使用 REVERSE / REV_LEFT / REV_RIGHT。
 */
enum class TurnDirection {
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT    = 2,
  RIGHT   = 3,
  REVERSE = 4,
  REV_LEFT  = 5,
  REV_RIGHT = 6
};

/**
 * @brief 运动原语位姿：记录一步运动的 (dx, dy, dtheta) 以及转向方向。
 */
class MotionPose {
public:
  MotionPose() = default;

  MotionPose(double x, double y, double theta, TurnDirection turn_dir)
  : x_(x), y_(y), theta_(theta), turn_dir_(turn_dir)
  {
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

#endif  // RMP_PATH_PLANNER_HYBRID_ASTAR_MOTIONS_H_
