/**
 * @file teb_types.h
 * @brief Shared data types used by the TEB controller modules.
 */
#ifndef RMP_CONTROLLER_TEB_TYPES_H_
#define RMP_CONTROLLER_TEB_TYPES_H_

#include "geometry_msgs/msg/point.hpp"
#include <vector>

namespace rmp::controller {

struct TEBControllerConfig
{
  // Basic controller limits and update rate.
  double control_frequency{20.0};
  double max_linear_velocity{0.8};
  double min_linear_velocity{0.0};
  double max_linear_velocity_increment{0.02};
  double max_angular_velocity{1.2};
  double min_angular_velocity{0.0};
  double max_angular_velocity_increment{0.2};
  double max_linear_acceleration{0.5};
  double max_angular_acceleration{1.5};

  // Teb band initialization and resizing.
  double dt_ref{0.3};
  double dt_hysteresis{0.1};
  int min_samples{3};
  int max_samples{20};
  double teb_length_scale{1.5};
  double local_goal_distance{3.0};

  // Optimization controls.
  int max_iterations{5};
  int feasibility_check_poses{5};
  double convergence_epsilon{1.0e-3};

  // Cost weights.
  double weight_kinematics{1.0};
  double weight_velocity{1.0};
  double weight_acceleration{1.0};
  double weight_obstacle{10.0};
  double weight_time{1.0};
  double weight_goal{2.0};
  double weight_smoothness{0.8};
  double weight_goal_heading{1.5};

  // Collision and obstacle handling.
  double min_obstacle_distance{0.25};
  double robot_radius{0.18};
  bool unknown_as_obstacle{false};
  bool allow_backward_motion{false};
};

struct TEBState
{
  // 当前机器人状态，由 controller 层在每个控制周期传给 optimizer。
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v{0.0};
  double w{0.0};
};

struct TEBPose
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct TEBTimedPose
{
  // TEB 的基本单元：一个离散位姿 + 到下一个位姿的时间间隔 dt。
  TEBPose pose;
  double dt{0.0};
};

struct TEBTrajectory
{
  // 整条弹性带。第一项通常贴当前机器人位姿，最后一项贴局部目标。
  std::vector<TEBTimedPose> states;
  double total_cost{0.0};
  bool feasible{false};
};

struct TEBCommand
{
  double v{0.0};
  double w{0.0};
};

struct TEBOptimizationSummary
{
  // 供 controller 层做调试和策略判断的优化结果摘要。
  int iterations{0};
  bool initialized{false};
  bool optimized{false};
  bool feasible{false};
  double total_cost{0.0};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_TEB_TYPES_H_
