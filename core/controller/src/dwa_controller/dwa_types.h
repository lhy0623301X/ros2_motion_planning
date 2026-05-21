/**
 * @file dwa_types.h
 * @brief Shared data types used by the DWA controller modules.
 */
#ifndef RMP_CONTROLLER_DWA_TYPES_H_
#define RMP_CONTROLLER_DWA_TYPES_H_

#include <vector>

namespace rmp::controller {

struct DWAControllerConfig
{
  // 控制周期与速度边界。第一版 DWA 只采样差速底盘的 v / w。
  double control_frequency{20.0};
  double max_linear_velocity{0.6};
  double min_linear_velocity{0.0};
  double max_linear_velocity_increment{0.02};
  double max_angular_velocity{1.2};
  double min_angular_velocity{0.0};
  double max_angular_velocity_increment{0.2};

  // 轨迹预测参数：每个候选速度会在 sim_time 时间内按 sim_time_step 前向积分。
  double sim_time{1.5};
  double sim_time_step{0.1};
  int vx_samples{5};
  int vtheta_samples{15};

  // 评分权重：path/goal/obstacle 负责贴路径与避障，velocity/twirling 抑制原地转圈。
  double path_distance_bias{1.0};
  double goal_distance_bias{1.2};
  double obstacle_distance_bias{0.02};
  double velocity_bias{4.0};
  double twirling_bias{0.8};
  bool unknown_as_obstacle{false};
};

struct DWAState
{
  // 世界坐标系下的机器人状态，v/w 是当前实际速度。
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v{0.0};
  double w{0.0};
};

struct DWACommand
{
  double v{0.0};
  double w{0.0};
};

struct DynamicWindow
{
  // 本控制周期内，根据当前速度和增量限制可达的速度采样范围。
  double min_v{0.0};
  double max_v{0.0};
  double min_w{0.0};
  double max_w{0.0};
};

struct DWATrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double t{0.0};
};

struct DWATrajectory
{
  // 一条候选速度对应一条前向预测轨迹，评分后用于选最优命令。
  DWACommand command;
  std::vector<DWATrajectoryPoint> points;
  double score{0.0};
  bool legal{false};
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_DWA_TYPES_H_
