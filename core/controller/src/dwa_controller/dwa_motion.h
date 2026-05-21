/**
 * @file dwa_motion.h
 * @brief Motion model, dynamic window, and candidate trajectory generation for DWA.
 */
#ifndef RMP_CONTROLLER_DWA_MOTION_H_
#define RMP_CONTROLLER_DWA_MOTION_H_

#include <vector>

#include "dwa_controller/dwa_types.h"

namespace rmp::controller::dwa_motion {

// 差速模型一步积分：给定当前状态和采样速度，得到 dt 后的预测状态。
DWAState motionModel(
  const DWAState & state,
  const DWACommand & command,
  double dt);

// 计算动态窗口：当前速度在一个控制周期内可以到达的 v/w 范围。
DynamicWindow calcDynamicWindow(
  const DWAControllerConfig & config,
  const DWAState & state);

// 对一个采样命令做短时前向模拟，生成对应候选轨迹。
DWATrajectory predictTrajectory(
  const DWAControllerConfig & config,
  const DWAState & initial_state,
  const DWACommand & command);

// 在动态窗口内离散采样 v/w，并直接生成所有候选轨迹。
std::vector<DWATrajectory> generateTrajectorySamples(
  const DWAControllerConfig & config,
  const DWAState & initial_state,
  const DynamicWindow & window);

}  // namespace rmp::controller::dwa_motion

#endif  // RMP_CONTROLLER_DWA_MOTION_H_
