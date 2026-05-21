/**
 * @file dwa_motion.cpp
 * @brief Motion model, dynamic window, and candidate trajectory generation for DWA.
 */
#include "dwa_controller/dwa_motion.h"

#include <algorithm>
#include <cmath>

namespace rmp::controller::dwa_motion {

namespace {

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double sampleLinear(double min_value, double max_value, int samples, int index)
{
  if (samples <= 1) {
    return 0.5 * (min_value + max_value);
  }
  const double ratio = static_cast<double>(index) / static_cast<double>(samples - 1);
  return min_value + ratio * (max_value - min_value);
}

}  // namespace

DWAState motionModel(
  const DWAState & state,
  const DWACommand & command,
  double dt)
{
  // 差速底盘运动模型：
  // x/y 按当前朝向前进，theta 按角速度积分。
  DWAState next = state;
  next.x += command.v * std::cos(state.theta) * dt;
  next.y += command.v * std::sin(state.theta) * dt;
  next.theta = normalizeAngle(state.theta + command.w * dt);
  next.v = command.v;
  next.w = command.w;
  return next;
}

DynamicWindow calcDynamicWindow(
  const DWAControllerConfig & config,
  const DWAState & state)
{
  // 动态窗口只允许采样“本周期按加速度/增量限制能到达”的速度。
  // 当前配置用 max_*_increment 表示单周期最大速度变化量。
  DynamicWindow window;
  window.min_v = std::max(config.min_linear_velocity, state.v - config.max_linear_velocity_increment);
  window.max_v = std::min(config.max_linear_velocity, state.v + config.max_linear_velocity_increment);
  window.min_w = std::max(-config.max_angular_velocity, state.w - config.max_angular_velocity_increment);
  window.max_w = std::min(config.max_angular_velocity, state.w + config.max_angular_velocity_increment);

  if (window.min_v > window.max_v) {
    std::swap(window.min_v, window.max_v);
  }
  if (window.min_w > window.max_w) {
    std::swap(window.min_w, window.max_w);
  }
  return window;
}

DWATrajectory predictTrajectory(
  const DWAControllerConfig & config,
  const DWAState & initial_state,
  const DWACommand & command)
{
  // 对单个候选速度进行 rollout：整段轨迹都假设执行同一个 v/w。
  DWATrajectory trajectory;
  trajectory.command = command;

  DWAState state = initial_state;
  const double dt = std::max(1.0e-3, config.sim_time_step);
  const double sim_time = std::max(dt, config.sim_time);

  for (double t = 0.0; t <= sim_time; t += dt) {
    state = motionModel(state, command, dt);
    trajectory.points.push_back({state.x, state.y, state.theta, t + dt});
  }

  return trajectory;
}

std::vector<DWATrajectory> generateTrajectorySamples(
  const DWAControllerConfig & config,
  const DWAState & initial_state,
  const DynamicWindow & window)
{
  // 采样与轨迹生成放在一起：每拿到一个 v/w 样本，立即生成对应预测轨迹。
  std::vector<DWATrajectory> trajectories;
  const int vx_samples = std::max(1, config.vx_samples);
  const int vtheta_samples = std::max(1, config.vtheta_samples);
  trajectories.reserve(static_cast<std::size_t>(vx_samples * vtheta_samples));

  for (int i = 0; i < vx_samples; ++i) {
    for (int j = 0; j < vtheta_samples; ++j) {
      DWACommand command;
      command.v = clamp(
        sampleLinear(window.min_v, window.max_v, vx_samples, i),
        config.min_linear_velocity,
        config.max_linear_velocity);
      command.w = clamp(
        sampleLinear(window.min_w, window.max_w, vtheta_samples, j),
        -config.max_angular_velocity,
        config.max_angular_velocity);
      trajectories.push_back(predictTrajectory(config, initial_state, command));
    }
  }
  return trajectories;
}

}  // namespace rmp::controller::dwa_motion
