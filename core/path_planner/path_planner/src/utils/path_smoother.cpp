/**
 * @file path_smoother.cpp
 * @brief Curve based path smoothing helpers for path_planner paths.
 */
#include "utils/path_smoother.h"

#include <algorithm>
#include <cmath>

#include "common/geometry/curve/bezier_curve.h"
#include "common/geometry/curve/bspline_curve.h"
#include "common/geometry/curve/cubic_spline_curve.h"

namespace rmp::path_planner::utils {

namespace {

common::geometry::Points3d toCommonPath(const Points3d & path)
{
  common::geometry::Points3d common_path;
  common_path.reserve(path.size());
  for (const auto & point : path) {
    common_path.emplace_back(point.x, point.y, point.theta);
  }
  return common_path;
}

Points3d fromCommonPath(const common::geometry::Points3d & path)
{
  Points3d planner_path;
  planner_path.reserve(path.size());
  for (const auto & point : path) {
    planner_path.push_back({point.x(), point.y(), point.theta()});
  }
  return planner_path;
}

double normalizedStep(double step)
{
  return std::max(step, 1e-3);
}

double pathLength(const Points3d & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

bool finishSmoothing(
  const common::geometry::Points3d & common_smoothed_path,
  const Points3d & input_path,
  Points3d & smoothed_path)
{
  if (common_smoothed_path.empty()) {
    smoothed_path = input_path;
    return false;
  }

  smoothed_path = fromCommonPath(common_smoothed_path);
  if (!smoothed_path.empty() && !input_path.empty()) {
    smoothed_path.front() = input_path.front();
    smoothed_path.back() = input_path.back();
  }
  return true;
}

}  // namespace

bool PathSmoother::smooth(
  const Points3d & input_path,
  Points3d & smoothed_path,
  const PathSmootherConfig & config)
{
  if (input_path.size() < 4) {
    smoothed_path = input_path;
    return false;
  }

  switch (config.type) {
    case PathSmootherType::BEZIER:
      return runBezier(input_path, smoothed_path, config);
    case PathSmootherType::BSPLINE:
      return runBSpline(input_path, smoothed_path, config);
    case PathSmootherType::CUBIC_SPLINE:
      return runCubicSpline(input_path, smoothed_path, config);
  }

  smoothed_path = input_path;
  return false;
}

bool PathSmoother::runBezier(
  const Points3d & input_path,
  Points3d & smoothed_path,
  const PathSmootherConfig & config)
{
  common::geometry::BezierCurve smoother(
    normalizedStep(config.step),
    std::max(config.bezier_offset, 1e-3));
  common::geometry::Points3d common_smoothed_path;
  if (!smoother.run(toCommonPath(input_path), common_smoothed_path)) {
    smoothed_path = input_path;
    return false;
  }

  return finishSmoothing(common_smoothed_path, input_path, smoothed_path);
}

bool PathSmoother::runBSpline(
  const Points3d & input_path,
  Points3d & smoothed_path,
  const PathSmootherConfig & config)
{
  // BSplineCurve 的 step 是 [0, 1] 曲线参数步长，不是米制采样间隔。
  // 外部配置仍按米制路径间距理解，这里用路径长度换算为归一化步长，
  // 避免一条长路径只生成十几个点导致控制器难以跟踪。
  const auto desired_spacing = normalizedStep(config.step);
  const auto sample_count = std::max<std::size_t>(
    input_path.size(),
    static_cast<std::size_t>(std::ceil(pathLength(input_path) / desired_spacing)) + 1U);
  const auto bspline_step = 1.0 / static_cast<double>(std::max<std::size_t>(sample_count, 2U));
  common::geometry::BSplineCurve smoother(
    bspline_step,
    std::max(config.bspline_order, 1),
    config.bspline_param_mode,
    config.bspline_mode);
  common::geometry::Points3d common_smoothed_path;
  if (!smoother.run(toCommonPath(input_path), common_smoothed_path)) {
    smoothed_path = input_path;
    return false;
  }

  return finishSmoothing(common_smoothed_path, input_path, smoothed_path);
}

bool PathSmoother::runCubicSpline(
  const Points3d & input_path,
  Points3d & smoothed_path,
  const PathSmootherConfig & config)
{
  common::geometry::CubicSplineCurve smoother(normalizedStep(config.step));
  common::geometry::Points3d common_smoothed_path;
  if (!smoother.run(toCommonPath(input_path), common_smoothed_path)) {
    smoothed_path = input_path;
    return false;
  }

  return finishSmoothing(common_smoothed_path, input_path, smoothed_path);
}

}  // namespace rmp::path_planner::utils
