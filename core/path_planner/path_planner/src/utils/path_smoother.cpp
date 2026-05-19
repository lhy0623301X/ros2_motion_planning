/**
 * @file path_smoother.cpp
 * @brief Curve based path smoothing helpers for path_planner paths.
 */
#include "utils/path_smoother.h"

#include <algorithm>
#include <cmath>

#include "geometry/curve/bezier_curve.h"
#include "geometry/curve/bspline_curve.h"
#include "geometry/curve/cubic_spline_curve.h"

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

Points3d downsamplePath(const Points3d & path, double min_spacing)
{
  if (path.size() < 3 || min_spacing <= 1e-6) {
    return path;
  }

  Points3d downsampled_path;
  downsampled_path.reserve(path.size());
  downsampled_path.push_back(path.front());

  double distance_since_last_keep = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    distance_since_last_keep +=
      std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (distance_since_last_keep >= min_spacing) {
      downsampled_path.push_back(path[i]);
      distance_since_last_keep = 0.0;
    }
  }

  downsampled_path.push_back(path.back());
  return downsampled_path;
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

  if (!std::all_of(smoothed_path.begin(), smoothed_path.end(), [](const Point3d & point) {
      return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.theta);
    }))
  {
    smoothed_path = input_path;
    return false;
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

  const double downsample_spacing =
    normalizedStep(config.step) * std::max(1.0, config.downsample_factor);
  const auto smoothing_input = downsamplePath(input_path, downsample_spacing);
  if (smoothing_input.size() < 4) {
    smoothed_path = input_path;
    return false;
  }

  switch (config.type) {
    case PathSmootherType::BEZIER:
      return runBezier(smoothing_input, smoothed_path, config);
    case PathSmootherType::BSPLINE:
      return runBSpline(smoothing_input, smoothed_path, config);
    case PathSmootherType::CUBIC_SPLINE:
      return runCubicSpline(smoothing_input, smoothed_path, config);
  }

  smoothed_path = input_path;
  return false;
}

bool PathSmoother::runBezier(
  const Points3d & input_path,
  Points3d & smoothed_path,
  const PathSmootherConfig & config)
{
  const double step = normalizedStep(config.step);
  const double offset = std::clamp(config.bezier_offset, 1e-3, 1.0);
  common::geometry::BezierCurve smoother(
    step,
    offset);

  common::geometry::Points3d common_smoothed_path;
  common_smoothed_path.reserve(input_path.size());
  for (std::size_t i = 1; i < input_path.size(); ++i) {
    const common::geometry::Point3d start(
      input_path[i - 1].x,
      input_path[i - 1].y,
      input_path[i - 1].theta);
    const common::geometry::Point3d goal(
      input_path[i].x,
      input_path[i].y,
      input_path[i].theta);
    const double distance = std::hypot(goal.x() - start.x(), goal.y() - start.y());
    if (distance < 1e-6) {
      continue;
    }

    const auto control_points = smoother.getControlPoints(start, goal);
    const auto sample_count = std::max<std::size_t>(
      2U,
      static_cast<std::size_t>(std::ceil(distance / step)) + 1U);

    for (std::size_t j = 0; j < sample_count; ++j) {
      if (i > 1 && j == 0) {
        continue;
      }

      const double t = static_cast<double>(j) / static_cast<double>(sample_count - 1U);
      const auto waypoint = smoother.bezier(t, control_points);
      common_smoothed_path.emplace_back(waypoint.x(), waypoint.y(), 0.0);
    }
  }

  if (common_smoothed_path.size() < 2) {
    smoothed_path = input_path;
    return false;
  }

  common_smoothed_path.front().setTheta(input_path.front().theta);
  common_smoothed_path.back().setTheta(input_path.back().theta);
  for (std::size_t i = 1; i + 1 < common_smoothed_path.size(); ++i) {
    const double dx = common_smoothed_path[i + 1].x() - common_smoothed_path[i - 1].x();
    const double dy = common_smoothed_path[i + 1].y() - common_smoothed_path[i - 1].y();
    common_smoothed_path[i].setTheta(std::atan2(dy, dx));
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
