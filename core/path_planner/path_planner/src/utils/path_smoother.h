/**
 * @file path_smoother.h
 * @brief Curve based path smoothing helpers for path_planner paths.
 */
#ifndef RMP_PATH_PLANNER_UTILS_PATH_SMOOTHER_H_
#define RMP_PATH_PLANNER_UTILS_PATH_SMOOTHER_H_

#include "path_planner.h"

namespace rmp::path_planner::utils {

enum class PathSmootherType
{
  BEZIER,
  BSPLINE,
  CUBIC_SPLINE,
};

struct PathSmootherConfig
{
  PathSmootherType type{PathSmootherType::BSPLINE};
  double step{0.1};
  double downsample_factor{4.0};
  double bezier_offset{0.5};
  int bspline_order{3};
  int bspline_param_mode{2};
  int bspline_mode{0};
};

class PathSmoother
{
public:
  static bool smooth(
    const Points3d & input_path,
    Points3d & smoothed_path,
    const PathSmootherConfig & config = {});

private:
  static bool runBezier(
    const Points3d & input_path,
    Points3d & smoothed_path,
    const PathSmootherConfig & config);
  static bool runBSpline(
    const Points3d & input_path,
    Points3d & smoothed_path,
    const PathSmootherConfig & config);
  static bool runCubicSpline(
    const Points3d & input_path,
    Points3d & smoothed_path,
    const PathSmootherConfig & config);
};

}  // namespace rmp::path_planner::utils

#endif  // RMP_PATH_PLANNER_UTILS_PATH_SMOOTHER_H_
