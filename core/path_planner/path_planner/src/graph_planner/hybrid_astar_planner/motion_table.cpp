/**
 * @file motion_table.cpp
 * @brief Hybrid A* 运动原语表实现 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * =====================================================================
 * 运动原语表的作用：
 *   在 Hybrid A* 搜索中，每个节点有 3 种基本运动（直行、左转、右转），
 *   运动原语表为每种运动在每个角度 bin 预计算好 (delta_x, delta_y)，
 *   搜索时直接查表，避免实时三角函数计算，大幅提升搜索速度。
 * =====================================================================
 */
#include "common/geometry/curve/dubins_curve.h"
#include "graph_planner/hybrid_astar_planner/motion_table.h"

using namespace rmp::common::geometry;

namespace rmp::path_planner {

// ===================================================================
// 步骤 1：根据当前位姿查表，生成 3 个候选后继位姿
// ===================================================================
MotionPoses HybridAStarMotionTable::getMotionPrimitives(
  const Point3d & pose) const
{
  MotionPoses projections_query;
  const double x = pose.x();
  const double y = pose.y();
  const int theta_bin = static_cast<int>(pose.theta());

  for (size_t i = 0; i < projections.size(); ++i) {
    // 计算新的角度 bin（考虑环绕）
    int new_theta = theta_bin + static_cast<int>(projections[i].theta());
    new_theta = (new_theta % num_angle_quantization + num_angle_quantization) %
                num_angle_quantization;
    // 查表获取该原语在当前角度下的增量
    const double delta_x = delta_xs[i][theta_bin];
    const double delta_y = delta_ys[i][theta_bin];
    projections_query.emplace_back(
      x + delta_x, y + delta_y,
      static_cast<double>(new_theta),
      projections[i].turn_dir());
  }

  return projections_query;
}

// ===================================================================
// 步骤 2：角度 bin 与弧度之间的互转
// ===================================================================
double HybridAStarMotionTable::getAngleFromBin(int bin_idx) const
{
  return bin_idx * bin_size;
}

int HybridAStarMotionTable::getOrientationBin(double theta) const
{
  const double local_bin_size = 2.0 * M_PI / num_angle_quantization;
  int orientation_bin = static_cast<int>(std::round(theta / local_bin_size));
  orientation_bin %= num_angle_quantization;
  if (orientation_bin < 0) {
    orientation_bin += num_angle_quantization;
  }
  return orientation_bin;
}

// ===================================================================
// 步骤 3：使用 Dubins 曲线几何参数初始化运动原语
// ===================================================================
void HybridAStarMotionTable::initDubins(
  int num_angle_quantization_, double min_turning_radius_, int map_width_,
  double curve_sample_ratio_, double change_penalty_,
  double non_straight_penalty_, double reverse_penalty_,
  double retrospective_penalty_)
{
  num_angle_quantization = num_angle_quantization_;
  min_turning_radius     = min_turning_radius_;
  map_width              = map_width_;
  change_penalty         = change_penalty_;
  non_straight_penalty   = non_straight_penalty_;
  reverse_penalty        = reverse_penalty_;
  travel_distance_reward = 1.0 - retrospective_penalty_;

  // 创建 Dubins 曲线生成器（用于 analytic expansion 和距离启发）
  curve_gen = std::make_unique<DubinsCurve>(
    curve_sample_ratio_, 1.0 / min_turning_radius);

  // 计算最小转弯对应的弧度步进
  double angle = 2.0 * std::asin(std::sqrt(2.0) / (2 * min_turning_radius));
  bin_size = 2.0 * M_PI / num_angle_quantization;
  double increments = (angle < bin_size) ? 1.0 : std::ceil(angle / bin_size);
  angle = increments * bin_size;

  // 3 种基本运动原语：直行、左转、右转
  const double d_x = min_turning_radius * std::sin(angle);
  const double d_y = min_turning_radius - (min_turning_radius * std::cos(angle));
  const double d_dist = std::hypot(d_x, d_y);
  projections = {
    MotionPose(d_dist, 0.0, 0.0, TurnDirection::FORWARD),
    MotionPose(d_x, d_y, increments, TurnDirection::LEFT),
    MotionPose(d_x, -d_y, -increments, TurnDirection::RIGHT)
  };
  initMotionPrimitives(angle, d_dist);
}

// ===================================================================
// 步骤 4：预计算三角函数查找表 + 每个原语的行驶代价
// ===================================================================
void HybridAStarMotionTable::initMotionPrimitives(double angle, double d_dist)
{
  trig_values.resize(num_angle_quantization);
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());

  for (size_t i = 0; i < projections.size(); ++i) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (int j = 0; j < num_angle_quantization; ++j) {
      const double theta = bin_size * j;
      const double cos_theta = std::cos(theta);
      const double sin_theta = std::sin(theta);

      if (i == 0) {
        trig_values[j] = {cos_theta, sin_theta};
      }

      // 将原语局部坐标旋转到全局方向
      delta_xs[i][j] = projections[i].x() * cos_theta -
                        projections[i].y() * sin_theta;
      delta_ys[i][j] = projections[i].x() * sin_theta +
                        projections[i].y() * cos_theta;
    }
  }

  // 计算每种运动的行驶弧长代价
  travel_costs.resize(projections.size());
  for (size_t i = 0; i < projections.size(); ++i) {
    if (projections[i].turn_dir() != TurnDirection::FORWARD &&
        projections[i].turn_dir() != TurnDirection::REVERSE)
    {
      const double arc_angle = projections[i].theta() * bin_size;
      const double turning_rad = 0.5 * d_dist / std::sin(arc_angle * 0.5);
      travel_costs[i] = turning_rad * arc_angle;
    } else {
      travel_costs[i] = d_dist;
    }
  }
}

}  // namespace rmp::path_planner
