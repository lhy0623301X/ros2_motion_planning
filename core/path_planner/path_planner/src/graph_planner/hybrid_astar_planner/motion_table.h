/**
 * @file motion_table.h
 * @brief Hybrid A* 运动原语表 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * 预计算 Dubins 曲线的离散运动原语（直行、左转、右转），
 * 包含每个角度 bin 的 delta_x / delta_y 查找表和行驶代价。
 */
#ifndef RMP_PATH_PLANNER_HYBRID_ASTAR_MOTION_TABLE_H_
#define RMP_PATH_PLANNER_HYBRID_ASTAR_MOTION_TABLE_H_

#include <memory>
#include <vector>

#include "common/geometry/curve/curve.h"
#include "graph_planner/hybrid_astar_planner/motions.h"

namespace rmp::path_planner {

class HybridAStarMotionTable {
public:
  HybridAStarMotionTable() = default;
  ~HybridAStarMotionTable() = default;

  /**
   * @brief 根据当前位姿检索所有可用运动原语。
   * @param pose 当前 (x, y, theta_bin) — theta 为角度 bin 索引
   * @return 投影后的候选位姿集合
   */
  MotionPoses getMotionPrimitives(
    const rmp::common::geometry::Point3d & pose) const;

  /**
   * @brief 角度 bin 索引 → 连续弧度值。
   */
  double getAngleFromBin(int bin_idx) const;

  /**
   * @brief 连续弧度值 → 量化角度 bin 索引。
   */
  int getOrientationBin(double theta) const;

  /**
   * @brief 使用 Dubins 曲线初始化运动原语表。
   */
  void initDubins(
    int num_angle_quantization, double min_turning_radius,
    int map_width, double curve_sample_ratio,
    double change_penalty, double non_straight_penalty,
    double reverse_penalty, double retrospective_penalty);

  // ---- 公开成员，供 Planner / NodeHybrid 直接访问 ----

  int num_angle_quantization{72};
  double min_turning_radius{1.0};
  int map_width{0};

  double change_penalty{1.0};
  double non_straight_penalty{1.0};
  double reverse_penalty{1.0};
  double travel_distance_reward{1.0};

  double bin_size{0.0};
  MotionPoses projections;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<std::pair<double, double>> trig_values;
  std::vector<double> travel_costs;

  std::unique_ptr<rmp::common::geometry::Curve> curve_gen;

private:
  void initMotionPrimitives(double angle, double d_dist);
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_HYBRID_ASTAR_MOTION_TABLE_H_
