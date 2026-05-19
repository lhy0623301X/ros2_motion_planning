/**
 * @file informed_rrt_star_planner.h
 * @brief Informed RRT* 采样规划器，从 ROS1 版本迁移到 ROS2 Nav2 插件框架。
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_INFORMED_RRT_STAR_PLANNER_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_INFORMED_RRT_STAR_PLANNER_H_

#include <limits>

#include "sample_planner/rrt_star_planner.h"

namespace rmp::path_planner {

/**
 * @brief Informed RRT* 路径规划器。
 *
 * 在找到第一条可行路径前，行为接近 RRT*；找到可行解后，将采样区域收缩到
 * 起点和终点构成的椭圆内，从而更集中地优化当前解。
 */
class InformedRRTStarPathPlanner : public RRTStarPathPlanner
{
public:
  explicit InformedRRTStarPathPlanner(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

protected:
  Node generateInformedRandomNode();
  Node transformFromUnitBall(double x, double y) const;

  double c_best_{std::numeric_limits<double>::max()};
  double c_min_{0.0};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_SAMPLE_PLANNER_INFORMED_RRT_STAR_PLANNER_H_
