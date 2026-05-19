/**
 * @file rrt_star_planner.h
 * @brief RRT* 采样规划器，从 ROS1 版本迁移到 ROS2 Nav2 插件框架。
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_STAR_PLANNER_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_STAR_PLANNER_H_

#include "sample_planner/rrt_planner.h"

namespace rmp::path_planner {

/**
 * @brief RRT*（Rapidly-exploring Random Tree Star）路径规划器。
 *
 * RRT* 在 RRT 的基础上增加邻域重连：
 *   1. 随机采样并从最近节点扩展新节点
 *   2. 在优化半径内寻找更低代价父节点
 *   3. 用新节点反向优化邻域节点的父节点
 *   4. 在采样次数内持续更新到目标的最优路径
 */
class RRTStarPathPlanner : public RRTPathPlanner
{
public:
  explicit RRTStarPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

protected:
  Node findNearestAndRewire(std::unordered_map<int, Node> & tree, const Node & sample);
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_STAR_PLANNER_H_
