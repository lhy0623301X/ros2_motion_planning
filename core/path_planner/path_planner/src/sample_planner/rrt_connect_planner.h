/**
 * @file rrt_connect_planner.h
 * @brief RRT-Connect 双向采样规划器，从 ROS1 版本迁移到 ROS2 Nav2 插件框架。
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_CONNECT_PLANNER_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_CONNECT_PLANNER_H_

#include "sample_planner/rrt_planner.h"

namespace rmp::path_planner {

/**
 * @brief RRT-Connect 路径规划器。
 *
 * 算法同时维护两棵树：一棵从起点扩展，一棵从目标扩展。每轮先扩展起点树，
 * 再让目标树朝新节点贪婪推进，直到碰撞、停滞或两棵树连接。
 */
class RRTConnectPathPlanner : public RRTPathPlanner
{
public:
  explicit RRTConnectPathPlanner(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

private:
  std::vector<Node> traceTreePath(
    const std::unordered_map<int, Node> & tree,
    const Node & root,
    const Node & leaf) const;

  void fillBiTreeVisualization(
    const std::unordered_map<int, Node> & start_tree,
    const std::unordered_map<int, Node> & goal_tree,
    const Points3d & expand);
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_CONNECT_PLANNER_H_
