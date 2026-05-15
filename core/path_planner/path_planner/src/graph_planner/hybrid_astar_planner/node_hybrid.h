/**
 * @file node_hybrid.h
 * @brief Hybrid A* 搜索节点 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * 与 2D 网格节点不同，HybridNode 存储连续 (x, y, theta) 位姿，
 * 并记录到达该节点所使用的运动原语索引与转向方向，
 * 用于路径回溯时还原运动方向信息。
 */
#ifndef RMP_PATH_PLANNER_HYBRID_ASTAR_NODE_HYBRID_H_
#define RMP_PATH_PLANNER_HYBRID_ASTAR_NODE_HYBRID_H_

#include <memory>
#include <cstdint>

#include "common/geometry/point.h"
#include "graph_planner/hybrid_astar_planner/motions.h"
#include "graph_planner/hybrid_astar_planner/motion_table.h"

namespace rmp::path_planner {

class HybridAStarMotionTable;  // forward declaration

class NodeHybrid {
public:
  using NodePtr = NodeHybrid *;

  explicit NodeHybrid(uint64_t index);
  ~NodeHybrid();

  void reset();

  bool operator==(const NodeHybrid & other) const;

  // ---- 累积代价 ----
  double accumulated_cost() const { return accumulated_cost_; }
  void setAccumulatedCost(double cost) { accumulated_cost_ = cost; }

  // ---- 访问标记（closed 列表） ----
  bool is_visited() const { return is_visited_; }
  void visited() { is_visited_ = true; }

  // ---- 索引 ----
  uint64_t index() const { return index_; }

  // ---- 位姿 ----
  rmp::common::geometry::Point3d pose() const { return pose_; }
  void setPose(const rmp::common::geometry::Point3d & pose);

  // ---- 运动原语 ----
  void setMotionPrimitiveIndex(unsigned int idx, TurnDirection dir);
  unsigned int motion_primitive_index() const { return motion_primitive_index_; }
  TurnDirection turn_direction() const { return turn_dir_; }

  /**
   * @brief 计算从当前节点到 child 的转移代价。
   *
   * 综合考虑行驶距离 + 转向惩罚 + 方向切换惩罚 + 倒车惩罚。
   */
  double getTraversalCost(
    const NodePtr & child,
    const HybridAStarMotionTable & motion_table) const;

  NodePtr parent{nullptr};

private:
  double accumulated_cost_{std::numeric_limits<double>::max()};
  bool is_visited_{false};
  uint64_t index_{0};
  unsigned int motion_primitive_index_{0};
  TurnDirection turn_dir_{TurnDirection::UNKNOWN};
  rmp::common::geometry::Point3d pose_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_HYBRID_ASTAR_NODE_HYBRID_H_
