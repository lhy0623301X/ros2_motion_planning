/**
 * @file node_hybrid.h
 * @brief Hybrid A* search node migrated from ROS1.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_NODE_HYBRID_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_NODE_HYBRID_H_

#include <cstdint>
#include <limits>

#include "geometry/point.h"
#include "graph_planner/hybrid_astar_planner/motion_table.h"
#include "graph_planner/hybrid_astar_planner/motions.h"

namespace rmp::path_planner {

class NodeHybrid
{
public:
  using NodePtr = NodeHybrid *;

  explicit NodeHybrid(uint64_t index);
  ~NodeHybrid();

  void reset();
  bool operator==(const NodeHybrid & other) const;

  double accumulated_cost() const { return accumulated_cost_; }
  void setAccumulatedCost(double cost) { accumulated_cost_ = cost; }
  bool is_visited() const { return is_visited_; }
  void visited() { is_visited_ = true; }
  uint64_t index() const { return index_; }
  rmp::common::geometry::Point3d pose() const { return pose_; }
  void setPose(const rmp::common::geometry::Point3d & pose);
  void setMotionPrimitiveIndex(unsigned int index, TurnDirection turn_dir);
  unsigned int motion_primitive_index() const { return motion_primitive_index_; }
  TurnDirection turn_direction() const { return turn_dir_; }

  double getTraversalCost(
    const NodePtr & child,
    const HybridAStarMotionTable & motion_table) const;

  NodePtr parent{nullptr};

private:
  double accumulated_cost_{std::numeric_limits<double>::max()};
  bool is_visited_{false};
  uint64_t index_{0};
  unsigned int motion_primitive_index_{std::numeric_limits<unsigned int>::max()};
  TurnDirection turn_dir_{TurnDirection::FORWARD};
  rmp::common::geometry::Point3d pose_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_NODE_HYBRID_H_
