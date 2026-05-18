/**
 * @file hybrid_astar_planner.h
 * @brief Hybrid A* global planner migrated from ROS1 to ROS2 Nav2.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_H_

#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "common/structure/node.h"
#include "graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "path_planner.h"

namespace rmp::path_planner {

struct HybridAStarConfig
{
  int dim_3_size{72};
  int max_iterations{10000};
  int max_approach_iterations{1000};
  double goal_tolerance{2.0};
  double minimum_turning_radius{0.4};
  double curve_sample_ratio{0.1};
  double non_straight_penalty{1.2};
  double change_penalty{0.0};
  double reverse_penalty{2.0};
  double retrospective_penalty{0.015};
  double analytic_expansion_max_length{3.0};
  double lambda_h{2.0};
  int default_graph_size{100000};
};

class HybridAStarPathPlanner : public PathPlanner
{
public:
  using QueueNode = std::pair<double, NodeHybrid::NodePtr>;

  struct NodeComparator
  {
    bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
    {
      return lhs.first > rhs.first;
    }
  };

  using Queue = std::priority_queue<QueueNode, std::vector<QueueNode>, NodeComparator>;
  using Graph = std::unordered_map<uint64_t, NodeHybrid>;

  explicit HybridAStarPathPlanner(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~HybridAStarPathPlanner() override;

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

  void setHybridConfig(const HybridAStarConfig & config);

  static rmp::common::geometry::Point3d getPose(uint64_t index);
  static uint64_t getIndex(const rmp::common::geometry::Point3d & pose);

private:
  bool createPath(
    const rmp::common::geometry::Point3d & start,
    const rmp::common::geometry::Point3d & goal,
    rmp::common::geometry::Points3d * path,
    rmp::common::geometry::Points3d * expand);

  double getHeuristicCost(
    const NodeHybrid::NodePtr & node,
    const NodeHybrid::NodePtr & goal) const;
  double getObstacleHeuristic(const NodeHybrid::NodePtr & node) const;
  double getDistanceHeuristic(
    const NodeHybrid::NodePtr & node,
    const NodeHybrid::NodePtr & goal) const;
  bool precomputeObstacleHeuristic(const NodeHybrid::NodePtr & goal);
  NodeHybrid::NodePtr tryAnalyticExpansion(
    const NodeHybrid::NodePtr & node,
    const NodeHybrid::NodePtr & goal);
  bool backtracePath(NodeHybrid::NodePtr & node, rmp::common::geometry::Points3d * path);
  void getNeighbors(
    const NodeHybrid::NodePtr & node,
    std::vector<NodeHybrid::NodePtr> & neighbors);
  bool isCollision(const rmp::common::geometry::Point3d & pose) const;
  NodeHybrid::NodePtr addToGraph(uint64_t index);
  void clearGraph();
  void addToQueue(double cost, NodeHybrid::NodePtr & node);
  void clearQueue();
  bool isReachGoal(
    const NodeHybrid::NodePtr & node,
    const NodeHybrid::NodePtr & goal) const;
  bool initializeMotionTableFromCostmap();
  void clearAnalyticExpansionNodes();
  void fillSearchedPointsDebugInfo(const rmp::common::geometry::Points3d & expand);

  HybridAStarConfig hybrid_cfg_;
  static HybridAStarMotionTable motion_table_;
  static std::vector<rmp::common::structure::Node<int>> grid_motions_;

  rmp::common::geometry::Point3d stored_goal_;
  rmp::common::geometry::Points3d hybrid_last_path_;

  Graph graph_;
  Queue queue_;
  std::pair<float, uint64_t> best_heuristic_node_;
  std::vector<std::vector<double>> obstacle_hmap_;
  std::vector<NodeHybrid::NodePtr> expansions_node_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_H_
