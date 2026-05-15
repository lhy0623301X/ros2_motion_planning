/**
 * @file hybrid_astar_planner.h
 * @brief Hybrid A* 路径规划器 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * Hybrid A* 在标准 A* 的基础上引入连续朝向维度，
 * 使用 Dubins 曲线作为运动原语和启发式函数，
 * 在考虑车辆运动学约束的同时搜索全局最优路径。
 *
 * 与 2D 图搜索算法的核心区别：
 *   - 搜索空间为 3D (x, y, theta)，节点索引由三维量化
 *   - 邻居扩展基于运动原语（直行/左转/右转），而非网格 4/8 连通
 *   - 启发式 = max(障碍启发, 距离启发)
 *     · 障碍启发：从目标反向 Dijkstra 忽略朝向的 2D 代价场
 *     · 距离启发：Dubins 曲线长度（考虑转弯半径约束）
 *   - 支持 Analytic Expansion：当距离目标足够近时，
 *     尝试用 Dubins 曲线直接连接到目标，加速收敛
 */
#ifndef RMP_PATH_PLANNER_HYBRID_ASTAR_PLANNER_H_
#define RMP_PATH_PLANNER_HYBRID_ASTAR_PLANNER_H_

#include <queue>
#include <unordered_map>
#include <vector>

#include "common/structure/node.h"
#include "path_planner.h"
#include "graph_planner/hybrid_astar_planner/node_hybrid.h"

namespace rmp::path_planner {

/**
 * @brief Hybrid A* 专用配置参数（通过工厂从 ROS2 参数读取）
 */
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

class HybridAStarPathPlanner : public PathPlanner {
public:
  using QueueNode = std::pair<double, NodeHybrid::NodePtr>;

  struct NodeComparator {
    bool operator()(const QueueNode & a, const QueueNode & b) const {
      return a.first > b.first;
    }
  };

  using Queue = std::priority_queue<QueueNode, std::vector<QueueNode>, NodeComparator>;
  using Graph = std::unordered_map<uint64_t, NodeHybrid>;

  explicit HybridAStarPathPlanner(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~HybridAStarPathPlanner() override = default;

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

  void setHybridConfig(const HybridAStarConfig & cfg);

  // ---- 3D 索引 ↔ 位姿 互转（基于 motion_table_ 的量化参数） ----

  static rmp::common::geometry::Point3d getPose(uint64_t index);
  static uint64_t getIndex(const rmp::common::geometry::Point3d & pose);

protected:
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

  bool backtracePath(
    NodeHybrid::NodePtr & node,
    rmp::common::geometry::Points3d * path);

  void getNeighbors(
    const NodeHybrid::NodePtr & node,
    std::vector<NodeHybrid::NodePtr> & neighbors);

  bool isCollision(const rmp::common::geometry::Point3d & pose);

  NodeHybrid::NodePtr addToGraph(uint64_t index);
  void clearGraph();
  void addToQueue(double cost, NodeHybrid::NodePtr & node);
  void clearQueue();

  bool isReachGoal(
    const NodeHybrid::NodePtr & node,
    const NodeHybrid::NodePtr & goal) const;

protected:
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

#endif  // RMP_PATH_PLANNER_HYBRID_ASTAR_PLANNER_H_
