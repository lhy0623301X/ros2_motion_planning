/**
 * @file rrt_planner.h
 * @brief RRT（快速扩展随机树）路径规划器 — 从 ROS1 迁移至 ROS2 Nav2。
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_PLANNER_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_PLANNER_H_

#include <random>
#include <unordered_map>
#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

/**
 * @brief 采样类规划器的共享配置参数。
 *
 * 从 ROS1 的 protobuf SamplePathPlanner 消息迁移而来，
 * 改由 ROS2 参数系统注入。所有 RRT 系列算法共享这组参数。
 */
struct SamplePlannerConfig
{
  int sample_points{5000};                      // 最大采样次数
  double sample_max_distance{10.0};             // 单步扩展最大距离（栅格）
  double optimization_radius{10.0};             // 优化半径（RRT* 使用）
  double optimization_sample_probability{0.05}; // 朝目标采样的概率
};

/**
 * @brief RRT（Rapidly-exploring Random Tree）路径规划器。
 *
 * 算法思路：
 *   1. 在地图中随机采样一个点
 *   2. 在已有树中找到距采样点最近的节点
 *   3. 从最近节点朝采样点方向以 max_distance 为步长扩展一个新节点
 *   4. Bresenham 直线碰撞检测：最近节点 → 新节点
 *   5. 若新节点距目标 < max_distance 且直线无碰撞，则连接目标，规划完成
 *   6. 重复直到达到最大采样次数
 */
class RRTPathPlanner : public PathPlanner
{
public:
  /**
   * @brief 内部树节点，复用 graph_planner 的 Node 结构风格。
   *
   * 与图搜索算法不同，RRT 的 g 值记录的是从根节点沿树边到达该节点的
   * 累计路径长度，而非网格代价。parent_id 指向树中的父节点。
   */
  struct Node
  {
    int x{0};
    int y{0};
    double g{0.0};
    double h{0.0};
    int id{0};
    int parent_id{0};

    Node() = default;
    Node(int x_in, int y_in, double g_in = 0.0, double h_in = 0.0,
         int id_in = 0, int pid_in = 0)
    : x(x_in), y(y_in), g(g_in), h(h_in), id(id_in), parent_id(pid_in) {}

    bool operator==(const Node & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  explicit RRTPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

  void setSampleConfig(const SamplePlannerConfig & cfg);

protected:
  /**
   * @brief 在树中找到距 node 最近的节点，并沿方向截断到 max_distance。
   * @return 新节点（id == -1 表示碰撞失败）
   */
  Node findNearestAndSteer(std::unordered_map<int, Node> & tree, const Node & node);

  /**
   * @brief 在地图范围内生成随机节点（以一定概率直接返回目标点）。
   */
  Node generateRandomNode();

  /**
   * @brief 检查新节点是否能直连目标（距离 < max_distance 且直线无碰撞）。
   */
  bool checkGoalReachable(const Node & new_node);

  /**
   * @brief Bresenham 直线碰撞检测：(x0,y0) → (x1,y1) 之间是否有致命障碍物。
   */
  bool isLineCollision(int x0, int y0, int x1, int y1) const;

  /**
   * @brief 填充采样可视化数据（采样点 + 树边）到 PlannerDebugInfo。
   */
  void fillSampleVisualization(const Points3d & expand);

  Node start_;
  Node goal_;
  std::unordered_map<int, Node> sample_list_;
  SamplePlannerConfig sample_cfg_;

  std::mt19937 rng_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_PLANNER_H_
