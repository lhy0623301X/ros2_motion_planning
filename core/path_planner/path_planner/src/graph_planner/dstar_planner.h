/**
 * @file dstar_planner.h
 * @brief D* 增量式全局规划器 — 支持动态代价地图变化时的增量重规划。
 *
 * D* 从目标向起点反向搜索。当机器人沿路径行进并检测到代价地图变化时，
 * 通过 RAISE/LOWER 状态传播机制对受影响区域进行局部修复，
 * 避免每次都从头执行完整搜索。
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_PLANNER_H_

#include <limits>
#include <map>
#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

class DStarPathPlanner : public PathPlanner
{
public:
  explicit DStarPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~DStarPathPlanner() override;

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

private:
  static constexpr double INF = std::numeric_limits<double>::max();
  static constexpr int kWindowSize = 70;

  struct DNode
  {
    enum Tag { NEW = 0, OPEN = 1, CLOSED = 2 };

    int x{0};
    int y{0};
    double g{std::numeric_limits<double>::max()};
    double h{std::numeric_limits<double>::max()};
    int id{0};
    int parent_id{-1};
    int tag{NEW};
    double k{std::numeric_limits<double>::max()};

    DNode(int x_in = 0, int y_in = 0,
          double g_in = std::numeric_limits<double>::max(),
          double h_in = std::numeric_limits<double>::max(),
          int id_in = 0, int pid_in = -1,
          int tag_in = NEW,
          double k_in = std::numeric_limits<double>::max())
    : x(x_in), y(y_in), g(g_in), h(h_in),
      id(id_in), parent_id(pid_in), tag(tag_in), k(k_in) {}

    bool operator==(const DNode & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  using DNodePtr = DNode *;

  void initMap();
  void reset();
  void insert(DNodePtr node_ptr, double h_new);
  bool isCollision(DNodePtr n1, DNodePtr n2) const;
  void getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr> & neighbours) const;
  double calculateObstacleCost(unsigned char cell_cost) const;
  double getCost(DNodePtr n1, DNodePtr n2) const;
  double processState();
  void extractPath(const DNode & start_node, const DNode & goal_node);
  DNode getState(const DNode & current) const;
  void modify(DNodePtr x);
  void fillSearchedPointsDebugInfo(const Points3d & expand_data);

  unsigned char * curr_global_costmap_{nullptr};
  unsigned char * last_global_costmap_{nullptr};
  DNodePtr ** map_{nullptr};
  std::multimap<double, DNodePtr> open_list_;
  Points3d path_;
  Points3d expand_;
  DNode goal_;

  int nx_{0};
  int ny_{0};
  int map_size_{0};
  bool initialized_{false};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_PLANNER_H_
