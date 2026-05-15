/**
 * @file dstar_lite_planner.h
 * @brief D* Lite 增量式全局规划器 — 支持动态代价地图变化时的高效重规划。
 *
 * D* Lite 从目标向起点反向搜索，当机器人移动或地图发生变化时，
 * 仅对受影响的局部区域进行增量修复，而无需重新搜索整个地图。
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_LITE_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_LITE_PLANNER_H_

#include <limits>
#include <map>
#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

class DStarLitePathPlanner : public PathPlanner
{
public:
  explicit DStarLitePathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~DStarLitePathPlanner() override;

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

private:
  static constexpr double INF = std::numeric_limits<double>::max();
  // static constexpr int kMaxExtractSteps = 1000;
  static constexpr int kWindowSize = 70;

  struct LNode
  {
    int x{0};
    int y{0};
    double g{std::numeric_limits<double>::max()};
    double h{std::numeric_limits<double>::max()};
    int id{0};
    int parent_id{-1};
    double rhs{std::numeric_limits<double>::max()};
    double key{std::numeric_limits<double>::max()};
    std::multimap<double, LNode *>::iterator open_it;

    LNode(int x_in = 0, int y_in = 0,
          double g_in = std::numeric_limits<double>::max(),
          double h_in = std::numeric_limits<double>::max(),
          int id_in = 0, int pid_in = -1,
          double rhs_in = std::numeric_limits<double>::max(),
          double key_in = std::numeric_limits<double>::max())
    : x(x_in), y(y_in), g(g_in), h(h_in),
      id(id_in), parent_id(pid_in), rhs(rhs_in), key(key_in) {}

    bool operator==(const LNode & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  using LNodePtr = LNode *;

  void initMap();
  void reset();
  void freeMap();
  void ensureInitialized();

  double getH(LNodePtr n1, LNodePtr n2) const;
  double calculateKey(LNodePtr s) const;
  bool isCollision(LNodePtr n1, LNodePtr n2) const;
  void getNeighbours(LNodePtr u, std::vector<LNodePtr> & neighbours) const;
  double getCost(LNodePtr n1, LNodePtr n2) const;
  void updateVertex(LNodePtr u);
  void computeShortestPath();
  bool extractPath(const LNode & start, const LNode & goal);
  LNode getState(const LNode & current) const;
  void fillSearchedPointsDebugInfo(const Points3d & expand);

  unsigned char * curr_global_costmap_{nullptr};
  unsigned char * last_global_costmap_{nullptr};
  LNodePtr ** map_{nullptr};
  std::multimap<double, LNodePtr> open_list_;
  Points3d path_;
  Points3d expand_;
  LNode start_;
  LNode goal_;
  LNodePtr start_ptr_{nullptr};
  LNodePtr goal_ptr_{nullptr};
  LNodePtr last_ptr_{nullptr};
  double km_{0.0};

  int nx_{0};
  int ny_{0};
  int map_size_{0};
  bool initialized_{false};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_DSTAR_LITE_PLANNER_H_
