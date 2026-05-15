/**
 * @file lpa_star_planner.h
 * @brief LPA* (Lifelong Planning A*) incremental global planner.
 *
 * LPA* searches FORWARD from start to goal (unlike D* Lite which searches
 * backward). It maintains rhs values that reflect one-step lookahead costs,
 * and only recomputes the portion of the search tree affected by edge-cost
 * changes (costmap updates), making re-planning after small environment
 * changes much cheaper than a full A* search.
 *
 * Limitation: LPA* is only incremental with respect to edge-cost changes
 * from a fixed start. When the robot moves (start changes), the entire
 * search must be re-initialised.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_LPA_STAR_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_LPA_STAR_PLANNER_H_

#include <limits>
#include <map>
#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

class LPAStarPathPlanner : public PathPlanner
{
public:
  explicit LPAStarPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~LPAStarPathPlanner() override;

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

private:
  static constexpr double INF = std::numeric_limits<double>::max();
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

    LNode() = default;
    LNode(int x_in, int y_in,
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
  double getH(const LNodePtr n1, const LNodePtr n2) const;
  double calculateKey(const LNodePtr s) const;
  bool isCollision(const LNodePtr n1, const LNodePtr n2) const;
  void getNeighbours(const LNodePtr node, std::vector<LNodePtr> & neighbours) const;
  double getCost(const LNodePtr n1, const LNodePtr n2) const;
  void updateVertex(LNodePtr u);
  void computeShortestPath();
  bool extractPath(const LNode & start, const LNode & goal);
  LNode getState(const LNode & node) const;
  void fillSearchedPointsDebugInfo(const Points3d & expand);

  bool initialized_{false};
  unsigned char * curr_global_costmap_{nullptr};
  unsigned char * last_global_costmap_{nullptr};
  LNodePtr * * map_{nullptr};
  std::multimap<double, LNodePtr> open_list_;
  Points3d path_;
  Points3d expand_;
  LNode start_;
  LNode goal_;
  LNodePtr start_ptr_{nullptr};
  LNodePtr goal_ptr_{nullptr};
  LNodePtr last_ptr_{nullptr};
  int nx_{0};
  int ny_{0};
  int map_size_{0};
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_LPA_STAR_PLANNER_H_
