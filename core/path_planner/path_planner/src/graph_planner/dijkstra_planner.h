/**
 * @file dijkstra_planner.h
 * @brief Minimal Dijkstra global planner migrated first for phased validation.
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_DIJKSTRA_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_DIJKSTRA_PLANNER_H_

#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

class DijkstraPathPlanner : public PathPlanner
{
public:
  explicit DijkstraPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

private:
  struct Node
  {
    int x{0};
    int y{0};
    double g{0.0};
    double h{0.0};
    int id{0};
    int parent_id{0};

    Node() = default;
    Node(int x_in, int y_in, double g_in = 0.0, double h_in = 0.0, int id_in = 0, int pid_in = 0)
    : x(x_in), y(y_in), g(g_in), h(h_in), id(id_in), parent_id(pid_in) {}

    Node operator+(const Node & other) const
    {
      return Node(x + other.x, y + other.y);
    }

    bool operator==(const Node & other) const
    {
      return x == other.x && y == other.y;
    }

    struct CompareCost
    {
      bool operator()(const Node & lhs, const Node & rhs) const
      {
        return (lhs.g + lhs.h > rhs.g + rhs.h) ||
               ((lhs.g + lhs.h == rhs.g + rhs.h) && (lhs.h > rhs.h));
      }
    };
  };

  static const std::vector<Node> motions_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_DIJKSTRA_PLANNER_H_
