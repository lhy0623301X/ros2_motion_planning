/**
 * @file jps_planner.h
 * @brief JPS（跳点搜索）全局路径规划器，从 ROS1 迁移至 ROS2 Nav2。
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_JPS_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_JPS_PLANNER_H_

#include <array>
#include <queue>
#include <unordered_map>
#include <vector>

#include "path_planner.h"

namespace rmp::path_planner {

class JPSPathPlanner : public PathPlanner
{
public:
  explicit JPSPathPlanner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool plan(
    const Point3d & start,
    const Point3d & goal,
    Points3d * path,
    Points3d * expand) override;

protected:
  // 跳点节点：在基础 Node 基础上增加 forced neighbor id（fid）
  struct JNode
  {
    int x{0};
    int y{0};
    double g{0.0};
    double h{0.0};
    int id{0};
    int parent_id{0};
    int fid{-1};

    JNode() = default;
    JNode(int x_in, int y_in, double g_in = 0.0, double h_in = 0.0,
          int id_in = 0, int pid_in = 0, int fid_in = -1)
    : x(x_in), y(y_in), g(g_in), h(h_in), id(id_in), parent_id(pid_in), fid(fid_in) {}

    bool operator==(const JNode & other) const
    {
      return x == other.x && y == other.y;
    }

    struct CompareCost
    {
      bool operator()(const JNode & lhs, const JNode & rhs) const
      {
        return (lhs.g + lhs.h > rhs.g + rhs.h) ||
               ((lhs.g + lhs.h == rhs.g + rhs.h) && (lhs.h > rhs.h));
      }
    };
  };

  using OpenList = std::priority_queue<JNode, std::vector<JNode>, JNode::CompareCost>;

  void jump(const JNode & node, OpenList & open_list);
  bool checkStraightLine(int dir, const JNode & node, OpenList & open_list);
  bool checkSlashLine(int dir, const JNode & node, OpenList & open_list, bool from_cur = true);
  bool forceNeighborDetect(int dir, int cur_id, std::vector<int> & fn_id);

private:
  void fillSearchedPointsDebugInfo(const Points3d & expand);

  JNode start_;
  JNode goal_;

  // 缓存地图尺寸，每次 plan() 开始时更新
  int nx_{0};
  int ny_{0};
  int map_size_{0};

  // 八方向偏移量与 forced neighbor 检测映射，在 plan() 中动态计算
  // [left, right, top, bottom, left-top, right-bottom, right-top, left-bottom]
  std::array<int, 8> dirs_{};
  std::unordered_map<int, std::pair<int, int>> dir_to_obs_id_;
};

}  // namespace rmp::path_planner

#endif  // RMP_PATH_PLANNER_GRAPH_PLANNER_JPS_PLANNER_H_
