/**
 * @file node_hybrid.cpp
 * @brief Hybrid A* 搜索节点实现 — 从 ROS1 迁移至 ROS2 Nav2。
 *
 * 主要实现转移代价计算逻辑：
 *   cost = 行驶弧长 × 距离折扣
 *         + 非直行惩罚
 *         + 方向切换惩罚
 *         + 倒车惩罚
 */
#include "graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "graph_planner/hybrid_astar_planner/motion_table.h"

namespace rmp::path_planner {

NodeHybrid::NodeHybrid(uint64_t index)
: index_(index)
{
}

NodeHybrid::~NodeHybrid() = default;

void NodeHybrid::reset()
{
  parent = nullptr;
  accumulated_cost_ = std::numeric_limits<double>::max();
  is_visited_ = false;
  motion_primitive_index_ = 0;
  turn_dir_ = TurnDirection::UNKNOWN;
}

bool NodeHybrid::operator==(const NodeHybrid & other) const
{
  return index_ == other.index_;
}

void NodeHybrid::setPose(const rmp::common::geometry::Point3d & pose)
{
  pose_.setX(pose.x());
  pose_.setY(pose.y());
  pose_.setTheta(pose.theta());
}

void NodeHybrid::setMotionPrimitiveIndex(unsigned int idx, TurnDirection dir)
{
  motion_primitive_index_ = idx;
  turn_dir_ = dir;
}

/**
 * @brief 计算当前节点到 child 的转移代价。
 *
 * 代价由 4 部分组成（与 Nav2 SmacPlanner 惩罚策略一致）：
 *   ① travel_cost × travel_distance_reward  — 基础行驶弧长
 *   ② non_straight_penalty                  — 非直行附加代价
 *   ③ change_penalty                        — 转向方向切换附加代价
 *   ④ reverse_penalty                       — 倒车附加代价
 */
double NodeHybrid::getTraversalCost(
  const NodePtr & child,
  const HybridAStarMotionTable & motion_table) const
{
  const unsigned int child_motion = child->motion_primitive_index();
  double travel_cost = motion_table.travel_costs[child_motion];
  double cost = travel_cost * motion_table.travel_distance_reward;

  // 非直行惩罚
  if (child->turn_direction() != TurnDirection::FORWARD &&
      child->turn_direction() != TurnDirection::REVERSE)
  {
    cost += motion_table.non_straight_penalty;
  }

  // 转向方向切换惩罚
  if (turn_dir_ != TurnDirection::UNKNOWN &&
      turn_dir_ != child->turn_direction())
  {
    cost += motion_table.change_penalty;
  }

  // 倒车惩罚
  if (child->turn_direction() == TurnDirection::REVERSE ||
      child->turn_direction() == TurnDirection::REV_LEFT ||
      child->turn_direction() == TurnDirection::REV_RIGHT)
  {
    cost += motion_table.reverse_penalty;
  }

  return cost;
}

}  // namespace rmp::path_planner
