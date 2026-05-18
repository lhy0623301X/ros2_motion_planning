/**
 * @file node_hybrid.cpp
 * @brief Hybrid A* search node implementation migrated from ROS1.
 */
#include "graph_planner/hybrid_astar_planner/node_hybrid.h"

namespace rmp::path_planner {

NodeHybrid::NodeHybrid(uint64_t index)
: index_(index)
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  accumulated_cost_ = std::numeric_limits<double>::max();
  is_visited_ = false;
  motion_primitive_index_ = std::numeric_limits<unsigned int>::max();
  turn_dir_ = TurnDirection::FORWARD;
  pose_.setX(0.0);
  pose_.setY(0.0);
  pose_.setTheta(0.0);
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

void NodeHybrid::setMotionPrimitiveIndex(unsigned int index, TurnDirection turn_dir)
{
  motion_primitive_index_ = index;
  turn_dir_ = turn_dir;
}

double NodeHybrid::getTraversalCost(
  const NodePtr & child,
  const HybridAStarMotionTable & motion_table) const
{
  if (motion_primitive_index_ == std::numeric_limits<unsigned int>::max()) {
    return motion_table.projections[0].x();
  }

  const double raw_cost =
    motion_table.travel_costs[child->motion_primitive_index()] *
    motion_table.travel_distance_reward;

  double travel_cost = raw_cost;
  if (child->turn_direction() != TurnDirection::FORWARD &&
    child->turn_direction() != TurnDirection::REVERSE)
  {
    travel_cost *=
      (turn_dir_ == child->turn_direction()) ?
      motion_table.non_straight_penalty :
      motion_table.non_straight_penalty + motion_table.change_penalty;
  }

  if (child->turn_direction() == TurnDirection::REV_LEFT ||
    child->turn_direction() == TurnDirection::REV_RIGHT ||
    child->turn_direction() == TurnDirection::REVERSE)
  {
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

}  // namespace rmp::path_planner
