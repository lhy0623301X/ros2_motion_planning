/**
 * @file teb_optimizer.h
 * @brief Lightweight TEB optimization core.
 */
#ifndef RMP_CONTROLLER_TEB_OPTIMIZER_H_
#define RMP_CONTROLLER_TEB_OPTIMIZER_H_

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "teb_controller/teb_types.h"

namespace rmp::controller {

class TEBOptimizer
{
public:
  TEBOptimizer() = default;
  ~TEBOptimizer() = default;

  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint);

  TEBTrajectory initializeTrajectory(
    const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const TEBControllerConfig & cfg,
    const nav2_costmap_2d::Costmap2D * costmap = nullptr) const;

  TEBOptimizationSummary optimize(
    TEBTrajectory & trajectory,
    const TEBState & state,
    const nav_msgs::msg::Path & global_plan,
    nav2_costmap_2d::Costmap2DROS & costmap_ros,
    const TEBControllerConfig & cfg) const;

  bool isTrajectoryFeasible(
    const TEBTrajectory & trajectory,
    nav2_costmap_2d::Costmap2DROS & costmap_ros,
    const TEBControllerConfig & cfg) const;
  bool isTrajectoryFeasible(
    const TEBTrajectory & trajectory,
    nav2_costmap_2d::Costmap2DROS & costmap_ros,
    const TEBControllerConfig & cfg,
    std::string * failure_reason) const;

  TEBCommand extractCommand(
    const TEBTrajectory & trajectory,
    const TEBState & state,
    const TEBControllerConfig & cfg) const;

private:
  std::vector<TEBPose> buildReferenceBand(
    const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const TEBControllerConfig & cfg,
    const nav2_costmap_2d::Costmap2D * costmap) const;
  TEBTrajectory initializeTrajectoryFromReferences(
    const std::vector<TEBPose> & references,
    const TEBControllerConfig & cfg) const;
  std::vector<std::vector<TEBPose>> buildCandidateReferenceBands(
    const std::vector<TEBPose> & center_references,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  std::vector<TEBPose> offsetReferenceBand(
    const std::vector<TEBPose> & references,
    double side,
    double offset_distance,
    const TEBControllerConfig & cfg) const;
  bool referenceBandHasHardCollision(
    const std::vector<TEBPose> & references,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  bool isReferenceBandCandidateValid(
    const std::vector<TEBPose> & references,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  void updateTimedElasticBand(
    TEBTrajectory & trajectory,
    const std::vector<TEBPose> & references,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  void updateTimeDiffs(
    TEBTrajectory & trajectory,
    const TEBState & state,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  void resizeTrajectory(
    TEBTrajectory & trajectory,
    const TEBControllerConfig & cfg) const;
  void refreshOrientations(TEBTrajectory & trajectory) const;
  double computeTotalCost(
    const TEBTrajectory & trajectory,
    const TEBState & state,
    const std::vector<TEBPose> & references,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  double goalHeadingCost(
    const TEBTrajectory & trajectory,
    const TEBPose & goal_pose) const;
  double velocityCost(
    const TEBTrajectory & trajectory,
    const TEBState & state,
    const TEBControllerConfig & cfg) const;
  double accelerationCost(
    const TEBTrajectory & trajectory,
    const TEBState & state,
    const TEBControllerConfig & cfg) const;
  double kinematicsCost(
    const TEBTrajectory & trajectory,
    const TEBControllerConfig & cfg) const;
  double obstaclePenalty(
    const TEBPose & pose,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  double obstaclePenalty(
    const TEBPose & pose,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg,
    std::string * debug_reason) const;
  std::pair<double, double> obstacleGradient(
    const TEBPose & pose,
    nav2_costmap_2d::Costmap2D & costmap,
    const TEBControllerConfig & cfg) const;
  TEBPose interpolateReference(
    const std::vector<TEBPose> & references,
    double index) const;
  double poseDistance(const TEBPose & lhs, const TEBPose & rhs) const;
  double normalizeAngle(double angle) const;

  std::vector<geometry_msgs::msg::Point> footprint_;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_TEB_OPTIMIZER_H_
