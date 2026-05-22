/**
 * @file teb_optimizer.cpp
 * @brief g2o-based TEB optimizer.
 */
#include "teb_controller/teb_optimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"

namespace rmp::controller {

namespace {

// 这份实现里的 TEB 采用“离散位姿 + 离散时间”的表达：
// - pose 顶点表示局部轨迹上的几何形状
// - dt 顶点表示相邻两个位姿之间的时间分配
//
// 然后把“贴参考路径、避障、平滑、速度约束、加速度约束、终点姿态约束”
// 全部写成 g2o 图中的 edge，让求解器在每个控制周期做一轮局部优化。

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double lerp(double lhs, double rhs, double t)
{
  return lhs + (rhs - lhs) * t;
}

double safeRatio(double numerator, double denominator)
{
  return denominator > 1.0e-6 ? numerator / denominator : 0.0;
}

double normalizeAngleLocal(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double segmentHeading(const TEBPose & start, const TEBPose & end)
{
  return std::atan2(end.y - start.y, end.x - start.x);
}

double computeObstaclePenaltyLocal(
  const TEBPose & pose,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  nav2_costmap_2d::Costmap2D & costmap,
  const TEBControllerConfig & cfg,
  std::string * debug_reason = nullptr)
{
  auto set_reason = [&](const std::string & reason) {
      if (debug_reason != nullptr) {
        *debug_reason = reason;
      }
    };

  // 这里不再只检查机器人中心点，而是把 footprint 变换到世界坐标后，
  // 沿边界线段和中心到边界的辐射线做 costmap 采样。
  auto makeFallbackFootprint = [&](double radius) {
      std::vector<geometry_msgs::msg::Point> local_footprint;
      const double safe_radius = std::max(radius, 1.0e-3);
      constexpr int kSegments = 12;
      local_footprint.reserve(kSegments);
      for (int i = 0; i < kSegments; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kSegments);
        geometry_msgs::msg::Point point;
        point.x = safe_radius * std::cos(angle);
        point.y = safe_radius * std::sin(angle);
        point.z = 0.0;
        local_footprint.push_back(point);
      }
      return local_footprint;
    };

  auto transformFootprintPoint = [&](const geometry_msgs::msg::Point & local_point) {
      geometry_msgs::msg::Point world_point;
      const double cos_theta = std::cos(pose.theta);
      const double sin_theta = std::sin(pose.theta);
      world_point.x = pose.x + local_point.x * cos_theta - local_point.y * sin_theta;
      world_point.y = pose.y + local_point.x * sin_theta + local_point.y * cos_theta;
      world_point.z = 0.0;
      return world_point;
    };

  auto costAtWorld = [&](double x, double y) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(x, y, mx, my)) {
        set_reason("footprint sample is outside local costmap");
        return nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      const unsigned char cost = costmap.getCost(mx, my);
      if (cfg.unknown_as_obstacle && cost == nav2_costmap_2d::NO_INFORMATION) {
        set_reason("footprint sample is unknown and treated as obstacle");
        return nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      return cost;
    };

  auto maxCostAlongLine = [&](const geometry_msgs::msg::Point & start,
      const geometry_msgs::msg::Point & end) {
      const double distance = std::hypot(end.x - start.x, end.y - start.y);
      const double step = std::max(0.01, costmap.getResolution() * 0.5);
      const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
      unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
      for (int i = 0; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        const double x = start.x + ratio * (end.x - start.x);
        const double y = start.y + ratio * (end.y - start.y);
        max_cost = std::max(max_cost, costAtWorld(x, y));
      }
      return max_cost;
    };

  const auto effective_footprint =
    footprint.empty() ? makeFallbackFootprint(cfg.robot_radius) : footprint;
  unsigned char max_raw_cost = costAtWorld(pose.x, pose.y);
  if (max_raw_cost >= nav2_costmap_2d::LETHAL_OBSTACLE && debug_reason != nullptr &&
    debug_reason->empty())
  {
    *debug_reason = "robot center hits lethal obstacle or lies outside local costmap";
  }
  if (effective_footprint.size() >= 3U) {
    for (std::size_t i = 0; i < effective_footprint.size(); ++i) {
      const auto start = transformFootprintPoint(effective_footprint[i]);
      const auto end = transformFootprintPoint(
        effective_footprint[(i + 1) % effective_footprint.size()]);
      max_raw_cost = std::max(max_raw_cost, maxCostAlongLine(start, end));

      geometry_msgs::msg::Point center;
      center.x = pose.x;
      center.y = pose.y;
      center.z = 0.0;
      max_raw_cost = std::max(max_raw_cost, maxCostAlongLine(center, start));
    }
  }

  constexpr double kHardObstaclePenalty = 1.0e4;
  if (max_raw_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    if (debug_reason != nullptr && debug_reason->empty()) {
      *debug_reason = "footprint boundary sampling reaches lethal obstacle";
    }
    return kHardObstaclePenalty;
  }
  if (max_raw_cost == nav2_costmap_2d::NO_INFORMATION) {
    set_reason("footprint sampling lands in unknown area");
    return 5.0;
  }

  const double normalized_cost = static_cast<double>(max_raw_cost) /
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const double approximate_clearance =
    (1.0 - std::min(normalized_cost, 1.0)) * cfg.min_obstacle_distance * 2.0;
  const double safety_margin = cfg.min_obstacle_distance + cfg.robot_radius;
  const double clearance_violation = std::max(0.0, safety_margin - approximate_clearance);
  return normalized_cost + clearance_violation * clearance_violation;
}

class VertexPose2D : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override
  {
    _estimate.setZero();
  }

  void oplusImpl(const double * update) override
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] = normalizeAngleLocal(_estimate[2] + update[2]);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override
  {
    _estimate = 0.1;
  }

  void oplusImpl(const double * update) override
  {
    _estimate = std::max(0.01, _estimate + update[0]);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

class EdgePosePrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose2D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 把 band 拉回参考路径，避免纯 obstacle / smoothness 约束把轨迹拉散。

  void computeError() override
  {
    const auto * vertex = static_cast<const VertexPose2D *>(_vertices[0]);
    const Eigen::Vector3d delta = vertex->estimate() - _measurement;
    _error[0] = delta[0];
    _error[1] = delta[1];
    _error[2] = normalizeAngleLocal(delta[2]);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

class EdgeGoalHeading : public g2o::BaseUnaryEdge<1, double, VertexPose2D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 单独约束终点朝向，让 band 在接近局部目标时也能收敛到合理姿态。

  void computeError() override
  {
    const auto * vertex = static_cast<const VertexPose2D *>(_vertices[0]);
    _error[0] = normalizeAngleLocal(vertex->estimate()[2] - _measurement);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

class EdgeTimeRef : public g2o::BaseUnaryEdge<1, double, VertexTimeDiff>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 偏好 dt 接近 dt_ref，但不强行固定，让图优化能自己拉伸/压缩时间轴。

  void computeError() override
  {
    const auto * vertex = static_cast<const VertexTimeDiff *>(_vertices[0]);
    _error[0] = vertex->estimate() - _measurement;
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

class EdgeObstacle : public g2o::BaseUnaryEdge<1, double, VertexPose2D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 对每个 pose 顶点施加 footprint obstacle 代价，是 TEB 避障的核心约束之一。

  EdgeObstacle(
    nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const TEBControllerConfig & cfg)
  : costmap_(costmap), footprint_(footprint), cfg_(cfg)
  {
  }

  void computeError() override
  {
    const auto * vertex = static_cast<const VertexPose2D *>(_vertices[0]);
    const TEBPose pose{vertex->estimate()[0], vertex->estimate()[1], vertex->estimate()[2]};
    _error[0] = std::sqrt(std::max(
      0.0, computeObstaclePenaltyLocal(pose, footprint_, costmap_, cfg_)));
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }

private:
  nav2_costmap_2d::Costmap2D & costmap_;
  const std::vector<geometry_msgs::msg::Point> & footprint_;
  const TEBControllerConfig & cfg_;
};

class EdgeKinematics : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose2D, VertexPose2D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 约束相邻 pose 之间尽量符合差速底盘的非完整运动学。

  explicit EdgeKinematics(bool allow_backward_motion)
  : allow_backward_motion_(allow_backward_motion)
  {
  }

  void computeError() override
  {
    const auto * start = static_cast<const VertexPose2D *>(_vertices[0]);
    const auto * end = static_cast<const VertexPose2D *>(_vertices[1]);
    const double dx = end->estimate()[0] - start->estimate()[0];
    const double dy = end->estimate()[1] - start->estimate()[1];
    const double heading = std::atan2(dy, dx);
    const double distance = std::hypot(dx, dy);
    const double heading_error = normalizeAngleLocal(heading - start->estimate()[2]);
    const double lateral_error = distance > 1.0e-6 ?
      (-dx * std::sin(start->estimate()[2]) + dy * std::cos(start->estimate()[2])) / distance :
      0.0;
    const double forward_projection =
      dx * std::cos(start->estimate()[2]) + dy * std::sin(start->estimate()[2]);

    _error[0] = lateral_error;
    _error[1] = allow_backward_motion_ ? heading_error : std::max(0.0, -forward_projection);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }

private:
  bool allow_backward_motion_;
};

class EdgeVelocity : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 用 pose_i, pose_{i+1}, dt_i 三个顶点约束线速度和角速度上界。

  EdgeVelocity(double max_linear_velocity, double max_angular_velocity, bool allow_backward_motion)
  : max_linear_velocity_(max_linear_velocity),
    max_angular_velocity_(max_angular_velocity),
    allow_backward_motion_(allow_backward_motion)
  {
    resize(3);
  }

  void computeError() override
  {
    const auto * start = static_cast<const VertexPose2D *>(_vertices[0]);
    const auto * end = static_cast<const VertexPose2D *>(_vertices[1]);
    const auto * dt = static_cast<const VertexTimeDiff *>(_vertices[2]);

    const double delta_t = std::max(0.01, dt->estimate());
    const double dx = end->estimate()[0] - start->estimate()[0];
    const double dy = end->estimate()[1] - start->estimate()[1];
    const double distance = std::hypot(dx, dy);
    double linear_speed = distance / delta_t;
    const double forward_projection =
      dx * std::cos(start->estimate()[2]) + dy * std::sin(start->estimate()[2]);
    if (allow_backward_motion_ && forward_projection < 0.0) {
      linear_speed *= -1.0;
    }

    const double angular_speed = std::fabs(
      normalizeAngleLocal(end->estimate()[2] - start->estimate()[2])) / delta_t;
    _error[0] = std::max(0.0, std::fabs(linear_speed) - max_linear_velocity_);
    _error[1] = std::max(0.0, angular_speed - max_angular_velocity_);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }

private:
  double max_linear_velocity_;
  double max_angular_velocity_;
  bool allow_backward_motion_;
};

class EdgeAcceleration : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 用三个位姿和两个 dt 构造二阶约束，限制速度变化过猛。

  EdgeAcceleration(double max_linear_acceleration, double max_angular_acceleration)
  : max_linear_acceleration_(max_linear_acceleration),
    max_angular_acceleration_(max_angular_acceleration)
  {
    resize(5);
  }

  void computeError() override
  {
    const auto * pose0 = static_cast<const VertexPose2D *>(_vertices[0]);
    const auto * pose1 = static_cast<const VertexPose2D *>(_vertices[1]);
    const auto * pose2 = static_cast<const VertexPose2D *>(_vertices[2]);
    const auto * dt0 = static_cast<const VertexTimeDiff *>(_vertices[3]);
    const auto * dt1 = static_cast<const VertexTimeDiff *>(_vertices[4]);

    const double delta_t0 = std::max(0.01, dt0->estimate());
    const double delta_t1 = std::max(0.01, dt1->estimate());
    const double v0 = std::hypot(
      pose1->estimate()[0] - pose0->estimate()[0],
      pose1->estimate()[1] - pose0->estimate()[1]) / delta_t0;
    const double v1 = std::hypot(
      pose2->estimate()[0] - pose1->estimate()[0],
      pose2->estimate()[1] - pose1->estimate()[1]) / delta_t1;
    const double w0 = std::fabs(
      normalizeAngleLocal(pose1->estimate()[2] - pose0->estimate()[2])) / delta_t0;
    const double w1 = std::fabs(
      normalizeAngleLocal(pose2->estimate()[2] - pose1->estimate()[2])) / delta_t1;

    _error[0] = std::max(
      0.0, std::fabs(v1 - v0) / std::max(delta_t1, 0.01) - max_linear_acceleration_);
    _error[1] = std::max(
      0.0, std::fabs(w1 - w0) / std::max(delta_t1, 0.01) - max_angular_acceleration_);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }

private:
  double max_linear_acceleration_;
  double max_angular_acceleration_;
};

class EdgeSmoothness : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 平滑项不直接管动力学，只抑制局部锯齿，减少 band 折线抖动。

  EdgeSmoothness()
  {
    resize(3);
  }

  void computeError() override
  {
    const auto * pose0 = static_cast<const VertexPose2D *>(_vertices[0]);
    const auto * pose1 = static_cast<const VertexPose2D *>(_vertices[1]);
    const auto * pose2 = static_cast<const VertexPose2D *>(_vertices[2]);
    _error[0] = pose1->estimate()[0] - 0.5 * (pose0->estimate()[0] + pose2->estimate()[0]);
    _error[1] = pose1->estimate()[1] - 0.5 * (pose0->estimate()[1] + pose2->estimate()[1]);
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }
};

}  // namespace

void TEBOptimizer::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint)
{
  // footprint 由 controller 层从 Nav2 costmap 刷新后注入。
  // optimizer 本身不拥有 costmap_ros，因此把机器人轮廓缓存下来供 obstacle edge 使用。
  footprint_ = footprint;
}

TEBTrajectory TEBOptimizer::initializeTrajectory(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const TEBControllerConfig & cfg,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  TEBTrajectory trajectory;
  // 初始化阶段只负责给图优化准备一个“合理初值”：
  // 沿当前局部路径截取一小段，重采样成参考带，再为每个段分配初始 dt。
  const auto references = buildReferenceBand(global_plan, robot_pose, cfg, costmap);
  if (references.empty()) {
    return trajectory;
  }

  trajectory.states.reserve(references.size());
  for (std::size_t i = 0; i < references.size(); ++i) {
    double dt = cfg.dt_ref;
    if (i + 1 < references.size()) {
      // 初始 dt 的启发式含义：
      // 假设机器人大致以 0.75 * vmax 通过这一段，用段长反推时间。
      const double segment_length = poseDistance(references[i], references[i + 1]);
      const double desired_speed = std::max(cfg.max_linear_velocity * 0.75, 0.08);
      dt = clamp(segment_length / desired_speed, cfg.dt_ref * 0.5, cfg.dt_ref * 2.0);
    }
    trajectory.states.push_back(TEBTimedPose{references[i], dt});
  }

  trajectory.feasible = trajectory.states.size() >= 2U;
  return trajectory;
}

TEBOptimizationSummary TEBOptimizer::optimize(
  TEBTrajectory & trajectory,
  const TEBState & state,
  const nav_msgs::msg::Path & global_plan,
  nav2_costmap_2d::Costmap2DROS & costmap_ros,
  const TEBControllerConfig & cfg) const
{
  TEBOptimizationSummary summary;
  summary.initialized = !trajectory.states.empty();
  if (!summary.initialized) {
    return summary;
  }

  auto * costmap = costmap_ros.getCostmap();
  if (costmap == nullptr) {
    return summary;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = global_plan.header;
  robot_pose.pose.position.x = state.x;
  robot_pose.pose.position.y = state.y;
  robot_pose.pose.orientation.z = std::sin(state.theta * 0.5);
  robot_pose.pose.orientation.w = std::cos(state.theta * 0.5);
  const auto references = buildReferenceBand(global_plan, robot_pose, cfg, costmap);

  // 在正式建图前先做一次离散密度和朝向刷新，避免图初值太差。
  resizeTrajectory(trajectory, cfg);
  refreshOrientations(trajectory);

  // 下面开始搭建 g2o 图：
  // - pose 顶点: band 上的离散几何状态
  // - dt 顶点:   相邻 pose 之间的时间间隔
  //
  // 求解器选择 LM + 稀疏线性求解。
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  auto linear_solver =
    std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
  auto block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver)));

  std::vector<VertexPose2D *> pose_vertices;
  std::vector<VertexTimeDiff *> dt_vertices;
  pose_vertices.reserve(trajectory.states.size());
  dt_vertices.reserve(trajectory.states.size() > 1U ? trajectory.states.size() - 1U : 0U);

  int next_id = 0;
  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    auto * vertex = new VertexPose2D();
    vertex->setId(next_id++);
    vertex->setEstimate(Eigen::Vector3d(
      trajectory.states[i].pose.x,
      trajectory.states[i].pose.y,
      trajectory.states[i].pose.theta));
    // 第一个 pose 固定在当前机器人位姿，相当于整条 TEB 的锚点。
    vertex->setFixed(i == 0U);
    optimizer.addVertex(vertex);
    pose_vertices.push_back(vertex);
  }

  // dt 顶点比 pose 顶点少一个：每个 dt 表示 pose_i -> pose_{i+1} 的时间。
  for (std::size_t i = 1; i < trajectory.states.size(); ++i) {
    auto * vertex = new VertexTimeDiff();
    vertex->setId(next_id++);
    vertex->setEstimate(std::max(0.05, trajectory.states[i].dt));
    optimizer.addVertex(vertex);
    dt_vertices.push_back(vertex);
  }

  for (std::size_t i = 1; i < pose_vertices.size(); ++i) {
    const auto reference = interpolateReference(
      references,
      static_cast<double>(i) * static_cast<double>(references.size() - 1) /
      static_cast<double>(std::max<std::size_t>(pose_vertices.size() - 1, 1)));
    auto * edge = new EdgePosePrior();
    edge->setVertex(0, pose_vertices[i]);
    edge->setMeasurement(Eigen::Vector3d(reference.x, reference.y, reference.theta));
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    information(0, 0) = cfg.weight_goal;
    information(1, 1) = cfg.weight_goal;
    // 中间点只弱约束朝向，末端点则更强地约束到局部目标姿态。
    information(2, 2) = (i + 1 == pose_vertices.size()) ? cfg.weight_goal_heading : 0.15;
    edge->setInformation(information);
    optimizer.addEdge(edge);
  }

  // obstacle edge 不固定目标值，只通过 footprint 代价把顶点推离障碍。
  // 起点是当前机器人位姿并且被固定，不能通过优化移动；末端局部目标也必须参与避障，
  // 否则它被 reference prior 拉到障碍上时，只会在后续 feasibility check 里失败。
  for (std::size_t i = 1; i < pose_vertices.size(); ++i) {
    auto * edge = new EdgeObstacle(*costmap, footprint_, cfg);
    edge->setVertex(0, pose_vertices[i]);
    edge->setMeasurement(0.0);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * cfg.weight_obstacle);
    optimizer.addEdge(edge);
  }

  for (std::size_t i = 0; i + 1 < pose_vertices.size(); ++i) {
    // Kinematics edge 不需要测量值，目标就是把误差压到 0。
    auto * edge = new EdgeKinematics(cfg.allow_backward_motion);
    edge->setVertex(0, pose_vertices[i]);
    edge->setVertex(1, pose_vertices[i + 1]);
    edge->setMeasurement(Eigen::Vector2d::Zero());
    edge->setInformation(Eigen::Matrix2d::Identity() * cfg.weight_kinematics);
    optimizer.addEdge(edge);
  }

  for (std::size_t i = 0; i < dt_vertices.size(); ++i) {
    // 每段时间都希望接近 dt_ref，同时受速度 edge 联合影响，最终实现“时间弹性”。
    auto * time_edge = new EdgeTimeRef();
    time_edge->setVertex(0, dt_vertices[i]);
    time_edge->setMeasurement(cfg.dt_ref);
    time_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * cfg.weight_time);
    optimizer.addEdge(time_edge);

    auto * velocity_edge = new EdgeVelocity(
      cfg.max_linear_velocity, cfg.max_angular_velocity, cfg.allow_backward_motion);
    velocity_edge->setVertex(0, pose_vertices[i]);
    velocity_edge->setVertex(1, pose_vertices[i + 1]);
    velocity_edge->setVertex(2, dt_vertices[i]);
    velocity_edge->setMeasurement(Eigen::Vector2d::Zero());
    velocity_edge->setInformation(Eigen::Matrix2d::Identity() * cfg.weight_velocity);
    optimizer.addEdge(velocity_edge);
  }

  for (std::size_t i = 0; i + 2 < pose_vertices.size(); ++i) {
    // smoothness edge 和 acceleration edge 都跨越 3 个 pose。
    // 前者管几何二阶平滑，后者管时域上的速度变化率。
    auto * smooth_edge = new EdgeSmoothness();
    smooth_edge->setVertex(0, pose_vertices[i]);
    smooth_edge->setVertex(1, pose_vertices[i + 1]);
    smooth_edge->setVertex(2, pose_vertices[i + 2]);
    smooth_edge->setMeasurement(Eigen::Vector2d::Zero());
    smooth_edge->setInformation(Eigen::Matrix2d::Identity() * cfg.weight_smoothness);
    optimizer.addEdge(smooth_edge);

    auto * acceleration_edge = new EdgeAcceleration(
      cfg.max_linear_acceleration, cfg.max_angular_acceleration);
    acceleration_edge->setVertex(0, pose_vertices[i]);
    acceleration_edge->setVertex(1, pose_vertices[i + 1]);
    acceleration_edge->setVertex(2, pose_vertices[i + 2]);
    acceleration_edge->setVertex(3, dt_vertices[i]);
    acceleration_edge->setVertex(4, dt_vertices[i + 1]);
    acceleration_edge->setMeasurement(Eigen::Vector2d::Zero());
    acceleration_edge->setInformation(Eigen::Matrix2d::Identity() * cfg.weight_acceleration);
    optimizer.addEdge(acceleration_edge);
  }

  if (!pose_vertices.empty()) {
    // 再额外加一条终点朝向 edge，保证末端姿态不会被中间点 prior 稀释掉。
    auto * edge = new EdgeGoalHeading();
    edge->setVertex(0, pose_vertices.back());
    edge->setMeasurement(references.back().theta);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * cfg.weight_goal_heading);
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  double previous_chi2 = std::numeric_limits<double>::infinity();
  const int max_outer_iterations = std::max(cfg.max_iterations, 1);
  // 每次只走一步 LM，然后手动检查 chi2 收敛，方便把外层节奏掌握在 controller 自己手里。
  for (int i = 0; i < max_outer_iterations; ++i) {
    optimizer.optimize(1);
    const double chi2 = optimizer.chi2();
    summary.iterations = i + 1;
    if (std::fabs(previous_chi2 - chi2) < cfg.convergence_epsilon) {
      previous_chi2 = chi2;
      break;
    }
    previous_chi2 = chi2;
  }

  // g2o 求解结束后，把顶点估计值回写成工程内部的 TEBTrajectory。
  for (std::size_t i = 0; i < pose_vertices.size(); ++i) {
    const auto & estimate = pose_vertices[i]->estimate();
    trajectory.states[i].pose.x = estimate[0];
    trajectory.states[i].pose.y = estimate[1];
    trajectory.states[i].pose.theta = normalizeAngleLocal(estimate[2]);
  }
  for (std::size_t i = 0; i < dt_vertices.size(); ++i) {
    trajectory.states[i + 1].dt = std::max(0.05, dt_vertices[i]->estimate());
  }

  refreshOrientations(trajectory);
  // 这里的 total_cost 不是 g2o 内部 chi2 的原样暴露，
  // 而是工程侧自己定义的一组可读代价汇总，便于后续调参和日志分析。
  trajectory.total_cost = computeTotalCost(trajectory, state, references, *costmap, cfg);
  trajectory.feasible = isTrajectoryFeasible(trajectory, costmap_ros, cfg, nullptr);
  summary.total_cost = trajectory.total_cost;
  summary.optimized = true;
  summary.feasible = trajectory.feasible;
  return summary;
}

bool TEBOptimizer::isTrajectoryFeasible(
  const TEBTrajectory & trajectory,
  nav2_costmap_2d::Costmap2DROS & costmap_ros,
  const TEBControllerConfig & cfg) const
{
  return isTrajectoryFeasible(trajectory, costmap_ros, cfg, nullptr);
}

bool TEBOptimizer::isTrajectoryFeasible(
  const TEBTrajectory & trajectory,
  nav2_costmap_2d::Costmap2DROS & costmap_ros,
  const TEBControllerConfig & cfg,
  std::string * failure_reason) const
{
  auto set_failure = [&](const std::string & reason) {
      if (failure_reason != nullptr) {
        *failure_reason = reason;
      }
    };

  if (trajectory.states.size() < 2U) {
    set_failure("trajectory has fewer than 2 states");
    return false;
  }

  auto * costmap = costmap_ros.getCostmap();
  if (costmap == nullptr) {
    set_failure("costmap is null");
    return false;
  }

  const std::size_t max_checks = static_cast<std::size_t>(std::max(cfg.feasibility_check_poses, 1));
  const std::size_t step = std::max<std::size_t>(1, trajectory.states.size() / max_checks);
  // feasibility check 比图内 obstacle edge 更偏保守：只要有明显致命风险就直接判失败。
  for (std::size_t i = 0; i < trajectory.states.size(); i += step) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(trajectory.states[i].pose.x, trajectory.states[i].pose.y, mx, my)) {
      set_failure("state " + std::to_string(i) + " is outside local costmap bounds");
      return false;
    }
    const unsigned char cost = costmap->getCost(mx, my);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      set_failure("state " + std::to_string(i) + " center cost is lethal obstacle");
      return false;
    }
    if (cfg.unknown_as_obstacle && cost == nav2_costmap_2d::NO_INFORMATION) {
      set_failure("state " + std::to_string(i) + " center cost is unknown and treated as obstacle");
      return false;
    }
    std::string collision_reason;
    (void)obstaclePenalty(trajectory.states[i].pose, *costmap, cfg, &collision_reason);
    if (!collision_reason.empty()) {
      set_failure(
        "state " + std::to_string(i) +
        " footprint collision detected (" + collision_reason + ")");
      return false;
    }
  }
  set_failure("feasible");
  return true;
}

TEBCommand TEBOptimizer::extractCommand(
  const TEBTrajectory & trajectory,
  const TEBState & state,
  const TEBControllerConfig & cfg) const
{
  TEBCommand command;
  if (trajectory.states.size() < 2U) {
    return command;
  }

  // TEB 输出并不是直接取某个顶点的速度，而是由前两段局部轨迹反推本周期命令。
  const auto & start = trajectory.states[0].pose;
  const auto & next = trajectory.states[1].pose;
  const double dt = std::max(trajectory.states[1].dt, 1.0 / std::max(cfg.control_frequency, 1.0));
  const double dx = next.x - start.x;
  const double dy = next.y - start.y;
  const double travel_heading = std::atan2(dy, dx);
  const double distance = std::hypot(dx, dy);
  const double forward_projection = dx * std::cos(state.theta) + dy * std::sin(state.theta);
  const double sign = (cfg.allow_backward_motion && forward_projection < 0.0) ? -1.0 : 1.0;

  command.v = sign * clamp(distance / dt, cfg.min_linear_velocity, cfg.max_linear_velocity);
  command.w = clamp(
    normalizeAngle(next.theta - state.theta) / dt,
    -cfg.max_angular_velocity,
    cfg.max_angular_velocity);

  if (!cfg.allow_backward_motion) {
    command.v = std::max(0.0, command.v);
  }
  if (std::fabs(normalizeAngle(travel_heading - state.theta)) > M_PI_2) {
    // 如果局部几何方向与当前车头差太大，先明显降速，避免一帧里给出过激前冲命令。
    command.v *= 0.25;
  }
  return command;
}

std::vector<TEBPose> TEBOptimizer::buildReferenceBand(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const TEBControllerConfig & cfg,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  std::vector<TEBPose> references;
  // reference band 不是把整条 global plan 全搬进来，而是只截取机器人前方的局部视野。
  // 这是 TEB 成为“局部优化器”而不是“全局轨迹优化器”的关键。
  references.push_back(
    TEBPose{
      robot_pose.pose.position.x,
      robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)});

  if (global_plan.poses.empty()) {
    return references;
  }

  const double nominal_speed = std::max(cfg.max_linear_velocity * 0.8, 0.1);
  const double target_spacing = std::max(cfg.dt_ref * nominal_speed, 0.1);
  // horizon_limit 决定这次局部优化看多远：
  // 一方面受 local_goal_distance 控制，
  // 另一方面也受“最多允许多少个离散点 * 每点典型间距”控制。
  const double horizon_limit = std::max(
    cfg.local_goal_distance,
    cfg.dt_ref * nominal_speed * static_cast<double>(std::max(cfg.max_samples - 1, 1)) *
      cfg.teb_length_scale);

  double footprint_radius = cfg.robot_radius;
  for (const auto & point : footprint_) {
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }

  const double costmap_resolution = costmap == nullptr ? 0.0 : costmap->getResolution();
  // local costmap 的 worldToMap() 只保证中心点在窗口内；footprint 边界采样还会向外扩一圈。
  // 因此这里把可用窗口按 footprint 半径、安全距离和一个栅格裕量向内收缩，
  // reference band 的末端只允许落在这个“可完整检测 footprint”的区域内。
  const double costmap_margin =
    footprint_radius + cfg.min_obstacle_distance + costmap_resolution;
  const double usable_min_x = costmap == nullptr ?
    -std::numeric_limits<double>::infinity() : costmap->getOriginX() + costmap_margin;
  const double usable_min_y = costmap == nullptr ?
    -std::numeric_limits<double>::infinity() : costmap->getOriginY() + costmap_margin;
  const double usable_max_x = costmap == nullptr ?
    std::numeric_limits<double>::infinity() :
    costmap->getOriginX() + costmap->getSizeInMetersX() - costmap_margin;
  const double usable_max_y = costmap == nullptr ?
    std::numeric_limits<double>::infinity() :
    costmap->getOriginY() + costmap->getSizeInMetersY() - costmap_margin;
  auto isInsideUsableCostmap = [&](const TEBPose & pose) {
      return
        pose.x >= usable_min_x && pose.x <= usable_max_x &&
        pose.y >= usable_min_y && pose.y <= usable_max_y;
    };

  double accumulated = 0.0;
  double next_sample_distance = target_spacing;
  TEBPose previous = references.front();
  TEBPose last_plan_pose = previous;
  bool reached_usable_costmap_boundary = false;

  for (std::size_t i = 0; i < global_plan.poses.size(); ++i) {
    const auto & pose = global_plan.poses[i].pose;
    TEBPose current{pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
    last_plan_pose = current;
    const double segment_length = poseDistance(previous, current);
    if (segment_length < 1.0e-6) {
      previous = current;
      continue;
    }

    while (
      accumulated + segment_length >= next_sample_distance &&
      references.size() + 1 < static_cast<std::size_t>(std::max(cfg.max_samples, 2)))
    {
      // 按固定弧长间距在路径段上插值，得到 TEB 的初始几何骨架。
      const double ratio = safeRatio(next_sample_distance - accumulated, segment_length);
      TEBPose sample;
      sample.x = lerp(previous.x, current.x, ratio);
      sample.y = lerp(previous.y, current.y, ratio);
      sample.theta = segmentHeading(previous, current);
      if (!isInsideUsableCostmap(sample)) {
        reached_usable_costmap_boundary = true;
        break;
      }
      references.push_back(sample);
      next_sample_distance += target_spacing;
    }
    if (reached_usable_costmap_boundary) {
      break;
    }

    accumulated += segment_length;
    previous = current;
    if (accumulated >= horizon_limit) {
      break;
    }
  }

  // 原始全局路径点可能比按弧长插出来的最后一个 sample 更靠前。
  // 如果它已经越过收缩后的可用窗口，就保留最后一个安全 sample 作为局部目标。
  const auto terminal_pose =
    (!reached_usable_costmap_boundary && isInsideUsableCostmap(last_plan_pose)) ?
    last_plan_pose : references.back();
  if (poseDistance(references.back(), terminal_pose) > 0.05) {
    if (references.size() < static_cast<std::size_t>(std::max(cfg.max_samples, 2))) {
      references.push_back(terminal_pose);
    } else {
      references.back() = terminal_pose;
    }
  } else {
    references.back().theta = terminal_pose.theta;
  }

  while (references.size() < static_cast<std::size_t>(std::max(cfg.min_samples, 2))) {
    // 如果 costmap 裁剪后有效参考点不足，用最后一个安全点补齐图规模；
    // 这样 g2o 仍能建图，但不会把越界的局部目标重新塞进 feasibility check。
    references.push_back(references.back());
  }

  return references;
}

void TEBOptimizer::updateTimedElasticBand(
  TEBTrajectory &,
  const std::vector<TEBPose> &,
  nav2_costmap_2d::Costmap2D &,
  const TEBControllerConfig &) const
{
}

void TEBOptimizer::updateTimeDiffs(
  TEBTrajectory &,
  const TEBState &,
  nav2_costmap_2d::Costmap2D &,
  const TEBControllerConfig &) const
{
}

void TEBOptimizer::resizeTrajectory(
  TEBTrajectory & trajectory,
  const TEBControllerConfig & cfg) const
{
  if (trajectory.states.size() < 2U) {
    return;
  }

  // 这是一个轻量的离散自适应步骤：
  // - 某段太长 / dt 太大 -> 在中间插一个点
  // - 某段太短 / dt 太小 -> 合并掉一个点
  //
  // 这样图优化不会长期卡在过粗或过密的离散分辨率上。
  std::vector<TEBTimedPose> resized;
  resized.reserve(static_cast<std::size_t>(cfg.max_samples));
  resized.push_back(trajectory.states.front());

  for (std::size_t i = 1; i < trajectory.states.size(); ++i) {
    const auto & prev = resized.back();
    const auto & curr = trajectory.states[i];
    const double segment_length = poseDistance(prev.pose, curr.pose);
    const double dt = curr.dt;

    if (
      (dt > cfg.dt_ref + cfg.dt_hysteresis ||
      segment_length > std::max(cfg.max_linear_velocity * cfg.dt_ref * 1.5, 0.4)) &&
      resized.size() + 2 <= static_cast<std::size_t>(std::max(cfg.max_samples, 2)))
    {
      TEBTimedPose mid;
      mid.pose.x = 0.5 * (prev.pose.x + curr.pose.x);
      mid.pose.y = 0.5 * (prev.pose.y + curr.pose.y);
      mid.pose.theta = segmentHeading(prev.pose, curr.pose);
      mid.dt = 0.5 * dt;
      resized.push_back(mid);
    } else if (
      dt < std::max(cfg.dt_ref - cfg.dt_hysteresis, 0.05) &&
      segment_length < std::max(cfg.max_linear_velocity * cfg.dt_ref * 0.4, 0.05) &&
      i + 1 < trajectory.states.size() &&
      resized.size() > static_cast<std::size_t>(std::max(cfg.min_samples, 2)))
    {
      continue;
    }

    resized.push_back(curr);
  }

  if (resized.size() >= static_cast<std::size_t>(std::max(cfg.min_samples, 2))) {
    trajectory.states.swap(resized);
  }
}

void TEBOptimizer::refreshOrientations(TEBTrajectory & trajectory) const
{
  if (trajectory.states.size() < 2U) {
    return;
  }

  // 这里用相邻几何点重新刷新 pose.theta，
  // 让“位姿朝向”和“轨迹切向方向”保持一致，减少图优化后的姿态漂移。
  for (std::size_t i = 0; i + 1 < trajectory.states.size(); ++i) {
    const double dx = trajectory.states[i + 1].pose.x - trajectory.states[i].pose.x;
    const double dy = trajectory.states[i + 1].pose.y - trajectory.states[i].pose.y;
    if (std::hypot(dx, dy) > 1.0e-6) {
      trajectory.states[i].pose.theta = std::atan2(dy, dx);
    }
  }
}

double TEBOptimizer::computeTotalCost(
  const TEBTrajectory & trajectory,
  const TEBState & state,
  const std::vector<TEBPose> & references,
  nav2_costmap_2d::Costmap2D & costmap,
  const TEBControllerConfig & cfg) const
{
  if (trajectory.states.empty() || references.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  // 这组代价不是直接参与 g2o 求解的 Jacobian 计算，
  // 而是工程侧对“这条优化后 TEB 看起来好不好”的统一评分。
  // 它主要用于日志、调试、以及后续如果要做多候选比较时的外部打分。
  double path_cost = 0.0;
  double obstacle_cost = 0.0;
  double time_cost = 0.0;
  double smooth_cost = 0.0;

  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    const auto reference = interpolateReference(
      references,
      static_cast<double>(i) * static_cast<double>(references.size() - 1) /
      static_cast<double>(std::max<std::size_t>(trajectory.states.size() - 1, 1)));
    path_cost += poseDistance(trajectory.states[i].pose, reference);
    obstacle_cost += obstaclePenalty(trajectory.states[i].pose, costmap, cfg);
    if (i > 0) {
      time_cost += std::fabs(trajectory.states[i].dt - cfg.dt_ref);
    }
    if (i > 0 && i + 1 < trajectory.states.size()) {
      const double midpoint_x =
        0.5 * (trajectory.states[i - 1].pose.x + trajectory.states[i + 1].pose.x);
      const double midpoint_y =
        0.5 * (trajectory.states[i - 1].pose.y + trajectory.states[i + 1].pose.y);
      smooth_cost += std::hypot(
        trajectory.states[i].pose.x - midpoint_x,
        trajectory.states[i].pose.y - midpoint_y);
    }
  }

  return
    cfg.weight_goal * path_cost +
    cfg.weight_goal_heading * goalHeadingCost(trajectory, references.back()) +
    cfg.weight_obstacle * obstacle_cost +
    cfg.weight_time * time_cost +
    cfg.weight_smoothness * smooth_cost +
    cfg.weight_velocity * velocityCost(trajectory, state, cfg) +
    cfg.weight_acceleration * accelerationCost(trajectory, state, cfg) +
    cfg.weight_kinematics * kinematicsCost(trajectory, cfg);
}

double TEBOptimizer::goalHeadingCost(
  const TEBTrajectory & trajectory,
  const TEBPose & goal_pose) const
{
  if (trajectory.states.empty()) {
    return 0.0;
  }
  return std::fabs(normalizeAngle(goal_pose.theta - trajectory.states.back().pose.theta));
}

double TEBOptimizer::velocityCost(
  const TEBTrajectory & trajectory,
  const TEBState &,
  const TEBControllerConfig & cfg) const
{
  double cost = 0.0;
  // 逐段检查速度上限，和图里的 EdgeVelocity 语义保持一致，
  // 但这里给的是便于解释和日志统计的外部代价。
  for (std::size_t i = 1; i < trajectory.states.size(); ++i) {
    const double distance = poseDistance(trajectory.states[i - 1].pose, trajectory.states[i].pose);
    const double dt = std::max(trajectory.states[i].dt, 1.0e-6);
    const double linear_speed = distance / dt;
    const double angular_speed = std::fabs(
      normalizeAngle(trajectory.states[i].pose.theta - trajectory.states[i - 1].pose.theta)) / dt;
    cost += std::max(0.0, linear_speed - cfg.max_linear_velocity);
    cost += std::max(0.0, angular_speed - cfg.max_angular_velocity);
  }
  return cost;
}

double TEBOptimizer::accelerationCost(
  const TEBTrajectory & trajectory,
  const TEBState & state,
  const TEBControllerConfig & cfg) const
{
  if (trajectory.states.size() < 2U) {
    return 0.0;
  }

  // 从当前实测速度出发，逐段估计 band 上的线速度/角速度变化率。
  double cost = 0.0;
  double previous_v = std::fabs(state.v);
  double previous_w = std::fabs(state.w);
  for (std::size_t i = 1; i < trajectory.states.size(); ++i) {
    const double distance = poseDistance(trajectory.states[i - 1].pose, trajectory.states[i].pose);
    const double dt = std::max(trajectory.states[i].dt, 1.0e-6);
    const double current_v = distance / dt;
    const double current_w = std::fabs(
      normalizeAngle(trajectory.states[i].pose.theta - trajectory.states[i - 1].pose.theta)) / dt;
    cost += std::max(0.0, std::fabs(current_v - previous_v) / dt - cfg.max_linear_acceleration);
    cost += std::max(0.0, std::fabs(current_w - previous_w) / dt - cfg.max_angular_acceleration);
    previous_v = current_v;
    previous_w = current_w;
  }
  return cost;
}

double TEBOptimizer::kinematicsCost(
  const TEBTrajectory & trajectory,
  const TEBControllerConfig & cfg) const
{
  double cost = 0.0;
  // 这里主要看三类违反：
  // 1. 轨迹切向和车体朝向不一致
  // 2. 横向滑移过大
  // 3. 不允许倒车时仍出现倒退分量
  for (std::size_t i = 1; i < trajectory.states.size(); ++i) {
    const auto & prev = trajectory.states[i - 1].pose;
    const auto & curr = trajectory.states[i].pose;
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1.0e-6) {
      continue;
    }
    const double heading = segmentHeading(prev, curr);
    cost += std::fabs(normalizeAngle(heading - prev.theta));
    cost += std::fabs(-dx * std::sin(prev.theta) + dy * std::cos(prev.theta)) / distance;
    if (!cfg.allow_backward_motion) {
      const double forward_projection = dx * std::cos(prev.theta) + dy * std::sin(prev.theta);
      cost += std::max(0.0, -forward_projection);
    }
  }
  return cost;
}

double TEBOptimizer::obstaclePenalty(
  const TEBPose & pose,
  nav2_costmap_2d::Costmap2D & costmap,
  const TEBControllerConfig & cfg) const
{
  // 对外统一走 footprint 版本 obstacle 模型，避免图内图外出现两套障碍语义。
  return computeObstaclePenaltyLocal(pose, footprint_, costmap, cfg, nullptr);
}

double TEBOptimizer::obstaclePenalty(
  const TEBPose & pose,
  nav2_costmap_2d::Costmap2D & costmap,
  const TEBControllerConfig & cfg,
  std::string * debug_reason) const
{
  return computeObstaclePenaltyLocal(pose, footprint_, costmap, cfg, debug_reason);
}

std::pair<double, double> TEBOptimizer::obstacleGradient(
  const TEBPose & pose,
  nav2_costmap_2d::Costmap2D & costmap,
  const TEBControllerConfig & cfg) const
{
  // 这里没有为 obstacle edge 手写解析 Jacobian，
  // 这个梯度函数只服务于外部代价评估/调试，可理解成数值微分近似。
  const double step = std::max(costmap.getResolution(), 0.05);
  const TEBPose x_plus{pose.x + step, pose.y, pose.theta};
  const TEBPose x_minus{pose.x - step, pose.y, pose.theta};
  const TEBPose y_plus{pose.x, pose.y + step, pose.theta};
  const TEBPose y_minus{pose.x, pose.y - step, pose.theta};

  const double grad_x =
    computeObstaclePenaltyLocal(x_minus, footprint_, costmap, cfg) -
    computeObstaclePenaltyLocal(x_plus, footprint_, costmap, cfg);
  const double grad_y =
    computeObstaclePenaltyLocal(y_minus, footprint_, costmap, cfg) -
    computeObstaclePenaltyLocal(y_plus, footprint_, costmap, cfg);
  return {grad_x, grad_y};
}

TEBPose TEBOptimizer::interpolateReference(
  const std::vector<TEBPose> & references,
  double index) const
{
  if (references.empty()) {
    return {};
  }
  if (index <= 0.0) {
    return references.front();
  }
  const double max_index = static_cast<double>(references.size() - 1);
  if (index >= max_index) {
    return references.back();
  }

  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double ratio = index - static_cast<double>(lower);

  // 按 band 索引而不是按真实弧长做参考插值。
  // 这样实现简单，也足够支撑当前这版局部 reference prior。
  TEBPose pose;
  pose.x = lerp(references[lower].x, references[upper].x, ratio);
  pose.y = lerp(references[lower].y, references[upper].y, ratio);
  pose.theta = normalizeAngle(
    references[lower].theta +
    ratio * normalizeAngle(references[upper].theta - references[lower].theta));
  return pose;
}

double TEBOptimizer::poseDistance(const TEBPose & lhs, const TEBPose & rhs) const
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double TEBOptimizer::normalizeAngle(double angle) const
{
  return normalizeAngleLocal(angle);
}

}  // namespace rmp::controller
