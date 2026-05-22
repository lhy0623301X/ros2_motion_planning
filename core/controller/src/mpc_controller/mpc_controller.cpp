/**
 * @file mpc_controller.cpp
 * @brief 基于线性时变 MPC 的局部轨迹跟踪控制器。
 *
 * 控制流程概览：
 *   1. prunePlan()          – 根据机器人当前位置裁剪已走过的全局路径
 *   2. computeReferencePoint() – 在前视距离处计算参考点 (x,y,θ) 与参考曲率 κ
 *   3. mpcControl()         – 构造并求解带约束的二次规划 (QP)
 *   4. linearRegularization() / angularRegularization() – 对 QP 输出做速度增量平滑
 *
 * MPC 问题形式（增广状态空间）：
 *   状态误差: e = [x - x_ref, y - y_ref, θ - θ_ref]^T
 *   控制增量: Δu_k = u_k - u_{k-1} （上一周期控制误差 du_p 作为增广状态的一部分）
 *   目标: min Σ( e_i^T Q e_i + Δu_i^T R Δu_i )
 *   约束: u_min ≤ u_ref + du_p + ΣΔu ≤ u_max
 *         Δu_min ≤ Δu ≤ Δu_max
 */
#include "mpc_controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <OsqpEigen/OsqpEigen.h>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>

#include "geometry/angles.h"
#include "geometry/point.h"
#include "geometry/vec2d.h"
#include "math/math_helper.h"
#include "tf2/utils.h"
#include "util/log.h"

namespace rmp::controller {

namespace {

/// 状态向量维度: [x, y, θ]^T
constexpr int kStateDim = 3;
/// 控制向量维度: [v, ω]^T
constexpr int kControlDim = 2;

/**
 * @brief 将值限定在 [low, high] 区间内。
 */
double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

/**
 * @brief 计算两个 PoseStamped 之间的平面欧氏距离。
 */
double planarDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

}  // namespace

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  parameter_prefix_ = plugin_name + "." + controller_name + ".";
  controller_name_ = controller_name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  readParameters();
  // 保存额定最大线速度，setSpeedLimit() 会基于此做百分比限速
  nominal_max_linear_velocity_ = cfg_.max_linear_velocity;
  lookahead_point_publisher_ =
    std::make_unique<utils::LookaheadPointPublisher>(node_, "lookahead_point");
  predicted_trajectory_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("mpc_predicted_trajectory", 1);

  // 构造状态误差权重矩阵 Q = diag(q_x, q_y, q_θ)
  // Q 越大表示对该状态分量的跟踪精度要求越高
  Q_.setZero();
  Q_(0, 0) = cfg_.q_matrix_x;
  Q_(1, 1) = cfg_.q_matrix_y;
  Q_(2, 2) = cfg_.q_matrix_theta;

  // 构造控制增量权重矩阵 R = diag(r_v, r_ω)
  // R 越大表示对控制输入的平滑性要求越高（惩罚大幅控制变化）
  R_.setZero();
  R_(0, 0) = cfg_.r_matrix_v;
  R_(1, 1) = cfg_.r_matrix_w;

  RCLCPP_INFO(node_->get_logger(), "%s configured in MPC tracking mode.", controller_name_.c_str());
}

void MPCController::cleanup()
{
  global_plan_.poses.clear();
  previous_control_error_.setZero();
}

void MPCController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "MPCController activated.");
}

void MPCController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "MPCController deactivated.");
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  // 路径更新后重置上一周期控制误差，避免旧误差污染新路径的 MPC 初始状态
  previous_control_error_.setZero();
  RCLCPP_INFO(node_->get_logger(), "MPCController received path with %zu poses.", path.poses.size());
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    AWARN << "[MPCController] computeVelocityCommands skipped: global plan is empty.";
    return cmd;
  }

  // 步骤 1：裁剪已经走过的路径，并用当前速度计算本周期前视距离。
  // 前视距离 = clamp(v * lookahead_time, min_lookahead_dist, max_lookahead_dist)
  // 速度越大看得越远，但限制在 [min, max] 范围内保证稳定性。
  prunePlan(pose);
  const double current_v = std::hypot(velocity.linear.x, velocity.linear.y);
  const double current_w = velocity.angular.z;
  const double current_yaw = tf2::getYaw(pose.pose.orientation);
  const double lookahead_dist = clamp(
    current_v * cfg_.lookahead_time,
    cfg_.min_lookahead_dist,
    cfg_.max_lookahead_dist);

  // 步骤 2：从局部路径提取 MPC 跟踪参考状态 s_d = [x_ref, y_ref, θ_ref]^T 和参考曲率 κ。
  // u_r[0] = current_v 作为参考线速度（恒速假设），u_r[1] = current_v * κ 作为参考角速度。
  const auto ref = computeReferencePoint(pose, lookahead_dist);
  if (lookahead_point_publisher_) {
    const auto frame_id = global_plan_.header.frame_id.empty() ?
      (pose.header.frame_id.empty() ? "map" : pose.header.frame_id) :
      global_plan_.header.frame_id;
    lookahead_point_publisher_->publish(ref.x, ref.y, ref.theta, frame_id, node_->now());
  }

  Eigen::Vector3d s(pose.pose.position.x, pose.pose.position.y, current_yaw);
  Eigen::Vector3d s_d(ref.x, ref.y, ref.theta);
  Eigen::Vector2d u_r(current_v, current_v * ref.kappa);

  // 步骤 3：调用 QP 型 MPC 求解器得到最优控制增量序列，
  // 取第一个增量与参考控制叠加得到本周期期望控制量，
  // 再经 linearRegularization/angularRegularization 做速度增量约束平滑。
  const auto control_result = mpcControl(s, s_d, u_r, previous_control_error_);
  publishPredictedTrajectory(
    control_result.predicted_points,
    pose.header.frame_id.empty() ? "map" : pose.header.frame_id,
    node_->now());

  const double u_v = linearRegularization(velocity.linear.x, control_result.command[0]);
  const double u_w = angularRegularization(current_w, control_result.command[1]);
  // 保存本周期控制误差 du_p = u_actual - u_ref，作为下一周期 MPC 的增广状态初值
  previous_control_error_ = Eigen::Vector2d(
    u_v - u_r[0],
    rmp::common::geometry::normalizeAngle(u_w - u_r[1]));

  cmd.twist.linear.x = u_v;
  cmd.twist.angular.z = u_w;
  return cmd;
}

void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    // 非正限速值视为恢复默认速度
    cfg_.max_linear_velocity = nominal_max_linear_velocity_;
    return;
  }
  cfg_.max_linear_velocity = percentage ?
    nominal_max_linear_velocity_ * speed_limit / 100.0 :
    speed_limit;
}

void MPCController::readParameters()
{
  cfg_.control_frequency = node_->declare_parameter<double>(
    parameter_prefix_ + "control_frequency", cfg_.control_frequency);
  cfg_.goal_dist_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "goal_dist_tolerance", cfg_.goal_dist_tolerance);
  cfg_.rotate_tolerance = node_->declare_parameter<double>(
    parameter_prefix_ + "rotate_tolerance", cfg_.rotate_tolerance);

  cfg_.max_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity", cfg_.max_linear_velocity);
  cfg_.min_linear_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_linear_velocity", cfg_.min_linear_velocity);
  cfg_.max_linear_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_linear_velocity_increment", cfg_.max_linear_velocity_increment);
  cfg_.max_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity", cfg_.max_angular_velocity);
  cfg_.min_angular_velocity = node_->declare_parameter<double>(
    parameter_prefix_ + "min_angular_velocity", cfg_.min_angular_velocity);
  cfg_.max_angular_velocity_increment = node_->declare_parameter<double>(
    parameter_prefix_ + "max_angular_velocity_increment", cfg_.max_angular_velocity_increment);

  cfg_.lookahead_time = node_->declare_parameter<double>(
    parameter_prefix_ + "lookahead_time", cfg_.lookahead_time);
  cfg_.min_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "min_lookahead_dist", cfg_.min_lookahead_dist);
  cfg_.max_lookahead_dist = node_->declare_parameter<double>(
    parameter_prefix_ + "max_lookahead_dist", cfg_.max_lookahead_dist);

  cfg_.predict_time_domain = node_->declare_parameter<int>(
    parameter_prefix_ + "predict_time_domain", cfg_.predict_time_domain);
  cfg_.control_time_domain = node_->declare_parameter<int>(
    parameter_prefix_ + "control_time_domain", cfg_.control_time_domain);
  // 控制增量约束：限制相邻周期 v/ω 的变化幅度
  cfg_.delta_linear_velocity_min = node_->declare_parameter<double>(
    parameter_prefix_ + "delta_linear_velocity_min", cfg_.delta_linear_velocity_min);
  cfg_.delta_linear_velocity_max = node_->declare_parameter<double>(
    parameter_prefix_ + "delta_linear_velocity_max", cfg_.delta_linear_velocity_max);
  cfg_.delta_angular_velocity_min = node_->declare_parameter<double>(
    parameter_prefix_ + "delta_angular_velocity_min", cfg_.delta_angular_velocity_min);
  cfg_.delta_angular_velocity_max = node_->declare_parameter<double>(
    parameter_prefix_ + "delta_angular_velocity_max", cfg_.delta_angular_velocity_max);

  cfg_.q_matrix_x = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_x", cfg_.q_matrix_x);
  cfg_.q_matrix_y = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_y", cfg_.q_matrix_y);
  cfg_.q_matrix_theta = node_->declare_parameter<double>(
    parameter_prefix_ + "q_matrix_theta", cfg_.q_matrix_theta);
  cfg_.r_matrix_v = node_->declare_parameter<double>(
    parameter_prefix_ + "r_matrix_v", cfg_.r_matrix_v);
  cfg_.r_matrix_w = node_->declare_parameter<double>(
    parameter_prefix_ + "r_matrix_w", cfg_.r_matrix_w);
}

void MPCController::prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.size() < 2) {
    return;
  }

  // 搜索范围：代价地图半宽（如果可用），覆盖机器人前方可感知区域
  const double search_distance = costmap_ros_ && costmap_ros_->getCostmap() ?
    costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0 :
    std::numeric_limits<double>::max();

  // 沿路径累积距离，确定搜索终点（不超过 search_distance）
  auto search_end = global_plan_.poses.end();
  double integrated = 0.0;
  for (auto it = global_plan_.poses.begin(); it + 1 != global_plan_.poses.end(); ++it) {
    integrated += planarDistance(*it, *(it + 1));
    if (integrated > search_distance) {
      search_end = it + 1;
      break;
    }
  }

  // 在搜索范围内找到距离机器人最近的路径点
  auto closest = std::min_element(
    global_plan_.poses.begin(), search_end,
    [&](const auto & lhs, const auto & rhs) {
      return planarDistance(robot_pose, lhs) < planarDistance(robot_pose, rhs);
    });

  // 删除最近点之前的所有路径点（机器人已走过）
  if (closest != global_plan_.poses.begin() && closest != global_plan_.poses.end()) {
    global_plan_.poses.erase(global_plan_.poses.begin(), closest);
  }
}

MPCController::ReferencePoint MPCController::computeReferencePoint(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_dist) const
{
  if (global_plan_.poses.empty()) {
    throw std::invalid_argument("Cannot compute MPC reference point from an empty path.");
  }

  using rmp::common::geometry::Point3d;
  using rmp::common::geometry::Vec2d;
  using rmp::common::math::arcCenter;
  using rmp::common::math::circleSegmentIntersection;

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  // 沿路径找到第一个距离机器人 ≥ lookahead_dist 的路径点
  auto goal_pose_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & ps) {
      return std::hypot(
        ps.pose.position.x - rx,
        ps.pose.position.y - ry) >= lookahead_dist;
    });

  Point3d lookahead_pt;
  double kappa = 0.0;

  if (goal_pose_it == global_plan_.poses.end()) {
    // 全局路径终点在前视距离内，直接用路径终点作为参考点
    goal_pose_it = std::prev(global_plan_.poses.end());
    lookahead_pt.setX(goal_pose_it->pose.position.x);
    lookahead_pt.setY(goal_pose_it->pose.position.y);
    lookahead_pt.setTheta(std::atan2(lookahead_pt.y() - ry, lookahead_pt.x() - rx));
  } else {
    double px;
    double py;
    const double gx = goal_pose_it->pose.position.x;
    const double gy = goal_pose_it->pose.position.y;
    if (goal_pose_it == global_plan_.poses.begin()) {
      // 第一个路径点就满足前视距离，以上一个点为机器人当前位置
      px = rx;
      py = ry;
    } else {
      auto prev_pose_it = std::prev(goal_pose_it);
      px = prev_pose_it->pose.position.x;
      py = prev_pose_it->pose.position.y;
    }

    // 步骤 2.1：求路径段与前视圆的精确交点，使参考点位于真实前视距离附近。
    // 将坐标转换到以机器人为原点的局部坐标系，简化圆形交点计算。
    Vec2d prev_p(px - rx, py - ry);
    Vec2d goal_p(gx - rx, gy - ry);
    const auto intersection_points =
      circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    // 取距离目标路径点最近的交点作为前视参考点
    double dist_to_goal = std::numeric_limits<double>::max();
    for (const auto & p : intersection_points) {
      const double dist = std::hypot(p.x() + rx - gx, p.y() + ry - gy);
      if (dist < dist_to_goal) {
        dist_to_goal = dist;
        lookahead_pt.setX(p.x() + rx);
        lookahead_pt.setY(p.y() + ry);
      }
    }

    auto next_pose_it = std::next(goal_pose_it);
    if (next_pose_it != global_plan_.poses.end()) {
      // 步骤 2.2：用相邻三点 (prev, goal, next) 估计路径曲率 κ。
      // κ = 1/R，其中 R 为三点外接圆半径。
      // w_ref = v_ref × κ，将曲率信息送入 MPC 作为前馈参考角速度。
      Vec2d p1(px, py);
      Vec2d p2(gx, gy);
      Vec2d p3(next_pose_it->pose.position.x, next_pose_it->pose.position.y);
      kappa = arcCenter(p1, p2, p3, false);
      if (!std::isfinite(kappa)) {
        kappa = 0.0;
      }
    }
    lookahead_pt.setTheta(std::atan2(lookahead_pt.y() - ry, lookahead_pt.x() - rx));
  }

  return {lookahead_pt.x(), lookahead_pt.y(), lookahead_pt.theta(), kappa};
}

MPCController::ControlResult MPCController::mpcControl(
  const Eigen::Vector3d & s,
  const Eigen::Vector3d & s_d,
  const Eigen::Vector2d & u_r,
  const Eigen::Vector2d & du_p) const
{
  using rmp::common::geometry::normalizeAngle;

  const int predict_horizon = std::max(1, cfg_.predict_time_domain);
  const int control_horizon = std::max(1, cfg_.control_time_domain);
  // 增广状态 = 原始状态误差(3) + 上一周期控制误差(2) = 5
  const int augmented_dim = kStateDim + kControlDim;
  // 优化变量维度: 控制维度 × 控制时域（每个控制步输出一个 Δu_k）
  const int variable_dim = kControlDim * control_horizon;
  // 约束维度: 上半部分约束累积控制量 U，下半部分约束单步增量 ΔU
  const int constraint_dim = 2 * variable_dim;
  const double dt = 1.0 / std::max(1.0, cfg_.control_frequency);

  // 步骤 3.1：构造增广状态向量 x = [e; du_p]。
  //   e = s - s_d 为当前状态与参考状态的误差（位置 + 航向）
  //   du_p = u_{k-1} - u_{r,k-1} 为上一周期的实际控制与参考控制的偏差
  Eigen::VectorXd x = Eigen::VectorXd::Zero(augmented_dim);
  x.topRows(kStateDim) = s - s_d;
  x[2] = normalizeAngle(x[2]);
  x.bottomRows(kControlDim) = du_p;

  // 步骤 3.2：围绕参考轨迹对运动学模型一阶泰勒展开，得到线性时变模型。
  //
  // 自行车模型的连续时间运动学 (简化, 忽略侧滑):
  //   ẋ = v cos θ
  //   ẏ = v sin θ
  //   θ̇ = ω
  //
  // 围绕参考状态 (x_ref, y_ref, θ_ref) 和参考控制 (v_ref, ω_ref) 线性化，
  // 再以 dt 离散化，得到离散状态空间模型:
  //   e_{k+1} = A_o e_k + B_o Δu_k
  //
  // 其中 A_o 和 B_o 由参考航向 θ_ref 和参考线速度 v_ref 决定。
  Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
  A_o(0, 2) = -u_r[0] * std::sin(s_d[2]) * dt;
  A_o(1, 2) = u_r[0] * std::cos(s_d[2]) * dt;

  Eigen::Matrix<double, kStateDim, kControlDim> B_o =
    Eigen::Matrix<double, kStateDim, kControlDim>::Zero();
  B_o(0, 0) = std::cos(s_d[2]) * dt;
  B_o(1, 0) = std::sin(s_d[2]) * dt;
  B_o(2, 1) = dt;

  // 增广系统矩阵：将控制误差 du 也纳入状态空间。
  //
  // 增广状态转移方程:
  //   ┌        ┐   ┌           ┐ ┌        ┐   ┌     ┐
  //   │ e_{k+1} │ = │ A_o   B_o │ │  e_k   │ + │ B_o │ Δu_k
  //   │ du_{k+1} │   │  0    I  │ │ du_k   │   │  I  │
  //   └        ┘   └           ┘ └        ┘   └     ┘
  //
  // 输出方程 (仅观测状态误差):
  //   y_k = [I_3  0] [e_k; du_k]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(augmented_dim, augmented_dim);
  A.topLeftCorner(kStateDim, kStateDim) = A_o;
  A.topRightCorner(kStateDim, kControlDim) = B_o;
  A.bottomRightCorner(kControlDim, kControlDim) = Eigen::Matrix2d::Identity();

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(augmented_dim, kControlDim);
  B.topRows(kStateDim) = B_o;
  B.bottomRows(kControlDim) = Eigen::Matrix2d::Identity();

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(kStateDim, augmented_dim);
  C.topLeftCorner(kStateDim, kStateDim) = Eigen::Matrix3d::Identity();

  // 步骤 3.3：展开预测矩阵 S_x / S_u。
  //
  // 将未来 N 步的预测输出（状态误差）写成当前增广状态与未来控制增量序列的线性组合:
  //   Y = S_x · x + S_u · ΔU
  //
  // 其中:
  //   Y   = [y_1^T, y_2^T, ..., y_N^T]^T                    (3N × 1)
  //   S_x = [CA; CA^2; ...; CA^N]^T                           (3N × 5)
  //   S_u = 下三角 Toeplitz 矩阵，由 CA^{i-j}B 组成          (3N × 2M)
  //   x   = 当前增广状态                                       (5 × 1)
  //   ΔU  = [Δu_0^T, Δu_1^T, ..., Δu_{M-1}^T]^T             (2M × 1)
  //
  // N = predict_horizon (预测时域), M = control_horizon (控制时域)
  Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(kStateDim * predict_horizon, augmented_dim);
  for (int i = 0; i < predict_horizon; ++i) {
    S_x.middleRows(kStateDim * i, kStateDim) = C * A_pow(i + 1);
  }

  Eigen::MatrixXd S_u =
    Eigen::MatrixXd::Zero(kStateDim * predict_horizon, variable_dim);
  for (int i = 0; i < predict_horizon; ++i) {
    for (int j = 0; j < control_horizon; ++j) {
      if (j <= i) {
        S_u.block(kStateDim * i, kControlDim * j, kStateDim, kControlDim) =
          C * A_pow(i - j) * B;
      }
    }
  }

  // 步骤 3.4：构造二次规划问题。
  //
  // 代价函数: J = Y^T Q_bar Y + ΔU^T R_bar ΔU
  // 代入 Y = S_x x + S_u ΔU，展开并忽略与 ΔU 无关的常数项，得到:
  //   J = ΔU^T (S_u^T Q_bar S_u + R_bar) ΔU + 2 x^T S_x^T Q_bar S_u ΔU
  //
  // 标准 QP 形式: min 1/2 ΔU^T P ΔU + q^T ΔU
  //   P = S_u^T Q_bar S_u + R_bar    (Hessian, 保证半正定)
  //   q = S_u^T Q_bar S_x x          (梯度向量)
  //
  // Q_bar = I_N ⊗ Q (Kronecker 积展开为块对角矩阵)
  // R_bar = I_M ⊗ R
  const Eigen::MatrixXd Q = Eigen::kroneckerProduct(
    Eigen::MatrixXd::Identity(predict_horizon, predict_horizon), Q_);
  const Eigen::MatrixXd R = Eigen::kroneckerProduct(
    Eigen::MatrixXd::Identity(control_horizon, control_horizon), R_);
  Eigen::MatrixXd P = S_u.transpose() * Q * S_u + R;
  // 对称化 Hessian，消除数值误差导致的微小不对称
  P = 0.5 * (P + P.transpose());
  const Eigen::VectorXd q = S_u.transpose() * Q * S_x * x;

  // 步骤 3.5：构造不等式约束 A_constraints · ΔU ≤ [upper; -lower]。
  //
  // 两部分约束（按行拼接）：
  //   1. 速度绝对值约束（上半部分）:
  //      对每个控制步 k: u_min - u_r - du_p ≤ Σ_{i=0}^{k} Δu_i ≤ u_max - u_r - du_p
  //      意义: 实际控制量 u_k = u_r + du_p + ΣΔu 必须在 [u_min, u_max] 内
  //
  //   2. 控制增量约束（下半部分）:
  //      对每个控制步 k: Δu_min ≤ Δu_k ≤ Δu_max
  //      意义: 限制相邻周期之间的控制量变化幅度
  //
  // A_constraints 结构:
  //   ┌                                    ┐
  //   │  I  O  O ... O  (累积和矩阵, 下三角) │  ← 速度约束
  //   │  I  I  O ... O                     │
  //   │  ...............                   │
  //   │  I  I  I ... I                     │
  //   │  ─────────────────                  │
  //   │  I  O  O ... O  (单位矩阵)          │  ← 增量约束
  //   │  O  I  O ... O                     │
  //   │  ...............                   │
  //   │  O  O  O ... I                     │
  //   └                                    ┘
  Eigen::Vector2d u_min(cfg_.min_linear_velocity, -cfg_.max_angular_velocity);
  Eigen::Vector2d u_max(cfg_.max_linear_velocity, cfg_.max_angular_velocity);
  Eigen::Vector2d du_min(cfg_.delta_linear_velocity_min, cfg_.delta_angular_velocity_min);
  Eigen::Vector2d du_max(cfg_.delta_linear_velocity_max, cfg_.delta_angular_velocity_max);

  Eigen::VectorXd lower = Eigen::VectorXd::Zero(constraint_dim);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(constraint_dim);
  for (int i = 0; i < control_horizon; ++i) {
    lower.segment(kControlDim * i, kControlDim) = u_min - du_p - u_r;
    upper.segment(kControlDim * i, kControlDim) = u_max - du_p - u_r;
    lower.segment(variable_dim + kControlDim * i, kControlDim) = du_min;
    upper.segment(variable_dim + kControlDim * i, kControlDim) = du_max;
  }

  Eigen::MatrixXd A_constraints = Eigen::MatrixXd::Zero(constraint_dim, variable_dim);
  for (int col = 0; col < control_horizon; ++col) {
    for (int row = col; row < control_horizon; ++row) {
      // 上半部分: 下三角单位矩阵（累积和）
      A_constraints.block(kControlDim * row, kControlDim * col, kControlDim, kControlDim) =
        Eigen::Matrix2d::Identity();
    }
    // 下半部分: 块对角单位矩阵（单步约束）
    A_constraints.block(
      variable_dim + kControlDim * col,
      kControlDim * col,
      kControlDim,
      kControlDim) = Eigen::Matrix2d::Identity();
  }

  // 步骤 3.6：使用 OsqpEigen 求解 QP。OSQP 使用 ADMM 算法，适合中小规模稀疏 QP。
  // 若求解失败，返回零控制量，由外层限幅平滑停车。
  Eigen::SparseMatrix<double> hessian = P.sparseView();
  Eigen::SparseMatrix<double> linear_matrix = A_constraints.sparseView();
  Eigen::VectorXd gradient = q;
  hessian.makeCompressed();
  linear_matrix.makeCompressed();

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  // 热启动：利用上一周期的解作为本周期迭代初值，加速收敛
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(variable_dim);
  solver.data()->setNumberOfConstraints(constraint_dim);
  if (!solver.data()->setHessianMatrix(hessian) ||
    !solver.data()->setGradient(gradient) ||
    !solver.data()->setLinearConstraintsMatrix(linear_matrix) ||
    !solver.data()->setLowerBound(lower) ||
    !solver.data()->setUpperBound(upper) ||
    !solver.initSolver())
  {
    AWARN << "[MPCController] failed to initialize OSQP problem.";
    return {};
  }

  const auto status = solver.solveProblem();
  if (status != OsqpEigen::ErrorExitFlag::NoError) {
    AWARN << "[MPCController] failed to solve OSQP problem.";
    return {};
  }

  const auto solution = solver.getSolution();
  ControlResult result;
  // 取第一个控制增量 Δu_0，叠加参考控制与上一周期误差补偿，得到实际控制指令:
  //   u = u_r + du_p + Δu_0
  result.command = Eigen::Vector2d(
    solution[0] + du_p[0] + u_r[0],
    normalizeAngle(solution[1] + du_p[1] + u_r[1]));

  // 步骤 3.7：沿最优控制增量序列 roll-out 预测轨迹，用于 RViz 可视化调试。
  // 轨迹以蓝色 LINE_STRIP 显示，可直观观察 MPC 的预测行为。
  Eigen::Vector3d predicted_state = s;
  Eigen::Vector2d accumulated_delta = Eigen::Vector2d::Zero();
  Eigen::Vector2d predicted_control = du_p + u_r;
  result.predicted_points.reserve(static_cast<std::size_t>(predict_horizon + 1));
  geometry_msgs::msg::Point start_point;
  start_point.x = predicted_state[0];
  start_point.y = predicted_state[1];
  start_point.z = 0.06;
  result.predicted_points.push_back(start_point);
  for (int i = 0; i < predict_horizon; ++i) {
    if (i < control_horizon) {
      // 控制时域内：累积 QP 求解的增量
      accumulated_delta += solution.segment(kControlDim * i, kControlDim);
      predicted_control = u_r + du_p + accumulated_delta;
    }
    // 控制时域外：保持最后一个控制量不变（零阶保持）
    predicted_control[0] = clamp(
      predicted_control[0],
      -cfg_.max_linear_velocity,
      cfg_.max_linear_velocity);
    predicted_control[1] = clamp(
      predicted_control[1],
      -cfg_.max_angular_velocity,
      cfg_.max_angular_velocity);

    // 自行车模型前向积分一步
    predicted_state[0] += predicted_control[0] * std::cos(predicted_state[2]) * dt;
    predicted_state[1] += predicted_control[0] * std::sin(predicted_state[2]) * dt;
    predicted_state[2] = normalizeAngle(predicted_state[2] + predicted_control[1] * dt);

    geometry_msgs::msg::Point point;
    point.x = predicted_state[0];
    point.y = predicted_state[1];
    point.z = 0.06;
    result.predicted_points.push_back(point);
  }

  return result;
}

void MPCController::publishPredictedTrajectory(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  // 无订阅者时跳过发布，节省带宽
  if (!predicted_trajectory_pub_ || predicted_trajectory_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "mpc_predicted_trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = points.size() >= 2U ?
    visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0F;
  marker.color.g = 0.2F;
  marker.color.b = 1.0F;
  marker.color.a = 1.0F;
  // 0.3 秒生命周期：若控制器停止发布，轨迹线自动消失
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker.points = points;

  predicted_trajectory_pub_->publish(marker);
}

double MPCController::linearRegularization(double current, double desired) const
{
  // 对线速度增量做限幅，防止加速度过大
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_linear_velocity_increment,
    cfg_.max_linear_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_linear_velocity,
    cfg_.max_linear_velocity);
  // 低于最低线速度阈值时输出零速度，避免电机低速抖动
  if (std::fabs(cmd) < cfg_.min_linear_velocity) {
    return 0.0;
  }
  return cmd;
}

double MPCController::angularRegularization(double current, double desired) const
{
  // 与 linearRegularization 同理，对期望角速度增量做限幅并死区处理
  desired = clamp(
    desired,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  double inc = desired - current;
  inc = clamp(
    inc,
    -cfg_.max_angular_velocity_increment,
    cfg_.max_angular_velocity_increment);
  double cmd = clamp(
    current + inc,
    -cfg_.max_angular_velocity,
    cfg_.max_angular_velocity);
  if (std::fabs(cmd) < cfg_.min_angular_velocity) {
    return 0.0;
  }
  return cmd;
}

}  // namespace rmp::controller
