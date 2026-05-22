# ROS2 Motion Planning

一个基于 ROS2 Humble、Nav2 和 Gazebo 的移动机器人运动规划仿真工程。项目包含自定义全局规划器、统一局部控制器插件、仿真环境、地图、机器人模型和 RViz 配置，适合验证路径规划、轨迹跟踪和局部避障算法。

## 功能概览

- Nav2 导航链路：map server、AMCL、planner server、controller server、BT navigator。
- 自定义全局规划器插件：`path_planner/PathPlanner`。
- 自定义局部控制器插件：`controller/Controller`。
- 全局规划算法可选：`GBFS`、`dijkstra`、`A*`、`JPS`、`D*`、`D* Lite`、`LPA*`、`hybrid A*`、`RRT`、`RRT*`、`Informed RRT*`、`RRT-Connect`。
- 局部控制算法可选：`PID`、`LQR`、`MPC`、`RPP`、`DWA`、`TEB`。
- 全局路径曲线平滑：`BezierCurve`、`BSplineCurve`、`CubicSplineCurve`。
- Gazebo 仿真环境：机器人模型、地图、world、RViz 配置。

## 目录结构

```text
src/
  core/
    common/          # 公共数学、几何、日志等工具
    path_planner/    # Nav2 全局规划器插件
    controller/      # Nav2 局部控制器插件和算法实现
  sim_env/           # Gazebo、Nav2、地图、模型、RViz 和 launch 文件
  plugins/           # Gazebo / RViz / costmap 扩展插件
```

## 依赖

建议环境：

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- Gazebo Classic
- Eigen3
- g2o
- OSQP 0.6.3
- OsqpEigen 0.8.1

如果 OSQP / OsqpEigen 安装在 `/usr/local/lib`，当前 controller 包已设置运行时搜索路径。

## 构建

```bash
cd /home/lhy/projects_files/ros2_motion_planning
colcon build
source install/setup.bash
```

只编译控制器：

```bash
colcon build --packages-select controller
source install/setup.bash
```

## 启动仿真导航

```bash
ros2 launch sim_env main_launch.py
```

常用参数示例：

```bash
ros2 launch sim_env main_launch.py \
  world:=/home/lhy/projects_files/ros2_motion_planning/src/sim_env/worlds/workshop.world \
  map:=/home/lhy/projects_files/ros2_motion_planning/src/sim_env/maps/workshop/workshop.yaml \
  x_pose:=0.0 y_pose:=0.0 yaw:=0.0
```

## 主要配置

- Nav2 参数：`src/sim_env/config/nav2_params.yaml`
- 控制器算法参数：`src/core/controller/config/controller_params.yaml`
- RViz 配置：`src/sim_env/config/rviz2_nav2.rviz`

切换全局规划算法：

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "path_planner/PathPlanner"
      planner_name: "A*"
```

全局规划算法可选值：

```text
GBFS, dijkstra, A*, JPS, D*, D* Lite, LPA*, hybrid A*,
RRT, RRT*, Informed RRT*, RRT-Connect
```

开启全局路径曲线平滑：

```yaml
planner_server:
  ros__parameters:
    GridBased:
      enable_path_smoother: true
      path_smoother_type: "CubicSplineCurve"
      path_smoother_downsample_factor: 10.0
```

曲线类型可选：

```text
BezierCurve, BSplineCurve, CubicSplineCurve
```

其中 `path_smoother_downsample_factor` 用于控制平滑前关键点降采样间距，数值越大，参与拟合的关键点越稀疏，曲线通常更顺滑，但也可能更偏离原始路径。

切换局部控制算法：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      controller_name: "MPC"
```

可选值：

```text
PID, LQR, MPC, RPP, DWA, TEB
```

## 调试可视化

RViz 中可按需添加以下 Marker / MarkerArray 话题：

- `lookahead_point`：控制器前视目标点
- `mpc_predicted_trajectory`：MPC 蓝色预测轨迹
- `dwa_trajectories`：DWA 采样轨迹
- `teb_trajectory`：TEB 优化轨迹和参考轨迹

全局调试可视化开关在：

```yaml
/**:
  ros__parameters:
    enable_debug_visualization: true
```
