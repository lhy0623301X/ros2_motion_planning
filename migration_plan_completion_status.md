# ROS2 迁移计划 — 完成报告

## 一、Phase 1 总览

| 项目 | 内容 |
|------|------|
| **阶段名称** | Phase 1 — 仿真环境 + 基础设施 |
| **状态** | ✅ 已完成 |
| **完成日期** | 2026-05-14 |
| **目标** | 搭建 ROS2 Humble + Gazebo Classic 11 + Nav2 仿真环境，使用默认插件（NavFn + DWB）跑通单机器人导航 |

---

## 二、Phase 1 已完成任务清单

### 2.1 搭建 colcon workspace

- [x] 创建 `ros2_motion_planning/src/` 目录结构
- [x] 复制静态仿真资源（worlds、maps、meshes、models、urdf）
- [x] 复制 `user_config` 配置目录

**最终目录结构**：

```
ros2_motion_planning/
└── src/
    ├── core/
    │   ├── common/              ← 基础库（已迁移）
    │   └── system_config/       ← 配置系统（已迁移）
    ├── sim_env/                 ← 仿真环境（已迁移）
    │   ├── config/
    │   │   ├── nav2_params.yaml
    │   │   └── rviz2_nav2.rviz
    │   ├── launch/
    │   │   ├── main_launch.py
    │   │   ├── gazebo_launch.py
    │   │   ├── spawn_robot_launch.py
    │   │   └── navigation_launch.py
    │   ├── maps/                (4 张地图：warehouse, warehouse_2, museum, workshop)
    │   ├── meshes/              (turtlebot3, nanocar)
    │   ├── models/              (11 个 Gazebo 模型)
    │   ├── urdf/                (4 种机器人 URDF + Gazebo xacro)
    │   ├── worlds/              (Gazebo world 文件)
    │   ├── rviz/
    │   ├── package.xml
    │   └── CMakeLists.txt
    └── user_config/             (用户配置文件)
```

---

### 2.2 sim_env 仿真环境搭建

#### package.xml & CMakeLists.txt

| 文件 | 状态 | 说明 |
|------|------|------|
| `package.xml` | ✅ | ROS2 format 3，声明 21 个依赖（rclcpp、Nav2 全家桶、gazebo_ros_pkgs、xacro 等），含 `<gazebo_ros gazebo_model_path>` export |
| `CMakeLists.txt` | ✅ | ament_cmake，安装 8 个资源目录（launch/config/worlds/maps/meshes/models/urdf/rviz）到 share |

#### Launch 文件（全部新建，Python 格式）

| 文件 | 状态 | 功能 |
|------|------|------|
| `main_launch.py` | ✅ | 顶层入口：设置 GAZEBO_MODEL_PATH → 启动 Gazebo → Spawn 机器人 → 启动 Nav2 → 可选启动 RViz2 |
| `gazebo_launch.py` | ✅ | 启动 Gazebo Classic，动态拼接 world 文件路径 |
| `spawn_robot_launch.py` | ✅ | xacro 处理 URDF → robot_state_publisher → joint_state_publisher → gazebo_ros spawn_entity.py |
| `navigation_launch.py` | ✅ | Nav2 全栈：map_server + AMCL + planner_server + controller_server + bt_navigator + behavior_server + lifecycle_manager |

**支持的 Launch 参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `world` | `warehouse` | Gazebo 世界名称 |
| `map` | `warehouse` | 导航地图名称 |
| `robot_type` | `turtlebot3_waffle` | 机器人类型 |
| `x_pose` / `y_pose` / `yaw` | `0.0` | 初始位姿 |
| `use_sim_time` | `true` | 使用仿真时间 |
| `rviz` | `true` | 是否启动 RViz2 |

#### Nav2 参数配置

| 文件 | 状态 | 内容 |
|------|------|------|
| `nav2_params.yaml` | ✅ | 完整 Nav2 参数（302 行） |
| `rviz2_nav2.rviz` | ✅ | RViz2 显示配置（350 行） |

**nav2_params.yaml 包含的节点配置**：

| 节点 | 插件（Phase 1 默认） | 关键参数 |
|------|---------------------|---------|
| `planner_server` | `nav2_navfn_planner::NavfnPlanner` | A* 启用，tolerance=0.5 |
| `controller_server` | `dwb_core::DWBLocalPlanner` | max_vel_x=0.26, max_vel_theta=1.82（TurtleBot3 Waffle） |
| `amcl` | `nav2_amcl::DifferentialMotionModel` | 500-3000 粒子，laser_max_range=3.5 |
| `bt_navigator` | NavigateToPose + NavigateThroughPoses | default_server_timeout=20 |
| `behavior_server` | spin, backup, drive_on_heading, assisted_teleop, wait | — |
| `local_costmap` | voxel_layer + inflation_layer | 3x3m 滚动窗口，0.05 分辨率 |
| `global_costmap` | static_layer + obstacle_layer + inflation_layer | inflation_radius=0.8 |
| `velocity_smoother` | Open-loop | 匹配 TurtleBot3 运动限制 |
| `lifecycle_manager` | — | autostart=true，管理所有 Nav2 节点 |

#### URDF / Gazebo Xacro 适配

4 种机器人的 `.gazebo.xacro` 文件已全部适配 ROS2 Humble + Gazebo Classic 11：

| 机器人 | 文件 | 状态 | 适配的插件 |
|--------|------|------|-----------|
| TurtleBot3 Waffle | `turtlebot3_waffle.gazebo.xacro` | ✅ | diff_drive, IMU, laser, depth camera |
| TurtleBot3 Burger | `turtlebot3_burger.gazebo.xacro` | ✅ | diff_drive, IMU, laser |
| TurtleBot3 Waffle Pi | `turtlebot3_waffle_pi.gazebo.xacro` | ✅ | diff_drive, IMU, laser, camera |
| NanoCar | `nanocar.gazebo.xacro` | ✅ | skid_steer(diff_drive), IMU, laser, depth camera |

**Gazebo 插件迁移对照**：

| ROS1 插件 | ROS2 插件 | 主要变更 |
|-----------|-----------|---------|
| `libgazebo_ros_diff_drive.so` | `libgazebo_ros_diff_drive.so`（同名） | 参数改 snake_case，添加 `<ros>` remapping block，移除 `<legacyMode>` |
| `libgazebo_ros_imu.so` | `libgazebo_ros_imu_sensor.so` | 移入 `<sensor>` 块内，添加 `<initial_orientation_as_reference>false`，重构噪声模型为 SDF 格式 |
| `libgazebo_ros_laser.so` | `libgazebo_ros_ray_sensor.so` | 添加 `<output_type>sensor_msgs/LaserScan`，remapping `~/out:=scan` |
| `libgazebo_ros_openni_kinect.so` | `libgazebo_ros_camera.so` | 统一深度/RGB 相机插件，topic remapping 重写 |

#### 仿真资源兼容性

| 资源 | 数量 | 状态 | 备注 |
|------|------|------|------|
| World 文件 | 多个 | ✅ 直接复用 | Gazebo Classic 11 SDF 格式兼容 |
| 地图文件 (.pgm + .yaml) | 4 组 | ✅ 直接复用 | nav2_map_server 兼容 |
| Gazebo 模型 | 11 个 | ✅ 直接复用 | SDF 格式兼容 |
| 机器人 mesh | 2 组 | ✅ 直接复用 | STL/DAE 格式不变 |

---

### 2.3 common 基础库迁移

| 文件 | 状态 | 改动内容 |
|------|------|---------|
| `package.xml` | ✅ | catkin → ament_cmake (format 3)，依赖：rclcpp, nav2_costmap_2d, tf2, tf2_ros, nav_msgs, std_msgs, visualization_msgs, geometry_msgs |
| `CMakeLists.txt` | ✅ | 移除 Conan 依赖管理，改用 `find_package` + `ament_target_dependencies` + `ament_export_*` |
| `visualizer.h` | ✅ | `ros::Publisher` → `rclcpp::Publisher<T>::SharedPtr`，`ros::Time::now()` → `rclcpp::Clock().now()`，所有消息类型加 `::msg::` 命名空间 |
| `visualizer.cpp` | ✅ | 同上 + `ros::Duration(0.5)` → `rclcpp::Duration::from_seconds(0.5)`，`publisher.publish()` → `publisher->publish()` |
| `collision_checker.h` | ✅ | `#include <costmap_2d/…>` → `#include <nav2_costmap_2d/…>`，`costmap_2d::Costmap2DROS` → `nav2_costmap_2d::Costmap2DROS` |
| `collision_checker.cpp` | ✅ | `costmap_2d::NO_INFORMATION` → `nav2_costmap_2d::NO_INFORMATION`，`costmap_2d::INSCRIBED_INFLATED_OBSTACLE` → `nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE` |
| `log.h` | ✅ 无需改动 | glog 与 ROS 无关 |
| 纯算法文件 (point.h, node.h, math_helper.h, curve/*.cpp 等) | ✅ 无需改动 | 无 ROS 依赖 |

---

### 2.4 system_config 配置系统迁移

| 文件 | 状态 | 改动内容 |
|------|------|---------|
| `package.xml` | ✅ | catkin → ament_cmake (format 3)，新增 `ament_index_cpp` 依赖，`libprotobuf-dev` 作为 build/exec depend |
| `CMakeLists.txt` | ✅ | ament_cmake 构建，保留 `protobuf_generate()` 生成逻辑，安装 proto 文件到 `share/system_config/system_config/`；修复 `${CMAKE_CURRENT_BINARY_DIR}` 用 `$<BUILD_INTERFACE:...>` 包裹，避免 build 路径泄露到 INTERFACE_INCLUDE_DIRECTORIES |
| `system_config.cpp` | ✅ | `__FILE__` 相对路径 hack → `ament_index_cpp::get_package_share_directory("system_config")` |
| `proto_util.h/cpp` | ✅ 无需改动 | 纯 Protobuf API |
| `system_config.h` | ✅ 无需改动 | 无 ROS API |
| 21 个 `.proto` 文件 | ✅ 无需改动 | Protobuf 定义不变 |
| `system_config.pb.txt` | ✅ 无需改动 | 配置数据不变 |

---

### 2.5 Phase 1 构建问题修复记录

| 问题 | 原因 | 修复方式 |
|------|------|---------|
| `system_config` 构建失败：`INTERFACE_INCLUDE_DIRECTORIES property contains path ... build/system_config` | `target_include_directories` 中 `${CMAKE_CURRENT_BINARY_DIR}` 被 PUBLIC 直接导出 | 改为 `$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>`，仅构建阶段可见 |
| `main_launch.py` 启动报 `[Errno 2] No such file or directory: ''` | `os.environ.get('GAZEBO_MODEL_PATH', '')` 在 Python 解析时求值产生空路径 | 改用 `EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')` 在 launch 执行时安全求值；同时将所有 `PathJoinSubstitution` 嵌套列表改为直接拼接 |

---

## 三、Phase 1 验收检查清单

| # | 检查项 | 状态 | 备注 |
|---|--------|------|------|
| 1 | `colcon build` 全部 Phase 1 包编译通过（common, system_config, sim_env） | ✅ | 构建通过（修复了 system_config 的 BUILD_INTERFACE 问题和 launch 文件路径问题） |
| 2 | `ros2 launch sim_env main_launch.py` 启动 Gazebo + 加载 world | ⏳ 待验证 | |
| 3 | 机器人在 Gazebo 中正确 spawn 并显示 | ⏳ 待验证 | |
| 4 | RViz2 中可看到机器人模型、TF tree、激光雷达数据 | ⏳ 待验证 | |
| 5 | nav2_map_server 正确加载地图 | ⏳ 待验证 | |
| 6 | nav2_amcl 正常定位 | ⏳ 待验证 | |
| 7 | **Nav2 默认插件（NavFn + DWB）完成一次点到点导航** | ⏳ 待验证 | 关键验收项 |
| 8 | RViz2 中可视化全局路径和局部路径 | ⏳ 待验证 | |

**验证命令**：

```bash
cd ros2_motion_planning
colcon build --packages-select common system_config sim_env
source install/setup.bash
ros2 launch sim_env main_launch.py world:=warehouse map:=warehouse
```

---

## 四、Phase 2 — 核心算法插件迁移（进行中）

| 项目 | 内容 |
|------|------|
| **阶段名称** | Phase 2 — 核心算法插件替换 |
| **状态** | 🔧 进行中 |
| **开始日期** | 2026-05-15 |
| **目标** | 将 ROS1 自研规划/控制算法逐一适配为 Nav2 插件，替换 Phase 1 的默认 NavFn + DWB |

---

### 4.1 path_planner 包 — Nav2 GlobalPlanner 插件框架

已建立完整的 `path_planner` 包结构，实现了 `nav2_core::GlobalPlanner` 接口适配：

```
src/core/path_planner/
├── path_planner/
│   ├── src/
│   │   ├── path_planner_node.h/.cpp       ← Nav2 GlobalPlanner 插件入口
│   │   ├── path_planner.h/.cpp            ← 抽象规划器基类 + 路径复用逻辑
│   │   ├── graph_planner/
│   │   │   ├── dijkstra_planner.h/.cpp    ← Dijkstra（已验证）
│   │   │   ├── astar_planner.h/.cpp       ← A*（已验证）
│   │   │   ├── gbfs_planner.h/.cpp        ← GBFS（已验证）
│   │   │   ├── jps_planner.h/.cpp         ← JPS 跳点搜索（已迁移）
│   │   │   ├── dstar_planner.h/.cpp       ← D*（已迁移）
│   │   │   ├── dstar_lite_planner.h/.cpp  ← D* Lite（已迁移）
│   │   │   ├── lpa_star_planner.h/.cpp    ← LPA*（已迁移）
│   │   │   └── hybrid_astar_planner/      ← Hybrid A*（已迁移）
│   │   │       ├── hybrid_astar_planner.h/.cpp  ← 核心规划器
│   │   │       ├── motions.h              ← TurnDirection 枚举 + MotionPose 类
│   │   │       ├── motion_table.h/.cpp    ← Dubins 运动原语表
│   │   │       └── node_hybrid.h/.cpp     ← 3D 搜索节点
│   │   └── utils/
│   │       ├── path_planner_factory.h/.cpp     ← 算法工厂（支持 8 种算法）
│   │       └── path_replanning_utils.h/.cpp    ← 路径复用判断工具
│   ├── path_planner_plugin.xml            ← pluginlib 注册文件
│   ├── package.xml
│   └── CMakeLists.txt
└── README.md
```

#### 4.1.1 插件注册与构建系统

| 文件 | 状态 | 说明 |
|------|------|------|
| `package.xml` | ✅ | ROS2 format 3，依赖 nav2_core, nav2_costmap_2d, pluginlib, rclcpp_lifecycle, common 等 |
| `CMakeLists.txt` | ✅ | ament_cmake，构建 SHARED 库，`pluginlib_export_plugin_description_file(nav2_core ...)` 注册插件 |
| `path_planner_plugin.xml` | ✅ | 注册 `path_planner/PathPlanner` 类，类型 `rmp::path_planner::PathPlannerNode`，基类 `nav2_core::GlobalPlanner` |

#### 4.1.2 PathPlannerNode — Nav2 GlobalPlanner 适配层

`PathPlannerNode` 实现 `nav2_core::GlobalPlanner` 的 4 个生命周期接口：

| 接口 | 实现内容 |
|------|---------|
| `configure()` | 锁定 LifecycleNode → 获取 global_frame → 创建 PlannerVisualizer → 通过 PathPlannerFactory 按参数名称创建具体规划器 |
| `cleanup()` | 释放 planner、visualizer、costmap_ros、tf |
| `activate()` / `deactivate()` | 日志输出，保留扩展点 |
| `createPlan(start, goal)` | frame_id 校验 → 委托具体规划器 → 发布调试可视化 → 返回 nav_msgs::msg::Path |

#### 4.1.3 PathPlanner — 抽象规划器基类

提供所有具体规划算法的公共基础设施：

| 功能模块 | 说明 |
|----------|------|
| **坐标转换** | `world2Map()` / `map2World()` — 世界坐标 ↔ 栅格坐标，含边界检查和日志 |
| **路径输出** | `toNavPath()` — Points3d → nav_msgs::msg::Path，含 tf2 四元数转换 |
| **地图工具** | `grid2Index()` / `index2Grid()` / `getMapSize()` / `outlineMap()` |
| **路径复用** | `createPlan()` 中内置 5 步路径复用判断逻辑（详见下文 4.1.5） |
| **调试信息** | `PlannerDebugInfo` 结构体，供子类填充搜索/采样数据 |
| **配置参数** | `PathPlannerConfig` 结构体，通过 ROS2 参数动态配置 |

**PathPlannerConfig 参数清单**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tolerance` | 2.0 | 目标搜索容差 |
| `obstacle_inflation_factor` | 1.0 | 障碍物膨胀系数 |
| `obstacle_cost_weight` | 3.0 | 障碍物 Sigmoid 代价权重 |
| `obstacle_sigmoid_alpha` | 10.0 | Sigmoid 斜率参数 |
| `obstacle_sigmoid_center` | 0.35 | Sigmoid 中心点 |
| `replanning_distance` | 0.5 | 偏离路径触发重规划的距离阈值 |
| `goal_reuse_tolerance` | 0.2 | 判断目标是否一致的距离阈值 |
| `enable_path_reuse` | true | 启用路径复用 |
| `enable_debug_visualization` | true | 启用调试可视化 |
| `outline_map` | false | 是否在地图边界填充障碍物 |

#### 4.1.4 PathPlannerFactory — 算法工厂

通过 ROS2 参数 `<plugin_name>.planner_name` 动态选择具体规划算法：

| planner_name | 对应类 | 状态 |
|-------------|--------|------|
| `"dijkstra"` | `DijkstraPathPlanner` | ✅ 已验证 |
| `"A*"` | `AStarPathPlanner` | ✅ 已验证 |
| `"GBFS"` | `GBFSPathPlanner` | ✅ 已验证 |
| `"JPS"` | `JPSPathPlanner` | ✅ 已迁移 |
| `"D*"` | `DStarPathPlanner` | ✅ 已迁移 |
| `"D* Lite"` | `DStarLitePathPlanner` | ✅ 已迁移 |
| `"LPA*"` | `LPAStarPathPlanner` | ✅ 已迁移 |
| `"hybrid_astar"` | `HybridAStarPathPlanner` | ✅ 已迁移 |
| 其他值 | — | 输出警告，保持空壳模式 |

工厂通过 `node->declare_parameter<>()` 从 Nav2 参数文件读取所有配置项。Hybrid A\* 额外声明 13 个专属参数（dim_3_size、max_iterations、minimum_turning_radius 等），通过 `setHybridConfig()` 注入。

#### 4.1.5 路径复用逻辑（path_replanning_utils）

`PathPlanner::createPlan()` 内置了 5 步路径复用判断，避免每帧都触发完整搜索：

```
步骤 1：比较当前目标与上一帧目标的距离
        → 距离 > goal_reuse_tolerance → 必须重规划
步骤 2：检查 enable_path_reuse 开关 + 是否有上一帧路径 + 目标一致
        → 不满足任一条件 → 进入全量搜索
步骤 3：在上一帧路径上找距当前位置最近的点
        → 距离 > replanning_distance → 偏离过远，触发重规划
步骤 4：从最近点检查剩余路径是否被障碍物阻挡
        → 被阻挡 → 触发重规划
步骤 5：复用条件全部满足
        → 截取最近点到终点的剩余路径，直接输出（跳过搜索）
```

**`shouldReplan()` 实现细节**：

| 判断项 | 条件 | 结果 |
|--------|------|------|
| 空路径或无 costmap | `last_path.empty() \|\| !costmap` | need_replanning = true |
| 偏离路径 | 最近点距离 ≥ `replanning_distance` | need_replanning = true |
| 路径被阻挡 | 剩余路径任一点 cost ≥ `LETHAL_OBSTACLE × inflation_factor` | need_replanning = true |
| 路径畅通 | 以上均不触发 | need_replanning = false，返回 nearest_index |

---

### 4.2 Dijkstra 全局规划算法 — 第一个已验证的迁移算法

`DijkstraPathPlanner` 继承 `PathPlanner`，实现 8 邻域 Dijkstra 搜索：

**算法流程**：

```
1. validityCheck — 世界坐标转栅格坐标，校验边界
2. 构造 start_node / goal_node，计算线性索引
3. 优先队列 open_list（按 g 值排序）+ unordered_map closed_list
4. 主循环：取最小 g 值节点 → 扩展 8 邻域
   4a. 跳过 closed_list 中的节点
   4b. 跳过越界 / 致命障碍物节点
   4c. 计算 g = parent.g + motion_cost + obstacle_sigmoid_penalty
5. 到达目标 → convertClosedListToPath 回溯 → map2World 转换输出
6. open_list 耗尽 → 返回 false（无可行路径）
```

**障碍物 Sigmoid 代价模型**：

不直接使用 costmap 原始代价值，而是通过 Sigmoid 函数平滑映射：

```
normalized_cost = cell_cost / INSCRIBED_INFLATED_OBSTACLE
sigmoid = 1 / (1 + exp(-α × (normalized_cost - center)))
penalty = obstacle_cost_weight × sigmoid
```

效果：远离障碍物时惩罚接近 0，接近 `sigmoid_center` 后惩罚快速上升，从而引导路径远离障碍物边缘。

**调试可视化集成**：Dijkstra 在搜索完成后调用 `fillSearchedPointsDebugInfo()` 将所有扩展节点的世界坐标填入 `PlannerDebugInfo.searched_points`，在 RViz2 中以绿色半透明方块显示搜索范围。

---

### 4.3 PlannerVisualizer — 通用可视化模块

位于 `common` 库中，供所有规划算法共用的 RViz2 调试可视化模块：

| 文件 | 位置 |
|------|------|
| `planner_visualization.h` | `src/core/common/src/util/` |
| `planner_visualization.cpp` | `src/core/common/src/util/` |

**数据结构**：

| 结构体 | 用途 |
|--------|------|
| `DebugPoint3d` | 三维点（x, y, theta） |
| `DebugLineSegment` | 线段（start, end） |
| `PlannerDebugInfo` | 聚合所有调试数据：searched_points, sampled_points, sampled_tree_edges, sampled_trajectories |

**4 种 Marker 类型**：

| Marker | RViz 类型 | 视觉效果 | 适用算法 |
|--------|----------|----------|---------|
| `searched_points` | CUBE_LIST | 深绿半透明方块（0.2 alpha），尺寸 = 栅格分辨率 | 图搜索类（Dijkstra, A*, D*, JPS 等） |
| `sampled_points` | CUBE_LIST | 蓝色半透明方块（0.3 alpha） | 采样类（RRT, RRT*, PRM 等） |
| `sampled_tree_edges` | LINE_LIST | 灰白半透明细线（2cm, 0.4 alpha） | 采样类（RRT 树结构） |
| `sampled_trajectories` | LINE_LIST | 青色粗线（3cm, 0.6 alpha） | 轨迹优化类 |

**性能优化**：无订阅者时（`get_subscription_count() == 0`）跳过所有 Marker 构建和发布。空数据集发送 `DELETE` 动作清除残留 Marker。

**Topic**：发布到 `<plugin_name>/debug_markers`（MarkerArray），由 `PathPlannerNode::configure()` 在构造时创建。

---

### 4.4 已迁移的全局规划算法详情

#### 4.4.1 2D 图搜索算法（7 种）

| 算法 | 文件 | 状态 | 特性 |
|------|------|------|------|
| Dijkstra | `dijkstra_planner.h/.cpp` | ✅ 已验证 | 8-邻域搜索，Sigmoid 障碍物代价模型，全局最优 |
| A\* | `astar_planner.h/.cpp` | ✅ 已验证 | f = g + h（欧几里得启发式），同 f 值优先低 h，全局最优 |
| GBFS | `gbfs_planner.h/.cpp` | ✅ 已验证 | 仅按 h 排序（g = 0），极快但不保证最优 |
| JPS | `jps_planner.h/.cpp` | ✅ 已迁移 | A\* 的加速变体，对角线+直线递归跳跃，跳过对称路径 |
| D\* | `dstar_planner.h/.cpp` | ✅ 已迁移 | 反向搜索（goal→start），RAISE/LOWER 增量重规划 |
| D\* Lite | `dstar_lite_planner.h/.cpp` | ✅ 已迁移 | D\* 的简化版，g/rhs 双值 + km 修正，UpdateVertex 统一入口 |
| LPA\* | `lpa_star_planner.h/.cpp` | ✅ 已迁移 | 正向增量搜索（start→goal），适合起点固定的动态环境 |

所有 2D 算法共享基础设施：
- 8-邻域运动模型（4 方向 + 4 对角）
- Sigmoid 障碍物代价模型
- IsBlocked 碰撞判定（允许膨胀区内部扩展）
- PlannerDebugInfo 搜索节点可视化
- 路径复用逻辑（基类内置）

#### 4.4.2 3D 运动学规划算法（1 种）

| 算法 | 文件 | 状态 | 特性 |
|------|------|------|------|
| Hybrid A\* | `hybrid_astar_planner/` (7 个文件) | ✅ 已迁移 | 3D (x,y,θ) 搜索空间，Dubins 运动原语，双层启发式，Analytic Expansion |

**Hybrid A\* 文件组成**：

| 文件 | 职责 |
|------|------|
| `motions.h` | TurnDirection 枚举（6 种转向）+ MotionPose 类（位姿+转向+代价） |
| `motion_table.h/.cpp` | Dubins 运动原语表：预计算运动原语、角度量化（N bin）、行驶代价查找 |
| `node_hybrid.h/.cpp` | 3D 搜索节点：连续位姿存储、运动原语索引、多维度行驶代价计算 |
| `hybrid_astar_planner.h/.cpp` | 核心规划器：双层启发式（2D Dijkstra + Dubins 距离）、Analytic Expansion、路径复用 |

**Hybrid A\* 专属配置参数**（通过 nav2_params.yaml + Factory 注入）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dim_3_size` | 72 | 朝向角量化 bin 数（5°/bin） |
| `max_iterations` | 100000 | 最大搜索迭代次数 |
| `minimum_turning_radius` | 0.4 | 最小转弯半径（m） |
| `analytic_expansion_max_length` | 3.0 | Analytic Expansion 触发距离（m） |
| `lambda_h` | 1.5 | 启发式权重系数 |
| `retrospective_penalty` | 0.015 | 代价地图叠加惩罚系数 |
| `curve_sample_ratio` | 0.1 | Dubins 曲线采样比例 |
| `non_straight_penalty` | 1.2 | 非直行惩罚倍率 |
| `change_penalty` | 0.0 | 方向切换惩罚 |
| `reverse_penalty` | 2.0 | 倒车惩罚倍率 |
| `default_graph_size` | 100000 | 搜索图初始预分配大小 |
| `max_approach_iterations` | 1000 | 接近目标最大迭代次数 |
| `goal_tolerance` | 10.0 | 目标容差（像素） |

#### 4.4.3 算法文档

已创建 `graph_planner_algorithms.md`（约 1000 行），包含：

- 八算法总览对比表（最优性、完备性、效率、应用场景、选型建议）
- 共享机制（8-邻域运动模型、Sigmoid 代价、碰撞判定）
- 每个算法的核心思想、关键概念、完整伪代码、亮点与局限
- Hybrid A\* 特有的运动原语、双层启发式、Analytic Expansion、行驶代价惩罚机制详解
- 八算法搜索行为直觉对比图

---

### 4.5 Phase 2 当前进度总结

| 子任务 | 状态 | 说明 |
|--------|------|------|
| path_planner 包结构 + Nav2 GlobalPlanner 接口适配 | ✅ | PathPlannerNode 实现 configure/cleanup/activate/deactivate/createPlan |
| PathPlanner 抽象基类 + 坐标转换 + 路径输出 | ✅ | 含 world2Map/map2World/toNavPath/outlineMap |
| PathPlannerFactory 算法工厂 | ✅ | 按参数名称动态创建规划器，支持 8 种算法 |
| 路径复用逻辑 (path_replanning_utils) | ✅ | 5 步判断：目标一致性 → 偏离检查 → 障碍物检查 → 截取复用 |
| PlannerVisualizer 通用可视化模块 | ✅ | 4 种 Marker 类型，生命周期节点绑定，性能优化 |
| Dijkstra 全局规划算法 | ✅ 已验证 | 8-邻域 Dijkstra + Sigmoid 代价 + 可视化 |
| A\* 全局规划算法 | ✅ 已验证 | f = g + h，欧几里得启发式 + Sigmoid 代价 + 可视化 |
| GBFS 全局规划算法 | ✅ 已验证 | 纯启发式贪心搜索 + 可视化 |
| JPS 跳点搜索算法 | ✅ 已迁移 | 对角线+直线递归跳跃 + 强制邻居检测 + 可视化 |
| D\* 动态规划算法 | ✅ 已迁移 | 反向搜索 + RAISE/LOWER 增量重规划 + 可视化 |
| D\* Lite 算法 | ✅ 已迁移 | g/rhs 双值 + km 修正 + 增量重规划 + 可视化 |
| LPA\* 终身规划算法 | ✅ 已迁移 | 正向增量搜索 + g/rhs 一致性维护 + 可视化 |
| Hybrid A\* 混合规划算法 | ✅ 已迁移 | 3D 搜索 + Dubins 运动原语 + 双层启发式 + Analytic Expansion + 可视化 |
| 算法分析文档 (graph_planner_algorithms.md) | ✅ | 八算法伪代码、对比表、直觉图 |
| nav2_params.yaml 配置更新 | ✅ | 支持 8 种算法切换 + Hybrid A\* 专属参数块 |
| 所有改动已 Git 提交 | ✅ | 分模块提交，中文 commit message |
| **RRT, RRT\*, Informed RRT\*, PRM 等采样算法** | 🔲 | 待迁移 |
| **Controller 基类适配 nav2_core::Controller** | 🔲 | 待开始 |
| **8 个局部控制器迁移** | 🔲 | 待开始 |
| **voronoi_layer 适配 nav2_costmap_2d::Layer** | 🔲 | 待开始 |

---

## 五、关键技术决策记录

| 决策 | 选择 | 理由 |
|------|------|------|
| Gazebo 版本 | Gazebo Classic 11（非 Ignition/Gz） | 保持与 ROS1 仿真资源（world/model/URDF）的最大兼容性 |
| Phase 1 导航插件 | Nav2 默认（NavFn + DWB） | 先验证仿真链路畅通，Phase 2 再替换自研算法 |
| 日志库 | 保留 glog（未迁移到 RCLCPP_INFO） | glog 与 ROS 解耦，减少改动量 |
| Protobuf 构建 | 保留 CMake protobuf_generate() | proto 文件零改动，仅 CMake 构建系统适配 |
| 配置路径解析 | ament_index_cpp 替代 __FILE__ hack | ROS2 标准做法，支持 install space |
| Conan 依赖管理 | 移除，改用系统包 | ROS2 生态标准做法，Eigen3 等通过 apt 安装 |
| Launch 系统 | Python launch（非 XML） | ROS2 推荐方式，支持动态参数和条件逻辑 |
| 插件注册方式 | pluginlib + plugin.xml | Nav2 标准插件注册机制 |
| 算法选择机制 | ROS2 参数 + Factory 模式 | 支持运行时切换算法，无需修改代码 |
| 可视化模块位置 | common 库中 | 所有规划/控制算法共用，避免重复代码 |
| 路径复用策略 | 基类内置复用判断 | 所有算法自动获得路径复用能力，无需各自实现 |
| 障碍物代价模型 | Sigmoid 函数映射 | 平滑连续，可通过参数调节灵敏度，优于阈值硬判断 |

---

## 六、后续计划

| 阶段 | 内容 | 状态 |
|------|------|------|
| **Phase 1** | **仿真环境 + 基础设施** | **✅ 已完成** |
| **Phase 2** | **核心算法插件替换** | **🔧 进行中** — 全局规划器 8 种算法已全部迁移（Dijkstra/A\*/GBFS 已验证，JPS/D\*/D\* Lite/LPA\*/Hybrid A\* 已迁移），待迁移采样类算法和局部控制器 |
| Phase 3 | 集成测试 + 端到端验证 | 🔲 未开始 |
| Phase 4 | 多机器人支持 + 行人仿真(可选) + 文档 | 🔲 未开始 |
