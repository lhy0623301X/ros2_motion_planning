## DWA Controller Design

这份文档定义当前 ROS2 工程中自研 DWA 控制器的**合理文件划分**。

目标不是把所有逻辑塞进一个文件，也不是为了“优雅”拆出很多碎模块，而是保持：

- 算法主线清晰
- 文件数量适中
- 与当前 `ControllerAlgorithm` 架构兼容
- 后续实现和调试成本可控

## 设计原则

- DWA 的主流程要围绕算法本身组织，而不是围绕过度抽象拆文件
- “采样速度”和“前向生成轨迹”属于一个连续逻辑单元，应放在同一个模块
- 参数直接通过 `DWAControllerConfig` 从 YAML 读取，不单独拆 `dwa_config`
- 不单独做 `debug_publisher`
- 不引入复杂 critic 类继承体系，优先使用简单直接的函数式结构

## 参考算法主线

当前建议围绕下面这条 DWA 主线实现：

- `prune_plan()`
- `motion_model()`
- `calc_dynamic_window()`
- `predict_trajectory()`
- `check_collision()`
- `evaluate_trajectory()`
- `plan()`

这条主线已经足够描述一个完整的 DWA 控制器。

## 推荐文件划分

当前最合理的是 **4 个文件**，必要时可以扩展到 **5 个文件**，但不建议更多。

### 1. `dwa_controller.h`

职责：

- 声明 `DWAController` 类
- 声明 `DWAControllerConfig`
- 声明 DWA 共享轻量数据结构

建议内容：

- `struct DWAControllerConfig`
- `struct DWAState`
- `struct DynamicWindow`
- `struct DWATrajectoryPoint`
- `struct DWATrajectory`
- `class DWAController : public ControllerAlgorithm`

说明：

- `DWAControllerConfig` 直接放这里最合理
- `DWAState / DynamicWindow / DWATrajectory` 这些也是 DWA 本体紧耦合结构，没必要单拆 `types.h`

### 2. `dwa_controller.cpp`

职责：

- DWA 顶层控制流程
- 参数读取
- `setPlan()`
- `computeVelocityCommands()`
- `plan()`
- 最终速度正则化

建议保留这些主函数：

- `readParameters()`
- `prunePlan(...)`
- `computeVelocityCommands(...)`
- `plan(...)`
- `linearRegularization(...)`
- `angularRegularization(...)`

说明：

- 这个文件负责“总控”
- 它应该让人一眼看到 DWA 每个周期是怎么跑的
- 但不应堆太多细节计算

### 3. `dwa_motion.h`

职责：

- 与“轨迹生成”直接相关的声明
- 包括动态窗口计算、运动模型、前向轨迹展开

建议内容：

- `DWAState motionModel(...)`
- `DynamicWindow calcDynamicWindow(...)`
- `DWATrajectory predictTrajectory(...)`
- `std::vector<DWATrajectory> generateTrajectorySamples(...)`

说明：

- **采样与轨迹生成放在一起**
- 这是你刚刚指出的关键点，也是这份设计里最重要的收口
- DWA 中“采一个 `(v, w)`”和“立刻把它滚成一条轨迹”本来就是一个模块

### 4. `dwa_motion.cpp`

职责：

- 实现 `motion_model`
- 实现 `calc_dynamic_window`
- 实现 `predict_trajectory`
- 实现候选命令采样与轨迹生成

建议内部逻辑：

1. 根据当前状态和速度限制计算 dynamic window
2. 在 dynamic window 中离散采样 `v / w`
3. 对每个样本调用 `motionModel()` 反复积分
4. 生成一条短时轨迹
5. 返回全部候选轨迹

### 5. `dwa_critic.h`

职责：

- 声明轨迹评价相关函数

建议内容：

- `bool checkCollision(...)`
- `double scorePath(...)`
- `double scoreGoal(...)`
- `double evaluateTrajectory(...)`

说明：

- 这里不建议拆成很多 critic 类
- 第一版直接保留函数式接口最合适

### 6. `dwa_critic.cpp`

职责：

- 实现碰撞检测
- 实现路径代价
- 实现目标推进代价
- 实现总代价组合

建议第一版只保留三项：

- `collision`
- `path`
- `goal`

以后如果确实需要，再加：

- `alignment`
- `oscillation`
- `twirling`

## 文件之间的关系

建议关系如下：

- `dwa_controller.cpp`
  - 调 `prunePlan(...)`
  - 调 `plan(...)`

- `plan(...)`
  - 调 `calcDynamicWindow(...)`
  - 调 `generateTrajectorySamples(...)`
  - 遍历每条轨迹，调 `evaluateTrajectory(...)`
  - 选出最佳轨迹

- `evaluateTrajectory(...)`
  - 先调 `checkCollision(...)`
  - 再调 `scorePath(...)`
  - 再调 `scoreGoal(...)`
  - 汇总最终分数

这样职责边界会很清楚：

- `dwa_controller.*`：控制主线
- `dwa_motion.*`：采样 + 轨迹生成
- `dwa_critic.*`：轨迹评价

## 为什么这样比“全放一个文件”更合理

因为 DWA 至少天然有两大块明显不同的逻辑：

1. **生成候选轨迹**
2. **评价候选轨迹**

如果把这两部分全堆在一个 cpp 里，后面代码会很快失控：

- 速度窗口逻辑
- 仿真积分逻辑
- costmap 碰撞逻辑
- 路径距离代价
- 目标推进代价

都会混在一起，不利于调试。

但如果拆得太碎，又会把一个很直白的算法拆成很多跳转层。

所以当前这个 4~6 文件结构是比较平衡的。

## 当前不建议拆出的文件

以下内容当前都不建议单独拆：

- `dwa_config.h`
- `dwa_types.h`
- `dynamic_window.h`
- `trajectory_generator.h`
- `trajectory_scorer.h`
- `debug_publisher.h`
- `base_critic.h`
- `obstacle_critic.h`
- `goal_critic.h`
- `path_critic.h`

原因：

- 这些命名虽然“看起来专业”，但会把当前实现拆得过碎
- 对你这个项目现阶段来说，阅读和调试成本会明显高于收益

## 第一版最小实现建议

第一版建议先支持：

- 差速底盘
- `(v, w)` 二维采样
- 固定时域前向仿真
- 基础 costmap 碰撞检测
- 路径贴合代价
- 目标推进代价

不建议第一版就做：

- `vy` 采样
- footprint 动态缩放
- 点云调试可视化
- 复杂 oscillation 状态机
- stop-rotate 内嵌到 DWA 内部

这些都可以在第二阶段再加。

## 与当前项目架构的关系

外层仍然保持不变：

- `ControllerNode`
  - start alignment
  - goal alignment
  - outer speed limit chain

`DWAController` 只负责：

- 基于当前局部状态，从候选轨迹中选最优控制命令

这样不会和你现有的：

- `heading_aligner`
- `goal_speed_limiter`
- `curvature_speed_limiter`

发生职责冲突。

## 推荐实现顺序

### Phase 1

- `dwa_controller.h`
- `dwa_controller.cpp`
- `dwa_motion.h`
- `dwa_motion.cpp`

先把：

- 参数读取
- dynamic window
- 采样
- 前向仿真

跑通

### Phase 2

- `dwa_critic.h`
- `dwa_critic.cpp`

补：

- 碰撞检测
- 路径评分
- 目标评分

### Phase 3

增强项按需增加：

- alignment cost
- oscillation control
- footprint scaling
- 轨迹可视化

## 最终结论

当前针对你这个项目，最合理的 DWA 文件结构不是：

- 全塞一个 cpp

也不是：

- 十几个抽象小文件

而是控制在：

- `dwa_controller.h`
- `dwa_controller.cpp`
- `dwa_motion.h`
- `dwa_motion.cpp`
- `dwa_critic.h`
- `dwa_critic.cpp`

其中最关键的约束是：

- **采样与轨迹生成必须放在同一个模块**

这最符合 DWA 算法本身的逻辑。*** End Patch
