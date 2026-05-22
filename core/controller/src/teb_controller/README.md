# TEB Controller

这个目录提供 TEB 控制器的最小骨架，目标是先把 ROS2 `controller` 包里的职责边界立清楚，再逐步回填真正的优化算法。

## 当前文件职责

- `teb_controller.h/.cpp`
  - TEB 对外入口
  - 负责参数读取、路径裁剪、调用优化器、输出速度命令

- `teb_types.h`
  - TEB 共用数据结构
  - 统一定义配置、状态、轨迹、命令和优化摘要

- `teb_optimizer.h/.cpp`
  - 轻量优化内核
  - 当前先提供轨迹初始化、可行性检查、命令提取和优化流程占位

- `teb_visualizer.h/.cpp`
  - RViz 调试可视化
  - 当前发布优化后的离散轨迹折线

## 当前状态

第一版是“可编译骨架”，不是完整 TEB。

已经具备：

- 接入统一 `ControllerAlgorithm` 框架
- 在 `controller_factory` 中注册 `TEB`
- 提供独立参数命名空间
- 具备最小轨迹初始化和 feasibility 检查

后续建议按这个顺序逐步补全：

1. 用局部路径片段初始化 band，并按 `dt_ref` 自适应重采样
2. 增加障碍物距离、速度、加速度、时间最优等真正代价项
3. 加入迭代更新策略，而不是当前占位式 optimize
4. 增强可视化，区分初始轨迹与优化后轨迹
