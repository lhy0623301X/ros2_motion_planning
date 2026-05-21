# DWA Controller

本文档说明当前 ROS2 工程中 DWA 控制器的算法流程和轨迹评价体系。

DWA 的核心思想是：每个控制周期不直接跟踪某一个路径点，而是在当前速度附近采样一批候选控制量 `(v, w)`，把每个控制量前向仿真成一条短时轨迹，然后用多项代价函数评价这些轨迹，选择总代价最低的轨迹作为本周期输出。

当前实现面向差速底盘，只采样前向线速度 `v` 和角速度 `w`，不采样横向速度 `vy`。

## 每周期流程

`DWAController::computeVelocityCommands()` 每次被 Nav2 controller server 调用时，大致执行以下步骤：

1. 裁剪全局路径

   `prunePlan()` 会删除机器人已经走过的路径点，让后续评分只关注机器人前方的局部路径。

2. 构造当前状态

   根据当前机器人位姿和速度构造 `DWAState`：

   ```text
   x, y, theta, v, w
   ```

3. 计算动态窗口

   `calcDynamicWindow()` 根据当前速度和单周期速度增量限制，得到本周期可采样的速度范围：

   ```text
   v  in [current_v - max_linear_velocity_increment,
          current_v + max_linear_velocity_increment]

   w  in [current_w - max_angular_velocity_increment,
          current_w + max_angular_velocity_increment]
   ```

   这个窗口表示机器人在一个控制周期内物理上来得及达到的速度集合。

4. 采样并生成轨迹

   `generateTrajectorySamples()` 在动态窗口中离散采样 `vx_samples * vtheta_samples` 个 `(v, w)`，并对每个样本调用 `predictTrajectory()`。

   `predictTrajectory()` 假设机器人在 `sim_time` 时间内持续执行同一个 `(v, w)`，按 `sim_time_step` 积分得到预测轨迹。

5. 评价每条轨迹

   `evaluateTrajectory()` 先做硬碰撞检查。如果轨迹撞到致命障碍，直接返回 `infinity`，该轨迹不会被选中。

   如果轨迹合法，再计算各项软代价，并按权重加权求和。

6. 选择最优轨迹

   DWA 选择总代价最低的合法轨迹。当前实现中总代价越小越好。

7. 输出速度

   最优轨迹对应的 `(v, w)` 经过 `linearRegularization()` 和 `angularRegularization()` 做最终速度保护后，输出给底盘。

8. 发布可视化

   `DWAVisualizer` 会发布当前周期所有采样轨迹和最优轨迹：

   ```text
   灰色线：所有采样轨迹
   红色线：最终选中的最优轨迹
   ```

## 轨迹评价总公式

当前每条轨迹的评分逻辑是：

```text
if collision:
  score = infinity
else:
  score =
    path_distance_bias     * path_score
  + goal_distance_bias     * goal_score
  + obstacle_distance_bias * obstacle_score
  + alignment_bias         * alignment_score
  + goal_front_bias        * goal_front_score
  + velocity_bias          * velocity_score
  + twirling_bias          * twirling_score
  + oscillation_penalty
```

其中 `oscillation_penalty` 在 `DWAController::plan()` 中额外加入，用于使用控制器内部保存的历史运动方向。

所有软代价都是“越小越好”。权重越大，该项对最终选择影响越强。

## 硬碰撞检查

硬碰撞检查由 `checkCollision()` 完成。

它会检查整条预测轨迹的 footprint 是否碰到致命障碍：

```text
cost >= LETHAL_OBSTACLE
```

如果碰到致命障碍，轨迹直接非法，不再进入评分。

注意：当前实现不会把 `INSCRIBED_INFLATED_OBSTACLE` 直接作为硬碰撞。膨胀区会进入 `obstacle_score` 做软惩罚。这样 DWA 仍然有空间选择靠近障碍但不真正碰撞的绕障轨迹。

### footprint collision

当前碰撞检查不再只看机器人中心点，而是使用 Nav2 当前 footprint。

检查过程：

1. 读取 `costmap_ros->getRobotFootprint()`。
2. 对每个轨迹点，把 footprint 按轨迹点的 `x, y, theta` 变换到世界坐标。
3. 沿 footprint 多边形边界采样 costmap 代价。
4. 如果任意边界点进入致命障碍，轨迹非法。

如果没有配置 footprint，则退化为 `robot_radius` 构造的圆形近似 footprint。

### footprint scaling

速度越高，机器人需要越保守的安全余量。当前实现会根据轨迹线速度放大 footprint：

```text
if abs(v) <= footprint_scaling_speed:
  scale = 1.0
else:
  scale = 1.0 + ratio * max_footprint_scaling_factor
```

其中 `ratio` 根据当前速度在 `[footprint_scaling_speed, max_linear_velocity]` 之间的位置线性计算。

这意味着高速轨迹会用更大的 footprint 检查碰撞，低速轨迹则使用原始 footprint。

### stop_time_buffer

`stop_time_buffer` 用于检查轨迹末端是否预留了刹停空间。

当前实现会根据速度增量和控制频率估算减速度：

```text
linear_deceleration  = max_linear_velocity_increment  * control_frequency
angular_deceleration = max_angular_velocity_increment * control_frequency
```

再估算当前轨迹命令从 `(v, w)` 刹停需要的时间：

```text
stop_time = max(abs(v) / linear_deceleration,
                abs(w) / angular_deceleration)
          + stop_time_buffer
```

如果轨迹本身结束后，在这段额外安全时间内继续按当前速度推进会撞上致命障碍，该轨迹也会被判非法。

这项的作用是避免 DWA 选择“短时不撞、但已经没有刹车空间”的危险轨迹。

## 软代价项

### 1. obstacle_score

`obstacle_score` 衡量轨迹经过区域的障碍代价。

计算方式：

1. 对轨迹点和额外刹停检查点进行 footprint cost 检查。
2. 取整条轨迹上的最大 costmap 代价。
3. 用 `INSCRIBED_INFLATED_OBSTACLE` 做归一化：

   ```text
   obstacle_score = max_cost / INSCRIBED_INFLATED_OBSTACLE
   ```

含义：

- 越靠近障碍，分数越大。
- 真正致命障碍会在硬碰撞阶段被淘汰。
- 膨胀区不会直接淘汰，但会让轨迹变贵。

对应权重：

```yaml
obstacle_distance_bias
```

如果机器人绕障不够积极，可以适当增大该权重。  
如果机器人过于保守、靠近障碍就停，可以适当减小该权重，或者减小 footprint scaling。

### 2. path_score

`path_score` 衡量轨迹终点离当前全局路径有多远。

这里的“当前全局路径”指的是 Nav2 传给控制器、并经过 `prunePlan()` 裁剪后的 `global_plan_`。它来源于全局规划器，不是 DWA 自己生成的局部路径。

计算方式：

```text
path_score = min_distance(trajectory_end, path_poses)
```

含义：

- 越贴近全局路径，分数越小。
- 偏离路径绕障会让该分数变大。

对应权重：

```yaml
path_distance_bias
```

这项太大时，DWA 会过度贴全局路径，遇到路径上的障碍时不愿意绕开。  
这项太小时，机器人可能绕得太散，不容易回到全局路径。

### 3. goal_score

`goal_score` 衡量轨迹终点离当前局部目标有多远。

当前局部目标使用裁剪后的路径末端：

```text
goal_score = distance(trajectory_end, path.back())
```

含义：

- 越朝路径终点推进，分数越小。
- 这项鼓励机器人整体向目标前进，而不是只贴着路径局部摆动。

对应权重：

```yaml
goal_distance_bias
```

### 4. alignment_score

`alignment_score` 用来约束车头方向，让机器人前方的“鼻子点”贴近路径。

鼻子点定义为：

```text
nose = trajectory_end + forward_point_distance * heading_vector
```

计算方式：

```text
alignment_score =
  distance(nose, nearest_path_pose)
  + 0.25 * abs(angle_diff(trajectory_end_theta, path_yaw))
```

含义：

- 只看机器人中心贴路径，可能出现车身方向乱摆。
- 加入鼻子点后，DWA 会更倾向选择车头顺着路径趋势的轨迹。
- 接近目标时，这项会自动关闭，避免和终点姿态调整冲突。

接近目标关闭条件：

```text
distance(trajectory_end, path.back())
  <= forward_point_distance * sqrt(alignment_goal_distance_scale)
```

对应参数：

```yaml
alignment_bias
forward_point_distance
alignment_goal_distance_scale
```

如果机器人遇障后不愿意偏离全局路径，可以适当降低 `alignment_bias`。  
如果机器人车头摆动明显，可以适当提高 `alignment_bias`。

### 5. goal_front_score

`goal_front_score` 让机器人前鼻子朝局部目标推进。

它不是比较机器人中心和目标，而是比较前鼻子点和“前移后的目标点”：

```text
shifted_goal = goal + forward_point_distance * direction(robot_start -> goal)
goal_front_score = distance(nose, shifted_goal)
```

含义：

- 鼓励车头朝目标方向走。
- 减少差速车低速原地找角度和蛇形修正。
- 和 `alignment_score` 不同，它更关注“朝目标推进”，而不是“贴路径趋势”。

对应参数：

```yaml
goal_front_bias
forward_point_distance
```

如果机器人在障碍附近总是原地调角，可以适当降低该权重。  
如果机器人朝目标推进不明显，可以适当提高该权重。

### 6. velocity_score

`velocity_score` 鼓励更大的前向速度。

计算方式：

```text
velocity_score = 1.0 - clamp(v / max_linear_velocity, 0.0, 1.0)
```

含义：

- 速度越大，分数越小。
- 速度越接近 0，分数越大。
- 这项用于避免 DWA 长期选择低速轨迹或原地转。

对应参数：

```yaml
velocity_bias
```

这项太大时，机器人可能过于想前进，绕障时不够谨慎。  
这项太小时，机器人容易选择低速或停转。

### 7. twirling_score

`twirling_score` 惩罚过大的角速度。

计算方式：

```text
twirling_score = abs(w) / max_angular_velocity
```

含义：

- 角速度越大，分数越大。
- 用于抑制无必要的原地旋转和大幅摆头。

对应参数：

```yaml
twirling_bias
```

这项太大时，机器人可能不愿意转弯。  
这项太小时，机器人可能频繁原地转或走出很急的弧线。

### 8. oscillation_penalty

`oscillation_penalty` 抑制短时间内角速度方向反复切换。

这项不在 `evaluateTrajectory()` 内部，因为它需要记忆上一周期的运动方向。当前由 `DWAController` 保存状态：

```text
last_angular_sign
oscillation_reset_pose
```

逻辑：

1. 如果上一次明显向左转，本周期明显向右转，则加罚。
2. 如果上一次明显向右转，本周期明显向左转，则加罚。
3. 如果角速度绝对值小于 `oscillation_min_angular_velocity`，不记录方向。
4. 如果机器人已经移动超过 `oscillation_reset_dist`，或转过超过 `oscillation_reset_angle`，清除振荡状态。

对应参数：

```yaml
oscillation_bias
oscillation_reset_dist
oscillation_reset_angle
oscillation_min_angular_velocity
```

这项太大时，机器人可能在需要反向修正时不够灵活。  
这项太小时，机器人可能左右来回抖。

## 参数含义

### 速度边界

```yaml
max_linear_velocity
min_linear_velocity
max_linear_velocity_increment
max_angular_velocity
min_angular_velocity
max_angular_velocity_increment
```

`max_*_velocity` 是速度绝对上限。  
`max_*_velocity_increment` 是单周期速度变化上限，直接决定动态窗口大小。

如果 `max_linear_velocity_increment` 太小，DWA 起步会很慢。  
如果 `max_angular_velocity_increment` 太小，DWA 转向响应会慢。

### 轨迹预测

```yaml
sim_time
sim_time_step
vx_samples
vtheta_samples
```

`sim_time` 越长，DWA 看得越远，但计算量更大，也更容易因为远端障碍而保守。  
`sim_time_step` 越小，轨迹检查越密，但计算量更大。  
`vx_samples` 和 `vtheta_samples` 越大，候选轨迹越丰富，但计算量越大。

### 前鼻子点

```yaml
forward_point_distance
```

这个点用于 `alignment_score` 和 `goal_front_score`。

值越大，越强调车头方向。  
值太大时，可能导致机器人绕障时过于拘泥于车头对齐。

### footprint 与安全距离

```yaml
robot_radius
footprint_scaling_speed
max_footprint_scaling_factor
stop_time_buffer
```

如果 Nav2 提供了 footprint，则优先使用 Nav2 footprint。  
如果没有 footprint，则用 `robot_radius` 生成圆形近似。

`footprint_scaling_speed` 和 `max_footprint_scaling_factor` 控制高速保守程度。  
`stop_time_buffer` 控制额外刹停安全检查时间。

## 调参建议

如果机器人遇障停下、不愿意绕：

- 降低 `path_distance_bias`
- 降低 `alignment_bias`
- 降低 `goal_front_bias`
- 降低 `max_footprint_scaling_factor`
- 适当提高 `obstacle_distance_bias`，让远离障碍的轨迹更有优势

如果机器人离路径太远：

- 提高 `path_distance_bias`
- 提高 `alignment_bias`

如果机器人原地转：

- 提高 `velocity_bias`
- 提高 `twirling_bias`
- 提高 `oscillation_bias`
- 检查是否有大量前进轨迹被硬碰撞判非法

如果机器人太贴障碍：

- 提高 `obstacle_distance_bias`
- 提高 `max_footprint_scaling_factor`
- 提高 `stop_time_buffer`

## 当前实现边界

当前 DWA 已经包含基本局部绕障能力，但仍是轻量实现：

- `path_score` 仍是几何距离，不是原 ROS1 DWA 的 MapGrid 波前传播代价。
- `goal_score` 也是几何距离，不是 costmap 上的 goal wavefront。
- footprint 检查使用边界采样，不是完整填充多边形内部。
- `oscillation` 当前是加罚，不是直接判非法。
- 当前只支持差速底盘 `(v, w)`，不支持全向底盘 `vy` 采样。

这些边界会影响复杂障碍场景下的绕障质量。后续如果要进一步接近原 ROS1 DWA，优先考虑把 `path_score / goal_score` 升级为基于局部 costmap 的 MapGrid 代价传播。
