# 图搜索全局规划算法分析

本文档介绍项目中已迁移的三种图搜索全局路径规划算法，以统一的伪代码形式呈现核心逻辑，并分析各算法的特性。

> **符号约定**
> - `g(n)` — 从起点到节点 n 的累计路径代价
> - `h(n)` — 从节点 n 到目标的启发式估计代价
> - `f(n) = g(n) + h(n)` — 综合评估代价

---

## 共享机制

三种算法共享以下基础设施，伪代码中以函数调用形式出现：

### 8 邻域运动模型

```
MOTIONS = [
    (0,+1, 1.0),  (+1,0, 1.0),  (0,-1, 1.0),  (-1,0, 1.0),     ← 四方向，代价 1.0
    (+1,+1, √2),  (+1,-1, √2),  (-1,+1, √2),  (-1,-1, √2)      ← 四对角，代价 √2
]
```

### Sigmoid 障碍物代价模型

将 costmap 的 inflation 代价通过 Sigmoid 函数映射为平滑惩罚，避免硬阈值产生的路径抖动：

```
function SigmoidCost(cell_cost):
    normalized = cell_cost / INSCRIBED_INFLATED_OBSTACLE    // 归一化到 [0, 1]
    clamped    = clamp(normalized, 0, 1)
    sigmoid    = 1 / (1 + exp(-α × (clamped - center)))    // α 控制斜率，center 控制拐点
    return weight × sigmoid
```

参数含义：
- `α` (`obstacle_sigmoid_alpha`) — 斜率，值越大过渡越陡峭
- `center` (`obstacle_sigmoid_center`) — 拐点位置，决定惩罚开始急剧上升的阈值
- `weight` (`obstacle_cost_weight`) — 最大惩罚上限

效果：远离障碍物时惩罚 ≈ 0，接近障碍物时惩罚平滑上升至 `weight`。

### 障碍物碰撞判定

```
function IsBlocked(next, current):
    if cost[next] ≥ LETHAL_OBSTACLE × inflation_factor AND cost[next] ≥ cost[current]:
        return true     // 致命障碍物，且不是从膨胀区内部扩展
    return false
```

亮点：第二个条件允许已经处于膨胀区内的节点继续扩展，避免机器人在膨胀区边缘卡死。

---

## 1. GBFS（贪心最佳优先搜索）

### 核心思想

> 完全丢弃累计路径代价 `g`，只按启发式 `h` 排序。搜索总是朝"看起来离目标最近"的方向贪心推进，速度极快但不保证最优。

### 伪代码

```
function GBFS(start, goal):
    open_list  ← MinHeap sorted by h          // 仅按 h 排序
    closed_list ← {}

    start.h ← 0
    open_list.push(start)

    while open_list is not empty:
        current ← open_list.pop()              // 取出 h 最小的节点

        if current in closed_list:
            continue
        closed_list[current.id] ← current

        if current == goal:
            return BacktracePath(closed_list, start, goal)    ✓ 找到路径（不一定最优）

        for each motion in MOTIONS:
            next ← current + motion

            if next out of bounds:       continue
            if next in closed_list:      continue
            if IsBlocked(next, current): continue

            next.g ← 0                                       // ← GBFS 的关键：丢弃 g
            next.h ← EuclideanDistance(next, goal)
            next.parent ← current
            open_list.push(next)

    return FAILURE
```

### 算法亮点

- **速度最快**：在无障碍直线路径上，GBFS 几乎只扩展起点到终点之间的节点，扩展量接近路径长度本身
- **极低内存占用**：扩展节点少意味着 open_list 和 closed_list 都更小
- **搜索形状为"窄带"**：不像 A\* 的椭圆或 Dijkstra 的圆，GBFS 的搜索区域是一条朝目标方向的窄带

### 局限

- **不保证最优**：可能找到绕远路的路径，因为它不考虑已走过的代价
- **不利用障碍物代价**：由于 `g = 0`，Sigmoid 惩罚项无法发挥作用，路径可能紧贴障碍物边缘
- **可能在凹形障碍物中低效**：贪心地冲向目标，遇到 U 型障碍物时需要大量回退

---

## 2. Dijkstra 算法

### 核心思想

> 不使用任何启发式信息，按照累计路径代价 `g` 从小到大扩展所有可达节点，保证找到全局最优路径。

### 伪代码

```
function Dijkstra(start, goal):
    open_list  ← MinHeap sorted by g          // 按 g 排序的最小堆
    closed_list ← {}

    start.g ← 0
    open_list.push(start)

    while open_list is not empty:
        current ← open_list.pop()              // 取出 g 最小的节点

        if current in closed_list:
            continue                            // 跳过重复节点
        closed_list[current.id] ← current

        if current == goal:
            return BacktracePath(closed_list, start, goal)    ✓ 找到最优路径

        for each motion in MOTIONS:            // 8 邻域扩展
            next ← current + motion

            if next out of bounds:       continue
            if next in closed_list:      continue
            if IsBlocked(next, current): continue

            next.g ← current.g + motion.cost + SigmoidCost(cost[next])
            next.h ← 0                        // ← Dijkstra 的关键：无启发式
            next.parent ← current
            open_list.push(next)

    return FAILURE                             // 无可行路径
```

### 与 GBFS 的关键区别

```diff
  // GBFS:
  next.g ← 0
  next.h ← EuclideanDistance(next, goal)

  // Dijkstra:
+ next.g ← current.g + motion.cost + SigmoidCost(cost[next])
+ next.h ← 0
```

GBFS 只看"离目标多远"，Dijkstra 只看"走过来花了多少"。

### 算法亮点

- **完备性 + 最优性**：在非负权图上，Dijkstra 必然能找到代价最小的路径
- **Sigmoid 代价模型**：不只是避开障碍物，还让路径主动远离障碍物边缘，提高安全裕度
- **无启发式偏差**：搜索结果完全由实际代价决定，不受启发式质量影响

### 局限

- 搜索呈"圆形波前"扩散，在大地图上扩展大量无关节点，效率最低

---

## 3. A* 算法

### 核心思想

> 结合 Dijkstra 的累计代价 `g` 和 GBFS 的启发式 `h`，用 `f = g + h` 排序。既有方向引导又保证最优性——两种算法的最佳融合。

### 伪代码

```
function AStar(start, goal):
    open_list  ← MinHeap sorted by f = g + h  // 按 f 排序的最小堆
    closed_list ← {}

    start.g ← 0
    start.h ← 0
    open_list.push(start)

    while open_list is not empty:
        current ← open_list.pop()              // 取出 f 最小的节点

        if current in closed_list:
            continue
        closed_list[current.id] ← current

        if current == goal:
            return BacktracePath(closed_list, start, goal)    ✓ 找到最优路径

        for each motion in MOTIONS:
            next ← current + motion

            if next out of bounds:       continue
            if next in closed_list:      continue
            if IsBlocked(next, current): continue

            next.g ← current.g + motion.cost + SigmoidCost(cost[next])
            next.h ← EuclideanDistance(next, goal)    // ← A* = Dijkstra 的 g + GBFS 的 h
            next.parent ← current
            open_list.push(next)

    return FAILURE
```

### A* 如何统一 GBFS 和 Dijkstra

```
A* :  f = g + h        ← 同时考虑"走了多少"和"还剩多远"
         │   │
         │   └── 令 h = 0        → 退化为 Dijkstra
         └────── 令 g = 0        → 退化为 GBFS
```

### 算法亮点

- **最优性保证**：欧几里得距离是可接受启发式（admissible），永远不会高估真实代价，因此 A\* 保证找到最优路径
- **效率提升显著**：搜索扩展呈"椭圆形"而非"圆形"，在目标方向上更加集中
- **同时享受 Sigmoid 代价**：路径不仅最短，还会主动远离障碍物
- **同 f 值时优先低 h**：比较器中 `f 相等时选 h 更小的`，优先选择已经接近目标的节点，避免在等代价面上做无用探索

### 局限

- 在迷宫等高障碍密度场景，启发式误导频繁，可能退化接近 Dijkstra 的扩展量

---

## 三算法搜索行为直觉图

以下是同一地图上三种算法的典型搜索范围示意（S = 起点，G = 终点，灰色 = 扩展区域）：

```
  GBFS                    Dijkstra              A*
  ┌──────────────┐   ┌──────────────┐     ┌──────────────┐
  │         ···  │   │ ·············│     │     ········ │
  │        ···   │   │ ·············│     │    ········· │
  │      S═════G │   │ ····S════════G     │   ··S════════G
  │        ···   │   │ ·············│     │    ········· │
  │         ···  │   │ ·············│     │     ········ │
  └──────────────┘   └──────────────┘     └──────────────┘
   窄带贪心推进           圆形全向扩散           椭圆定向扩散
   扩展节点: ~80         扩展节点: ~2000        扩展节点: ~500
   路径质量: 不保证       路径质量: 最优          路径质量: 最优
```
