# 图搜索全局规划算法分析

本文档介绍项目中已迁移的七种图搜索全局路径规划算法，以统一的伪代码形式呈现核心逻辑，并分析各算法的特性。

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

---

## 4. JPS（跳点搜索）

### 核心思想

> A\* 在均匀网格上会产生大量对称路径（代价相同但走法不同的冗余路径），JPS 通过"跳跃"规则直接跳过这些对称区域，只在关键转折点（跳点）处生成后继节点，大幅减少 open list 规模。

### 关键概念

**强制邻居（Forced Neighbor）**：沿某方向行进时，如果侧面存在障碍物，而障碍物对角方向是自由的，则该自由格就是"强制邻居"——表示必须在此处产生跳点以覆盖被遮挡的路径。

```
沿右(→)行进时的强制邻居示例：

    ██ F          ██ = 障碍物
    →  C          C  = 当前位置
                  F  = 强制邻居（必须产生跳点）
```

**跳跃规则**：沿一个方向持续前进，直到遇到以下之一才停止：
- 遇到障碍物 → 终止该方向
- 到达目标 → 目标即跳点
- 发现强制邻居 → 当前位置即跳点

### 伪代码

```
function JPS(start, goal):
    open_list  ← MinHeap sorted by f = g + h
    closed_list ← {}

    // 初始化：从起点向四个对角方向发起探测
    for each diagonal_dir in [↗, ↙, ↘, ↖]:
        SlashLineJump(diagonal_dir, start, open_list)
    closed_list[start.id] ← start

    while open_list is not empty:
        current ← open_list.pop()

        if current in closed_list:
            continue
        closed_list[current.id] ← current

        if current == goal:
            return BacktracePath(closed_list, start, goal)    ✓ 找到最优路径

        // 从跳点继续跳跃
        Jump(current, open_list)

    return FAILURE


function Jump(node, open_list):
    dir ← 还原 node 的到达方向（从 parent 到 node 的偏移量）

    if dir 是直线方向（↑↓←→）:
        StraightLineJump(dir, node, open_list)
    else:
        SlashLineJump(dir, node, open_list)

    if node.forced_neighbor_id ≠ -1:
        fn_dir ← 从 node 到 forced_neighbor 的方向
        SlashLineJump(fn_dir, node, open_list)


function StraightLineJump(dir, node, open_list):
    pt ← node.id
    while true:
        pt ← pt + dir
        if pt 越界 OR pt 是障碍物:      return false
        if pt == goal:                    记录跳点; return true
        if HasForcedNeighbor(dir, pt):    记录跳点; return true


function SlashLineJump(dir, node, open_list):
    分解 dir 为 x_dir（水平分量）和 y_dir（垂直分量）

    // 先从当前点向水平和垂直方向做直线探测
    StraightLineJump(x_dir, node, open_list)
    StraightLineJump(y_dir, node, open_list)

    pt ← node.id
    while true:
        pt ← pt + dir
        if pt 越界 OR pt 是障碍物:      return
        if pt == goal:                    记录跳点; return
        if HasForcedNeighbor(dir, pt):    记录跳点; return

        // 在每个对角步位置，向水平和垂直方向做直线探测
        temp ← 构造临时节点(pt)
        if StraightLineJump(x_dir, temp) OR StraightLineJump(y_dir, temp):
            记录 temp 为跳点; return
```

### 算法亮点

- **A\* 的加速变体**：JPS 找到的路径与 A\* 完全等价（同为最优），但 open list 中只保留跳点，扩展量可减少一个数量级
- **零额外内存**：不需要额外数据结构，只在 A\* 基础上修改后继生成规则
- **对角线+直线递归探测**：一次对角步内嵌两次直线探测，高效覆盖整个可达区域

### 局限

- **仅适用于均匀代价网格**：JPS 假设所有非障碍格的通行代价相同，无法利用 Sigmoid 代价模型
- **在高障碍密度场景退化**：障碍物密集时跳跃距离短，优势减弱

---

## 5. D\*（Dynamic A\*）

### 核心思想

> D\* 从目标向起点反向搜索。首次搜索等效于反向 Dijkstra，但当机器人沿路径行进中发现代价地图变化时，D\* 通过 RAISE/LOWER 状态传播机制仅修复受影响的局部区域，无需重新搜索整个地图。

### 关键概念

**节点状态**：每个节点有三种标记：
- `NEW` — 未被搜索过
- `OPEN` — 在开放列表中，等待处理
- `CLOSED` — 已处理完成

**双值机制**：每个节点维护两个代价值：
- `h` — 当前已知的从目标到该节点的路径代价
- `k` — 节点在 open list 中的排序键值

**RAISE/LOWER 传播**：
- `k < h`（RAISE）：节点代价增大了（如前方出现障碍），需要从邻居中寻找替代路径
- `k = h`（LOWER）：节点代价减小或首次计算，正常向邻居传播

### 伪代码

```
function DStar_Init(goal):
    // 反向搜索：从 goal 开始
    goal.h ← 0
    Insert(goal, 0)

    while open_list is not empty:
        ProcessState()
        if start.tag == CLOSED:
            break


function Insert(node, h_new):
    // D* 的键值管理规则
    if node.tag == NEW:     node.k ← h_new
    if node.tag == OPEN:    node.k ← min(node.k, h_new)
    if node.tag == CLOSED:  node.k ← min(node.h, h_new)
    node.h ← h_new
    node.tag ← OPEN
    open_list.insert(node.k, node)


function ProcessState():
    X ← open_list.pop_min()           // 取出 k 值最小的节点
    k_old ← X.k
    X.tag ← CLOSED

    neighbors ← GetNeighbors(X)

    // RAISE 阶段：k_old < h，节点代价增大，尝试从邻居修复
    if k_old < X.h:
        for each Y in neighbors:
            if Y.tag ≠ NEW AND Y.h ≤ k_old AND X.h > Y.h + Cost(X, Y):
                X.parent ← Y
                X.h ← Y.h + Cost(X, Y)

    // LOWER 阶段：k_old == h，正常传播
    if k_old == X.h:
        for each Y in neighbors:
            if Y.tag == NEW
               OR (Y.parent == X AND Y.h ≠ X.h + Cost(X, Y))
               OR (Y.parent ≠ X AND Y.h > X.h + Cost(X, Y)):
                Y.parent ← X
                Insert(Y, X.h + Cost(X, Y))
    else:
        // RAISE 后的特殊传播
        for each Y in neighbors:
            if Y.tag == NEW OR (Y.parent == X AND Y.h ≠ X.h + Cost(X, Y)):
                Y.parent ← X
                Insert(Y, X.h + Cost(X, Y))
            else if Y.parent ≠ X AND Y.h > X.h + Cost(X, Y):
                Insert(X, X.h)                    // 重新激活 X
            else if Y.parent ≠ X AND X.h > Y.h + Cost(X, Y)
                    AND Y.tag == CLOSED AND Y.h > k_old:
                Insert(Y, Y.h)                    // 重新激活 Y


function DStar_Replan(changed_cells):
    for each cell in changed_cells:
        Modify(cell)                              // 将 CLOSED 节点重新入队
        for each neighbor of cell:
            Modify(neighbor)

    // 持续处理直到 start 再次稳定
    while start.tag ≠ CLOSED:
        ProcessState()

    ExtractPath(start → goal)
```

### 算法亮点

- **增量重规划**：地图局部变化后仅修复受影响区域，效率远高于从头搜索
- **RAISE/LOWER 机制**：精确区分代价增大和代价减小两种情况，传播行为最小化
- **反向搜索优势**：路径从 start 沿 parent 链直接到达 goal，无需反转

### 局限

- **实现复杂度高**：`ProcessState` 函数的分支逻辑较多，调试困难
- **内存开销大**：需要维护全地图的节点网格（2D 指针数组）和两份代价地图快照

---

## 6. D\* Lite

### 核心思想

> D\* Lite 是 D\* 的简化版本，基于 LPA\* 的思想重新设计。用 `g` 和 `rhs` 双值代替 D\* 的 `h/k` 和三状态标记，逻辑更清晰。同样从目标向起点反向搜索，支持增量重规划。

### 关键概念

**双值机制**：
- `g(s)` — 节点 s 的当前最优代价估计
- `rhs(s)` — 节点 s 的一步前瞻代价（从后继的 g 值推导）
- 当 `g = rhs` 时节点"局部一致"，搜索完成的标志

**局部一致性**：
- `g > rhs`（过一致，overconsistent）：找到了更好的路径，降低 g 使其一致
- `g < rhs`（欠一致，underconsistent）：原路径变差了，将 g 设为 ∞ 触发重传播

**km 修正项**：机器人移动后，所有节点到 start 的启发式距离都变了。km 累计补偿这个偏移，避免重新计算所有键值。

### 伪代码

```
function DStarLite_Init(start, goal):
    km ← 0
    // 反向搜索：goal 的 rhs = 0（搜索源）
    goal.rhs ← 0
    goal.key ← CalculateKey(goal)
    open_list.insert(goal)


function CalculateKey(s):
    return min(g(s), rhs(s)) + h(s, start) + km    // 启发式指向 start


function UpdateVertex(u):
    // 跳过搜索源（goal），其 rhs 恒为 0
    if u ≠ goal:
        u.rhs ← min over all neighbors s of { g(s) + Cost(s, u) }

    if u in open_list:
        open_list.remove(u)

    if g(u) ≠ rhs(u):                             // 局部不一致 → 需要处理
        u.key ← CalculateKey(u)
        open_list.insert(u)


function ComputeShortestPath():
    while open_list is not empty:
        u ← open_list.pop_min()

        // 终止条件：start 局部一致
        if u.key ≥ CalculateKey(start) AND g(start) == rhs(start):
            break

        if g(u) > rhs(u):                         // 过一致 → 降低 g
            g(u) ← rhs(u)
        else:                                      // 欠一致 → g 设为 ∞
            g(u) ← ∞
            UpdateVertex(u)

        for each neighbor s of u:
            UpdateVertex(s)


function DStarLite_Replan(new_start, changed_cells):
    // 步骤 1：累加 km 补偿机器人移动
    km ← km + h(last_start, new_start)
    last_start ← new_start

    // 步骤 2：增量更新受影响的节点
    for each cell in changed_cells:
        UpdateVertex(cell)
        for each neighbor of cell:
            UpdateVertex(neighbor)

    // 步骤 3：增量重新计算
    ComputeShortestPath()

    // 步骤 4：从 start 贪心追踪到 goal（选 g 最小的邻居）
    ExtractPath(start → goal)
```

### 与 D\* 的关键区别

```
D*:      三状态标记 (NEW/OPEN/CLOSED) + h/k 双值 + RAISE/LOWER 分支
D* Lite: 无状态标记 + g/rhs 双值 + 过一致/欠一致 两分支 + km 修正

D* Lite 更简洁：UpdateVertex 统一处理所有情况，不再需要复杂的 ProcessState 分支。
```

### 算法亮点

- **D\* 的简化替代**：行为等价但实现更简洁，`UpdateVertex` 是唯一的节点更新入口
- **km 修正**：避免机器人移动后重新计算所有键值，一个标量修正即可
- **贪心路径提取**：搜索完成后从 start 出发选 g 最小的邻居，路径自然朝 goal 方向

### 局限

- **内存开销同 D\***：同样需要全地图节点网格和两份代价地图快照
- **路径提取可能失败**：如果增量更新不充分，贪心追踪可能进入死循环（代码中有步数上限保护）

---

## 7. LPA\*（Lifelong Planning A\*）

### 核心思想

> LPA\* 与 D\* Lite 共享 g/rhs 双值机制和过一致/欠一致处理逻辑，但搜索方向相反：LPA\* 从起点正向搜索到目标。这意味着它能高效处理边代价变化（如障碍物出现/消失），但当起点改变（机器人移动）时必须完全重置。

### 与 D\* Lite 的对称关系

```
              搜索方向      搜索源 rhs=0    启发式方向     起点变化时
D* Lite:     goal → start    goal           h → start     增量更新 + km 修正
LPA*:        start → goal    start          h → goal      必须完全重置
```

### 伪代码

```
function LPAStar_Init(start, goal):
    // 正向搜索：start 的 rhs = 0（搜索源）
    start.rhs ← 0
    start.key ← CalculateKey(start)
    open_list.insert(start)


function CalculateKey(s):
    return min(g(s), rhs(s)) + h(s, goal)         // 启发式指向 goal（非 start）


function UpdateVertex(u):
    // 跳过搜索源（start），其 rhs 恒为 0
    if u ≠ start:
        u.rhs ← min over all neighbors s of { g(s) + Cost(s, u) }

    if u in open_list:
        open_list.remove(u)

    if g(u) ≠ rhs(u):
        u.key ← CalculateKey(u)
        open_list.insert(u)


function ComputeShortestPath():
    while open_list is not empty:
        u ← open_list.pop_min()

        // 终止条件：goal 局部一致（D* Lite 检查 start）
        if u.key ≥ CalculateKey(goal) AND g(goal) == rhs(goal):
            break

        if g(u) > rhs(u):
            g(u) ← rhs(u)
        else:
            g(u) ← ∞
            UpdateVertex(u)

        for each neighbor s of u:
            UpdateVertex(s)


function LPAStar_Replan(changed_cells):
    // 注意：起点不能变！如果起点改变，必须调用 LPAStar_Init 重置。

    for each cell in changed_cells:
        UpdateVertex(cell)
        for each neighbor of cell:
            UpdateVertex(neighbor)

    ComputeShortestPath()

    // 路径从 goal 回溯到 start（选 g 最小的邻居），然后反转
    path ← ExtractPath(goal → start)
    reverse(path)
```

### 与 D\* Lite 的代码级差异

```diff
  // D* Lite:
  goal.rhs ← 0                           // 搜索源是 goal
  key = min(g, rhs) + h(s, start) + km   // 启发式指向 start，有 km 修正
  终止条件: g(start) == rhs(start)        // 检查 start

  // LPA*:
+ start.rhs ← 0                          // 搜索源是 start
+ key = min(g, rhs) + h(s, goal)          // 启发式指向 goal，无 km（起点固定）
+ 终止条件: g(goal) == rhs(goal)          // 检查 goal
+ 路径需要反转（goal → start → reverse）
```

### 算法亮点

- **正向搜索**：概念上更直觉（从起点出发找目标），适合起点固定的场景（如固定基站的路径规划）
- **边代价增量更新**：地图局部变化后仅重新计算受影响节点，效率远高于完整 A\*

### 局限

- **起点变化 = 完全重置**：机器人移动后无法增量更新，必须从头搜索。这是 LPA\* 相比 D\* Lite 的最大劣势
- **路径需要反转**：正向搜索后路径是 goal→start 方向，需要额外反转操作

---

## 七算法搜索行为直觉图

以下是同一地图上各算法的典型搜索范围示意（S = 起点，G = 终点，灰色 = 扩展区域）：

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


  JPS                     D* / D* Lite / LPA*
  ┌──────────────┐        ┌──────────────┐
  │     · ·  · · │        │ ·············│   首次搜索类似
  │      ·       │        │ ·············│   Dijkstra 全图扩散；
  │      S═══════G        │ ····S════════G   地图变化后仅局部修复
  │        ·     │        │ ····↕↕↕······│   （↕ = 增量修复区域）
  │     · ·  ·   │        │ ·············│
  └──────────────┘        └──────────────┘
   跳点稀疏分布             首次全图 + 增量局部
   扩展节点: ~50           首次: ~2000, 增量: ~50
   路径质量: 最优           路径质量: 最优
```
