## 当前完成状态

本次实现对应的勾选状态如下；尚未运行的长时间 A/B/C 场景实验保持未勾选。

### TODO 1：原始 Baseline

* [ ] 保留并运行固定起点/终点的原始 Lazy Theta* 基线
* [ ] 生成优化前的 A/B/C 长时间实验结果

### TODO 2：统一地图可通行判断

* [x] 找到并保留 `0.50 m clearance` 硬阈值
* [x] 使用预计算的 clearance 和 `traversable_mask`
* [x] Neighbor、LOS、起点和终点统一使用 `is_traversable(x, y)`

### TODO 3：统一 LOS

* [x] 统一 corner cutting 禁止规则
* [x] 遇到不可通行 cell 立即结束检查
* [x] 增加 `los_checks` 和 `los_cells_checked` 统计
* [x] 统一使用 `line_of_sight(a, b)`

### TODO 4：统一 `edge_cost()`

* [x] Neighbor relaxation
* [x] Parent shortcut
* [x] SetVertex/父节点更新路径
* [x] LOS repair

### TODO 5：第一次回归测试

* [ ] 与原始 Baseline 完全相同的固定场景对比
* [x] corner cutting 和 `0.50 m clearance` 针对性回归测试

### TODO 6：搜索数据结构优化

* [ ] 尚未进行独立的性能剖析和大规模数据结构重构

### TODO 7：Soft Clearance Cost

* [x] `clearance < 0.50 m` 仍然不可通行
* [x] `0.50 m ~ 0.85 m` 提供 `clearance_cost(x, y)` 软代价

### TODO 8：可配置 `edge_cost()`

* [x] 实现 `L * (1 + lambda_geo * C_segment)`
* [x] `lambda_geo=0.0` 保持距离型 Lazy Theta*
* [x] `lambda_geo>0` 切换到安全感知模式
* [x] 支持 `--lambda-geo` 命令行参数

### TODO 9：Heuristic

* [x] 保持 `h(n)=EuclideanDistance(n, goal)`，未加入 clearance cost

### TODO 10：前后对比实验

* [ ] 尚未运行长时间 A/B/C 场景实验，也未生成结果表
* [x] 已加入规划统计字段，运行时会记录耗时、路径长度、展开节点、LOS、净距、路点数和成功状态

### 实验执行记录

* [x] 已在固定 warehouse 地图、固定起点和终点执行 A/B/C 短时对比
* [x] 已生成 `test/result/warehouse/analysis/edge_cost/metrics.csv`
* [x] 已生成 `test/result/warehouse/analysis/edge_cost/metrics.json`
* [x] 已生成 `test/result/warehouse/analysis/edge_cost/paths_comparison.png`
* [ ] 真实 RViz 窗口截图：当前环境无 `DISPLAY`，使用等价路径可视化图代替

## 最终目标

第一阶段最终形成：

```text
现有地图
   ↓
统一 traversable 判断
   ↓
统一 LOS
   ↓
统一 edge_cost()
   ↓
Lazy Theta*
   │
   ├── λ = 0 → 原始距离规划
   │
   └── λ > 0 → 安全代价规划
   ↓
统一 Benchmark
   ↓
优化前 / 优化后量化对比
```

### 实施原则

> **先统一接口，不改变算法行为；先完成回归测试，再增加 clearance cost；每次只改一个模块，并与 Baseline 对比。**

在现有 Lazy Theta* 工程中，只新增“高度代价 Height Cost”功能。

现有 `edge_cost()` 已经完成，不要重写 Lazy Theta*，不要修改现有 clearance cost 逻辑。

## 目标

利用现有 3D / 仿真环境信息，为每个二维栅格计算垂直净空：

```python
height_clearance[y, x]
```

然后生成：

```python
height_cost[y, x]
```

范围为 `[0, 1]`。

## 高度代价定义

配置参数：

```yaml
robot_height: 0.60
height_safety_margin: 0.10
preferred_height: 1.00
lambda_height: 1.0
```

其中：

```text
hard_height = robot_height + height_safety_margin
```

对于每个 `(x, y)` 栅格，计算机器人地面上方最近悬空障碍物的高度：

```python
height_clearance[y, x]
```

规则：

```text
height_clearance <= hard_height
→ 不可通行

height_clearance >= preferred_height
→ height_cost = 0

hard_height < height_clearance < preferred_height
→ 产生 0~1 的软代价
```

代价公式：

```python
def compute_height_cost(clearance_h, hard_h, preferred_h):
    if clearance_h <= hard_h:
        return float("inf")

    if clearance_h >= preferred_h:
        return 0.0

    r = (preferred_h - clearance_h) / (preferred_h - hard_h)
    return r * r
```

## 与现有 edge_cost 集成

不要重新设计 `edge_cost()`。

只在现有 edge cost 中增加高度项，例如：

```python
edge_cost = existing_edge_cost + lambda_height * segment_height_cost
```

`segment_height_cost` 应根据该 edge / LOS 经过的栅格计算，例如使用平均值：

```python
segment_height_cost = mean(
    height_cost[y, x]
    for x, y in line_cells
)
```

如果路径段经过任意：

```text
height_clearance <= hard_height
```

的栅格，则该 edge 直接不可通行。

## 桌子场景预期

例如：

```text
桌面高度 = 0.90 m
机器人高度 = 0.60 m
安全余量 = 0.10 m
preferred_height = 1.00 m
```

则：

```text
hard_height = 0.70 m

0.90 m > 0.70 m
→ 桌下允许通过

但 0.90 m < 1.00 m
→ 产生一定 height cost
```

如果桌面净空只有：

```text
0.65 m
```

则：

```text
0.65 < 0.70
→ blocked
```

## 实现要求

* [x] 检查现有工程中可获得的 3D / MuJoCo 高度信息
* [x] 不重复创建新的地图坐标系，沿用现有 2D grid
* [x] 生成 `height_clearance[y, x]`
* [x] 生成 `height_cost[y, x]`
* [x] 高度不足区域作为 hard obstacle
* [x] 把 `height_cost` 接入现有 `edge_cost()`
* [x] `lambda_height = 0` 时必须完全关闭高度软代价
* [x] 参数全部配置化，不硬编码机器人高度
* [x] 不修改无关模块

## 最低测试

至少验证三种情况：

```text
1. 净空 < hard_height
   → edge 不可通行

2. hard_height < 净空 < preferred_height
   → edge 可通行，但有 height cost

3. 净空 >= preferred_height
   → height cost = 0
```

另外建立一个桌子测试场景，确认：

```text
桌面足够高 → 桌下允许规划
桌面过低 → 桌下禁止规划
```

实现验证：

* [x] 净空 `< hard_height` 的 edge 被拒绝
* [x] `hard_height < 净空 < preferred_height` 可通行且产生高度代价
* [x] 净空 `>= preferred_height` 的高度代价为 0
* [x] 使用现有 warehouse `work_table` 高度语义，并用合成桌面点云覆盖测试高/低桌面

先检查现有代码中高度信息可以从哪里取得，然后以最小改动实现以上功能。
