# 避障与建图测试

本目录集中存放避障、规划与建图相关的自动化测试。所有命令在
`worker_scene/` 根目录下执行。

## 目录约定

测试代码按“场景 / 测试内容”归类，Python 与 Shell 入口保持分离：

```text
test/
├── py/
│   ├── ariac/                 # ARIAC 模型、巡检路线、shelves 窄道
│   ├── warehouse/             # warehouse MuJoCo 物理回归
│   ├── common/                # 与具体场景无关的导航/避障逻辑
│   └── analysis/edge_cost/    # 代价地图、多路径和高度证据分析
├── sh/
│   ├── ariac/                 # ARIAC 模型与 shelves 入口
│   ├── common/                # 通用避障入口
│   └── analysis/edge_cost/    # 代价地图分析入口
└── result/
    ├── ariac/<content>/       # ARIAC 结果
    └── warehouse/<content>/   # warehouse 结果
```

## 一键入口

```bash
./test/sh/common/avoidance/run_avoidance_test.sh [quick|regression|nav|ros]
```

| 模式 | 作用 | 耗时 |
| --- | --- | --- |
| `quick` | 无 ROS 逻辑检查 + 规划器单测（编译、水平净距、恢复状态机、全局规划、点云→净距→急停） | 秒级 |
| `regression` | 完整无界面 MuJoCo 巡逻/接触/桌面回归（默认模式） | 约两分钟 |
| `ros` | 启动 ROS 2 + RTAB-Map + MuJoCo 查看器（`--view --fresh` 巡逻） | 一直运行 |

## 测试脚本

### ARIAC 模型与默认场景回归

```bash
./test/sh/ariac/model/run_ariac_scene_test.sh
```

该测试无需 ROS 或图形界面，使用项目要求的 Python 3.8 + MuJoCo 3.2.3：

- 检查 ARIAC 文件已按 `model/scenes`、`model/assets`、`model/robot` 分类。
- 检查 XML 引用的全部网格存在。
- 实际加载纯 ARIAC 和 ARIAC + 机器人组合模型。
- 检查 `dog_base`、`lidar3d_frame`、执行器和雷达传感器。
- 将机器人放到 ARIAC 巡视通道，实际投射 `64 x 360` 束激光并检查至少 10 束
  直接命中 `1.90 m` 高的柜顶上表面。
- 检查 `run_slam_3d.sh` 默认选择 `ariac`，且旧场景仍可显式选择。

### 1. 快速逻辑检查 — `test/py/common/avoidance/verify_avoidance_code.py`

无需 ROS 2 或图形界面，验证：

```bash
python3.8 test/py/common/avoidance/verify_avoidance_code.py
```

- 共享模块、3D bridge、`explore_planner.py`、`frontier_explorer.py` 均可编译。
- `0.90 m` 向下斜距正确换算为约 `0.45 m` 水平净距。
- 恢复过程按 `BACKUP -> TURN -> CLEAR -> IDLE` 退出。

### 2. 全局规划器单测 — `test/py/common/navigation/test_explore_planner.py`

无需 ROS，验证探索导航的核心逻辑：

```bash
python3.8 test/py/common/navigation/test_explore_planner.py
```

- EDT 障碍膨胀（`0.45 m`）与地图边界强制不可走。
- Dijkstra 可达性：墙另一侧目标不可达并被过滤。
- 绕墙路径：全程落在可走格、穿过缺口到达另一侧。
- 目标拉黑：失败目标不会被再次选中。
- 点云 → 每方位角净距 → `LocalAvoidance` 前方净距与急停（`BACKUP`）。

### 3. Warehouse 完整物理回归 — `test/py/warehouse/avoidance/test_avoidance_simple.py`

直接运行仓库 MuJoCo 模型，无图形界面：

```bash
python3.8 test/py/warehouse/avoidance/test_avoidance_simple.py
```

通过条件：

1. 17 个仓库航点全部完成，不能通过跳过航点伪装成功。
2. 每个物理步都不能发生机器狗与非地面障碍的接触。
3. 单次恢复状态持续时间必须小于 8 秒。
4. 矮障碍必须触发水平净距紧急判断。
5. 仓库桌面在固定位置必须至少有 250 个直接激光命中点。

当前基线：

```text
PASS sim_time=204.66s waypoints=17 recoveries=1 max_recovery=4.20s obstacle_contacts=0 tabletop_hits=343 scans=2048
```

### 4. ARIAC edge-cost 算法对比

无需启动 ROS 导航循环或图形界面，直接在已保存的 ARIAC 3D 地图上运行与
`nav_p2p.py` 相同的 Lazy Theta*、障碍膨胀、水平净空和高度净空计算：

```bash
./test/sh/analysis/edge_cost/run_edge_cost_experiment.sh
```

水平净空实验包含以下三组：

| 算法 | `lambda_geo` | 目的 |
| --- | ---: | --- |
| `A_baseline_reference` | 0 | 原始最短距离基线 |
| `B_unified_distance` | 0 | 验证统一 edge-cost 接口与基线等价 |
| `C_safety_aware` | 2 | 以少量路程换取更大的障碍净空 |

同一次运行还会比较 `lambda_height = 0, 1, 2`。若点云中的高度观测栅格已经
全部被二维占据/膨胀层阻挡，结果摘要会明确标记高度证据覆盖率为零，此时三组路径
一致只说明当前地图没有独立的可通行高度证据。默认固定路线为 ARIAC
地图坐标 `(-4.0, -4.0) -> (7.0, 7.0)`，端点直线距离约 `15.56 m`；端点和地图输入均写入
结果元数据，便于复现。结果保存在 `test/result/ariac/analysis/edge_cost/`：

- `metrics.json`：完整参数、规划器统计量和两组实验指标。
- `metrics.csv`：便于表格软件或后续统计程序读取的合并指标。
- `summary.md`：水平净空对比摘要。
- `paths_comparison.png`、`height_paths_comparison.png`：路径叠图。
- `height_clearance.png`、`height_cost.png`、`height_blocked.png`、
  `path_height_profile.png`：高度净空诊断图。
- `README.md`：自动复跑命令和 RViz 手动测试方案。

多路径批量分析使用：

```bash
./test/sh/analysis/edge_cost/run_multi_path_analysis.sh
```

其结果在同一目录的 `multi_path/` 子目录下。

指标会采样任意角路径每条边穿过的全部栅格，而不只统计路径折点。需要复跑旧仓库
基线时使用：

```bash
./test/sh/analysis/edge_cost/run_edge_cost_experiment.sh --scene warehouse
```

### 5. Shelves 窄道四向通行回归

```bash
./test/sh/ariac/navigation/run_shelves_corridor_test.sh
```

该测试直接加载 `model/robot/ariac_lab_with_robot_3d.xml` 中的 ARIAC `shelf_1`～
`shelf_6`，从南→北、北→南、西→东、东→西四个方向沿货架中心窄道采样。它同时
检查机器人外接半径与货架网格包围盒的实际净距，并用 MuJoCo 水平激光回波验证局部避障触发条件：平行于路径且
在 `0.43 m` 扫掠宽度之外的左右货架不会触发 DWA；落在前方扫掠带内的新障碍仍
会触发 DWA，并禁止继续高速直行。

结果写入 `test/result/ariac/navigation/shelves/`：

- `shelves_all_directions.png`：四个方向总路径图；
- `shelves_south_to_north.png`、`shelves_north_to_south.png`、
  `shelves_west_to_east.png`、`shelves_east_to_west.png`：单方向路径图；
- `shelves_metrics.json`、`shelves_metrics.csv`：端点、净距和触发门控的机器可读结果；
- `summary.md`：本次运行摘要。

脚本默认设置 `MUJOCO_GL=osmesa` 以适配无显示环境；有真实图形上下文时可通过
`SHELVES_MUJOCO_GL=egl` 覆盖。

## 实测运行（联调）

### 巡逻建图

```bash
./slam/run_slam_3d.sh --view --fresh
```

重点观察：

- 终端 `[避障状态]` 应按 后退 → 转向 → 越障 → 恢复导航 推进，不持续停在同
  一状态。
- 点云发布规格应为 `64 layers x 360 rays = 23040 total`。
- RViz 累积三维点云话题为 `/cloud_map`；桌面应为连续水平点云而非只有桌腿。

无界面冒烟检查：

```bash
timeout 35s ./slam/run_slam_3d.sh --fresh --no-rviz --teleop
```

启动初期短暂出现 `base_footprint frame does not exist` 属正常，TF 建立后应
恢复；持续出现才表示链路有问题。

### Frontier 自主探索建图

```bash
./slam/run_slam_3d.sh --view --fresh --explore
```

探索模式使用 `slam/frontier_explorer.py`（全局可达路径规划 + 复用
`local_avoidance.py` 局部避障）。重点观察：

- RViz 中 `/frontier_markers`（绿色 frontier、红色当前目标）与 `/explore_path`
  （规划路径）是否正确。
- 目标应沿规划路径绕开障碍，不直接撞墙、不沿整面墙滑行。
- 行驶中已被地图覆盖的旧目标应被取消，近期访问区域不会立即被重新选择。
- 到达 frontier 后先停车等新地图；目标消失则跳过补扫，否则只补扫 90°。
- 遇到角落/新障碍时能自动脱困，多次失败后拉黑该目标改选其他 frontier。

### 建图进度检查

```bash
```

未知比例 < 30% 时地图基本建完，可保存。
