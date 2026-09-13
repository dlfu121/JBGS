# worker_scene：MuJoCo 四足机械狗 SLAM 与 VLA / 巡检最终任务

本仓库在 MuJoCo 中搭建 ARIAC 2025 实验室场景，配四足机械狗（双臂双手）进行
3D 激光 SLAM 建图与导航；最终任务（`_ultimate_task`）为 VLA 螺丝刀抓取与
定点仪表巡检，其 VLA 控制与数据采集来自 `datecollect_vla/`。

## 项目文件配置

```text
worker_scene/
├── README.md                    # 本文件，项目总览
├── request.list                 # 项目需求清单（运行环境与关键文件）
│
├── _ultimate_task/              # ★ 最终任务（VLA / inspect，详见其 README）
│   ├── vla/                     # VLA 螺丝刀任务：导航、ACT 客户端/服务、任务定义
│   └── inspect/                 # 定点巡检任务：路线 JSON、巡检执行、任务定义
│
├── model/                       # MuJoCo 模型文件
│   ├── assets/                  # 3D 网格和纹理资源
│   │   ├── ariac/               # ARIAC 网格、材质、预览图和转换报告
│   │   ├── arm/                 # RM65 机械臂 STL（base_link ~ link_6）
│   │   ├── dog/                 # 四足机械狗 STL（dog_visual_0~5）
│   │   ├── hand/                # 灵巧手 STL（左手/右手各指段）
│   │   ├── urdf/                # 原始 URDF 文件（RM65、dexhand 左右手）
│   │   └── calib_board.png      # 标定板纹理
│   ├── robot/                   # 机器人组装与场景集成
│   │   ├── build_robot.py       # 机器人装配脚本（需 mujoco ≥3.6）
│   │   ├── gen_ariac_robot.py   # 将机器人和 3D 雷达注入 ARIAC
│   │   ├── ariac_lab_with_robot_3d.xml # SLAM 使用的 ARIAC 组合模型
│   │   ├── robot.xml            # 纯机器人模型
│   │   ├── robot_py38.xml       # Python 3.8 兼容机器人模型
│   │   └── robot_template_3d_py38.xml   # 通用 3D 机器人模板（Python 3.8 兼容）
│   └── scenes/                  # 纯场景定义
│       ├── ariac_lab.xml        # 主 ARIAC 纯场景
│       └── build_ariac_compat_meshes.py # 生成 3.2.3 兼容平面网格
│
├── slam/                        # SLAM 建图与导航（详见 slam/README.md）
│   ├── mapping/                 # 建图：3D SLAM 启动/保存、点云处理、Frontier 探索
│   │   ├── run_slam_3d.sh       # 3D SLAM 一键启动
│   │   ├── save_map_3d.sh       # 保存 3D 地图到 maps/
│   │   ├── frontier_explorer.py # Frontier 自主探索 + 避障节点
│   │   ├── rtabmap_3d.launch.py # rtabmap ROS2 launch
│   │   └── rtabmap_params.yaml  # rtabmap ICP-SLAM 参数
│   ├── navigation/              # 导航：点到点规划与目标发送
│   │   ├── nav_p2p.py           # 点对点导航：3D地图 + Lazy Theta* 规划 + 纯追踪跟随
│   │   ├── send_goal.py         # 发送导航目标点（支持 map/world 坐标系）
│   │   ├── run_nav.sh           # 导航启动（在线建图模式）
│   │   ├── run_nav_saved.sh     # 导航启动（加载已保存地图）
│   │   ├── restart_nav.sh       # 重启导航栈
│   │   └── view_map.sh          # 离线查看已保存地图
│   └── bridge/                  # MuJoCo → ROS 桥接节点
│       ├── bridge_ariac.py      # 默认 ARIAC 场景 3D 桥接
│       └── local_avoidance.py   # 局部避障状态机
│
├── maps/                        # 建图输出（SLAM 生成的地图文件）
│   ├── ariac/                   # ARIAC 场景地图
│   └── lab_map.yaml             # 实验室参考地图
│
├── datecollect_vla/             # VLA 与数据采集入口（_ultimate_task/vla 的 VLA 控制来源）
    ├── assets/                  # RM65-6F 机械臂、DexHand 灵巧手与 MuJoCo 场景
    ├── config/                  # 机器人/仿真/动捕/录制配置（robot.yaml 等）
    ├── scripts/                 # 遥操作、数据采集、ACT 训练与推理入口
    ├── rm65/                    # RM65 解析 FK/IK 与 MATLAB 验证
    ├── data/                    # 数据集（rm65_dexhand_merged 等）
    └── outputs/                 # ACT 训练 checkpoint
```

各目录职责：

- `_ultimate_task/`：**最终任务**，本仓库的顶层验收目标；
- `model/`：MuJoCo 模型与场景生成脚本；
- `slam/`：3D SLAM 建图、导航与 MuJoCo→ROS 桥接；
- `maps/`：SLAM 地图产物；
- `datecollect_vla/`：**VLA 与数据采集入口**，提供 LeRobot ACT 训练、数据集与
  推理脚本，是 `_ultimate_task/vla` 的 VLA 控制来源，详见其 README；
- `tools/`：巡检报表生成工具。

## 最终任务：_ultimate_task

`_ultimate_task` 是本仓库的最终任务，包含两个相互隔离的上层任务，复用同一套
MuJoCo、点云、里程计与导航栈，完整说明见
[`_ultimate_task/README.md`](_ultimate_task/README.md)：

| 配置 | VLA | inspect |
| --- | --- | --- |
| 任务 | 导航到装配桌前，右臂 + 灵巧手完成螺丝刀任务 | 依次前往 cabinet、tank、hydrant，左臂拍摄仪表 |
| 启动 | `--task vla` | `--task inspect` |
| 场景 | 随机螺丝刀 VLA 场景 | 普通 ARIAC 巡检场景 |
| 相机 | `table_camera`、`wrist_overhead_camera` | `lefthand_camera` |
| 机械臂 | 右臂微分 IK | 左臂关节空间平滑插值 |
| 控制接口 | `/act/*` | `/inspection/*` |

VLA 闭环流程（VLA 控制来源为 `datecollect_vla/` 的 LeRobot ACT checkpoint、
数据集与推理脚本）：

```text
导航到桌前 -> 启动 ACT 推理 -> 螺丝刀脱离桌面 -> 结束 ACT 推理
```

inspect 流程：

```text
依次导航到 cabinet / tank / hydrant -> 左臂抬起到拍摄姿态 -> 保存照片与 JSON -> 恢复行走姿态
```

最小运行入口：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash

# VLA 任务：仿真 + 导航栈
./slam/navigation/run_nav_saved.sh --scene ariac --task vla --view
python3.8 _ultimate_task/vla/run_vla.py

# inspect 任务：仿真 + 导航栈
./slam/navigation/run_nav_saved.sh --scene ariac --task inspect --view
python3.8 _ultimate_task/inspect/run_inspection.py --navigation-only   # 只验证导航、停车点和朝向
python3.8 _ultimate_task/inspect/run_inspection.py                     # 执行完整巡检
```

同一套仿真一次只能选择一个任务配置。

## 运行环境

完整需求清单见 [`request.list`](request.list)。

| 组件 | 版本 |
| --- | --- |
| OS | Ubuntu 20.04 |
| Python | 3.8 |
| MuJoCo | 3.2.3 |
| ROS | ROS 2 Foxy |
| SLAM | `ros-foxy-rtabmap-ros`（若 apt 无此包需源码编译） |

> 路径统一使用真实目录 `model.test`（点），不是 `model_test`（下划线）。

## 运行方式

### 生成 ARIAC 组合场景

更新 ARIAC 资源后重新生成 3.2.3 兼容网格和机器人组合场景：

```bash
python3.8 model/scenes/build_ariac_compat_meshes.py
python3.8 model/robot/gen_ariac_robot.py
```

### 3D 点云 SLAM 建图（rtabmap）

```bash
source /opt/ros/foxy/setup.bash

# 固定航点巡视建图
./slam/mapping/run_slam_3d.sh --view --fresh

# Frontier 自主探索建图（无需航点，适用于陌生环境）
./slam/mapping/run_slam_3d.sh --view --fresh --explore

# 完成后保存地图
./slam/mapping/save_map_3d.sh           # 默认保存到 ./maps/ariac/
```

输出：`maps/ariac/ariac_map_3d.yaml` + `.pgm`（2D 投影栅格）、
`maps/ariac/rtabmap.db`（完整 3D 地图数据库）。

### 使用保存地图导航

```bash
./slam/navigation/run_nav_saved.sh --scene ariac --view
python3.8 slam/navigation/send_goal.py --scene ariac 2.0 1.0 --wait
```

### 只查看纯 ARIAC 场景

```bash
../../bin/simulate model/scenes/ariac_lab.xml
```

### VLA 训练与数据采集（datecollect_vla 入口）

`datecollect_vla/` 是 VLA 与数据采集入口：动捕遥操作、数据采集、ACT 训练与
推理都在该目录内完成，详细安装与使用见
[`datecollect_vla/README.md`](datecollect_vla/README.md)。其训练出的
checkpoint（`outputs/train/`）与数据集（`data/`）即
`_ultimate_task/vla` 的 VLA 控制来源。

### 巡检能力报表

使用 `_ultimate_task/inspect/record` 中的拍摄 JSON/PNG 生成可插入技术文档的
图表和表格：

```bash
python3 tools/inspection_report.py --out-dir inspection_report
```

输出 `inspection_dashboard.png`、`inspection_metrics.csv`、
`inspection_summary.md` 和 `inspection_summary.json`。如果同时提供包含
`time_s,x_m,y_m,collision,dynamic_avoid` 列的导航遥测 CSV，报表会追加路径长度、
运行时长、碰撞和动态避障统计。

批量展示静态地图上的巡检规划路径：

```bash
python3 tools/inspection_path_sim.py --runs 6
```

脚本会在 `inspection_report/path_simulation/` 下生成各次截图和
`inspection_paths_montage.png` 汇总大图。

## 可调节参数

### 建图启动参数（`slam/mapping/run_slam_3d.sh`）

| 参数 | 说明 |
| --- | --- |
| `--view` | 同时打开 MuJoCo 查看器 |
| `--fresh` | 清理遗留进程后启动（推荐首次运行时使用） |
| `--explore` | 使用 Frontier Exploration 自主探索（不走固定航点） |
| `--fast` | 以 2 倍仿真速度运行（等价 `--sim-speed 2.0`） |
| `--sim-speed N` | 自定义仿真速度倍率（0.1~4.0，建议不超过 2.0） |
| `--teleop` | 手动遥控模式，不自动巡视 |
| `--no-rviz` | 不启动 RViz |
| `--scene ariac` | 使用 ARIAC 2025 实验室（默认） |
| `--laps N` | 固定航点模式下巡视圈数（默认 1） |
| `--odom-noise` | 显式注入里程计随机游走，仅用于回环抗漂移压力测试；默认关闭 |

### 模型生成参数（`model/robot/gen_ariac_robot.py`）

| 参数 | 说明 |
| --- | --- |
| `--start-x` | 机器人起点 X（默认 `4`） |
| `--start-y` | 机器人起点 Y（默认 `4.6`） |

### 任务启动参数（`_ultimate_task`）

| 参数 | 说明 |
| --- | --- |
| `--task vla` | 选择 VLA 螺丝刀任务 |
| `--task inspect` | 选择定点巡检任务 |
| `VLA_SEED=<整数>` | 固定螺丝刀随机种子，用于复现实验；只能与 `--task vla` 同用 |

更多导航、Frontier 探索、巡视和 rtabmap 参数见
[`slam/README.md`](slam/README.md)；datecollect_vla 侧的遥操作、采集与训练
参数见 [`datecollect_vla/README.md`](datecollect_vla/README.md)。

## 备注

### datecollect_vla 与 VLA 控制链

`datecollect_vla/` 是本仓库的 VLA 与数据采集入口：它维护 RM65-6F + DexHand
的 MuJoCo 仿真、动捕遥操作、LeRobot 数据集与 ACT 训练/推理。`_ultimate_task/vla`
的 ACT 推理服务、checkpoint（`datecollect_vla/outputs/train/`）与数据集
（`datecollect_vla/data/`）均来自该目录，二者构成完整的 VLA 控制链路。

### 模型生成（mujoco 3.2.3 环境）

基准模型由各场景的生成脚本预先生成，ROS 侧使用对应的 Python 3.8 兼容 XML。
ARIAC 组合 3D 模型由 `gen_ariac_robot.py` 生成，运行时禁用 XML 中逐束传感器，
改用 `mj_multiRay` 批量投射 64 层 × 360 束（23040 束）。`build_robot.py` 是为
mujoco ≥3.6 编写的装配脚本，当前 3.2.3 环境无法运行，保留作为未来升级参考。

### ROS 话题（3D 模式）

- `/pointcloud` (PointCloud2) — 最多 23040 个有效命中点，10 Hz
- `/odom` (Odometry) — 50 Hz
- `/cloud_map` (PointCloud2) — 保留水平表面的 3D 点云地图
- `/rtabmap/map` (OccupancyGrid) — 2D 投影栅格
- TF: `map → odom → base_footprint → base_link → lidar3d`

3D 桥接默认发布无噪声仿真里程计，以避免长路径随机游走在回环时造成点云整体
跳变。

### 路径规划

建图完成后的地图可直接用于 nav2：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=maps/ariac/ariac_map_3d.yaml \
  -p use_sim_time:=true
```

rtabmap 也支持定位模式（不再建图，只做位姿估计）：在
`slam/mapping/rtabmap_params.yaml` 中设置 `Mem/IncrementalMemory: "false"`，
并移除 launch 文件中的 `--delete_db_on_start` 参数。
