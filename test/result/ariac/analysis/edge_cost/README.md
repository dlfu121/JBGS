# ARIAC 代价地图对比测试

本目录记录固定长距离路线 `(-4.0, -4.0) -> (7.0, 7.0)` 的 Lazy Theta*
对比实验。端点直线距离约 `15.56 m`，路线穿过 ARIAC 地图中央的多个设备区域，
用于比较距离最短策略和水平净空安全策略。

## 自动复现

在仓库根目录执行：

```bash
./test/sh/analysis/edge_cost/run_edge_cost_experiment.sh --scene ariac
```

脚本直接读取 `maps/ariac/ariac_map_3d.pgm`、对应 YAML 和点云，不需要启动
ROS 导航循环或图形界面。运行结束后检查：

1. `metrics.csv` 和 `metrics.json` 中 3 组水平净空实验均规划成功。
2. `A_baseline_reference` 与 `B_unified_distance` 的路径长度、最小净空、平均净空
   和采样栅格数完全一致。
3. `C_safety_aware` 的最小净空明显高于基线，同时路径长度只小幅增加。
4. `paths_comparison.png` 中红色安全路径相对蓝色/绿色距离路径有清晰可见的绕行。
5. `summary.md` 中路线仍为 `(-4.0, -4.0) -> (7.0, 7.0)`，防止误用旧的短路线结果。

`height_*` 图和 `path_height_profile.png` 用于检查高度证据。本地图若显示
`height_observed_traversable_cells = 0`，表示高度观测已经全部被二维占据或膨胀层
阻挡；此时高度权重路径重合是地图证据限制，不应解读为高度代价算法优劣。

多路径分析使用 `./test/sh/analysis/edge_cost/run_multi_path_analysis.sh`，结果写入
本目录下的 `multi_path/`。如果 `height_profiles_grid.png` 的某个面板标记
`No finite height observations`，表示该条路径穿过的所有栅格都没有有限的垂直
高度观测；这不是绘图失败，而是点云在该路径上的高度证据为空。

## RViz 手动测试方案

`slam/run_nav_saved.sh` 当前不会把 `--lambda-geo` 转发给 `nav_p2p.py`，因此手动
对比时分终端启动各组件。每个终端先进入仓库根目录并执行：

```bash
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
source /opt/ros/foxy/setup.bash
```

距离模式按以下顺序启动：

```bash
# 终端 1：生成固定起点模型并启动仿真。
python3.8 model/robot/gen_ariac_robot.py --start-x -4.0 --start-y -4.0
python3.8 slam/bridge/bridge_ariac.py --view

# 终端 2：发布保存地图导航所需的 map -> odom 变换。
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  --ros-args -p use_sim_time:=true

# 终端 3：启动距离模式导航。
python3.8 nav_p2p.py --use-saved --scene ariac --lambda-geo 0.0

# 终端 4：启动 RViz，随后发送固定目标。
rviz2 -d slam/view_map.rviz --ros-args -p use_sim_time:=true &
python3.8 send_goal.py --scene ariac 7.0 7.0 --wait
```

完成距离模式后停止以上进程，再从终端 1 的模型生成步骤重新启动仿真，以恢复完全
相同的机器人初始状态。安全模式仅把终端 3 的命令改为：

```bash
python3.8 nav_p2p.py --use-saved --scene ariac --lambda-geo 2.0
```

等待 RViz 中地图、机器人位姿和 `/planning_map` 稳定后，按以下步骤比较。每次只
改变 `lambda_geo`，起终点、地图和动态障碍条件必须保持一致。

1. 确认机器人初始位姿在地图坐标 `(-4.0, -4.0)` 附近且落在自由栅格内。
2. 距离模式设置 `lambda_geo = 0.0`，发送目标 `(7.0, 7.0)`，记录 `/nav_path`、
   规划耗时、路径长度和最小净空，并保存 RViz 截图。
3. 重新把机器人置于同一初始位姿，安全模式设置 `lambda_geo = 2.0`，发送相同目标，
   记录相同指标并保存截图。
4. 在 RViz 中同时显示 `/planning_map`、`/nav_path` 和机器人 footprint，重点观察地图
   中央设备附近：距离路径应贴近低净空带，安全路径应主动留出更大侧向间距。
5. 实车或仿真跟随路径时保持动态障碍条件一致，确认两次均无障碍接触、无不可达状态，
   且安全模式没有因额外绕行导致局部规划持续振荡。

手动测试通过标准：两种模式均成功到达；`lambda_geo = 2.0` 的最小水平净空高于
`lambda_geo = 0.0`；安全路径允许略长，但应与本目录 `metrics.csv` 的量级一致；
RViz 路径差异应与 `paths_comparison.png` 的绕行方向一致。

## 结果文件

- `summary.md`：核心量化结论。
- `metrics.csv`：便于表格对比的全部指标。
- `metrics.json`：地图、端点、参数、时间戳和规划器完整统计。
- `paths_comparison.png`：水平净空 A/B/C 路径叠图。
- `height_paths_comparison.png`：高度权重路径叠图。
- `height_clearance.png`、`height_cost.png`、`height_blocked.png`：高度层诊断。
- `path_height_profile.png`：各路径沿程高度净空。
