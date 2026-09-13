# SLAM 建图与导航模块

本目录包含 MuJoCo 仿真环境中的 3D SLAM 建图、自主探索和点到点导航相关代码，
按用途分为三类：

```
slam/
├── README.md          # 本文件（快速启动、参数、故障排查都在这里）
├── mapping/           # 建图：3D SLAM 启动、地图保存、点云处理、Frontier 探索
├── navigation/        # 导航：点到点规划、目标发送、已保存地图导航启动
└── bridge/            # MuJoCo → ROS 桥接（场景入口 + 规划/避障/行人等公共库）
```

依赖：Python 3.8 + mujoco 3.2.3、ROS 2 Foxy、`ros-foxy-rtabmap-ros`、`numpy`、`scipy`。

```bash
pip3.8 install scipy numpy
```

## 运行方式

### 方案 A：使用已保存地图导航（推荐新手测试）

**特点**：不需要重新建图，狗站在原地等你发目标点，立即导航。

**1. 启动仿真 + 导航系统**（打开 MuJoCo 仿真、RViz 可视化两个窗口）：

```bash
cd /path/to/worker_scene
source /opt/ros/foxy/setup.bash
./slam/navigation/run_nav_saved.sh --view
```

**2. 发送导航目标点**（另开终端）：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

也可以用 `slam/navigation/send_goal.py` 辅助脚本（支持 map/world 坐标换算和到达等待）：

```bash
python3.8 slam/navigation/send_goal.py --scene ariac 4.5 1.0 --wait
```

**3. 观察导航过程**：

- **RViz**：绿色路径线显示规划结果，机器人沿路径移动
- **MuJoCo**：3D 仿真窗口实时显示狗的运动
- **终端输出**：显示导航状态（PLANNING/FOLLOWING/ARRIVED/STUCK）

**4. RViz 图形化发送目标点**：工具栏点 **"2D Goal Pose"**（快捷键 `G`）→ 在地图上
点击目标位置 → 拖动鼠标设置目标朝向，松开即发送。

**5. 停止系统**：启动终端按 `Ctrl+C`，自动清理所有进程。

### 方案 B：实时建图 + 导航

**特点**：狗先自动巡视场景建图，可以在建图过程中或完成后发送目标点。适合首次建图
或地图需要更新的情况。

**终端 1**：启动 3D SLAM 建图（狗沿预设航点巡视，实时构建 3D 点云并投影为 2D 栅格）

```bash
source /opt/ros/foxy/setup.bash
./slam/mapping/run_slam_3d.sh --view
```

**终端 2**（等狗开始巡视几秒后）：启动导航节点

```bash
source /opt/ros/foxy/setup.bash
python3.8 slam/navigation/nav_p2p.py
```

**终端 3**：发送目标点（同方案 A 第 2 步）。导航节点会用当前已建立的地图规划路径并导航。

**终端 4（可选）**：保存新地图供后续使用

```bash
./slam/mapping/save_map_3d.sh      # 保存到 maps/ariac/
```

### 其他建图模式

```bash
# Frontier 自主探索建图（推荐，无需航点）
./slam/mapping/run_slam_3d.sh --view --fresh --explore

# 手动遥控建图
./slam/mapping/run_slam_3d.sh --view --fresh --teleop
```

### 建图输出

建图完成后保存：

```bash
./slam/mapping/save_map_3d.sh                    # 默认保存 ARIAC
./slam/mapping/save_map_3d.sh --scene ariac
```

保存 ARIAC 地图后可直接启动定位与点到点导航：

```bash
./slam/navigation/run_nav_saved.sh --scene ariac --view
python3.8 slam/navigation/send_goal.py --scene ariac 2.0 1.0 --wait
```

输出文件（保存到 `maps/` 目录）：

| 文件 | 用途 |
|------|------|
| `maps/*_3d.yaml` + `.pgm` | 2D 投影栅格地图（nav2 路径规划用） |
| `maps/rtabmap.db` | 完整 3D 地图数据库（含位姿图，可离线导出 .pcd） |

### 离线决策回放（replay_decisions.py）

用已保存的成品地图当 ground truth，离线模拟"逐步建图 + 探索决策"，几分钟内
复现线上 `frontier_explorer.py` 的决策流（frontier 检测/可达性/选目标/规划），
不启动 MuJoCo / rtabmap / ROS，专门用来快速抓决策逻辑 bug、改参数后先验证再上真机。

**跑之前**：真机探索会话要先停（本脚本直接复用 CPU，且不依赖 ROS 话题）。

```bash
cd /path/to/worker_scene
source /opt/ros/foxy/setup.bash        # 需要（frontier_explorer 依赖 rclpy，cpython-38）
python3.8 slam/mapping/replay_decisions.py \
    --map maps/ariac/ariac_map_3d.yaml \
    --start 4.0,4.6 \
    --max-steps 300
```

输出：
- stdout 每 3s 打印一次决策：`pose=... frontiers=N cov=xx.x% 候选: (x,y) s=..✓/✗ ←选中`，
  `✓/✗` = Dijkstra 可达/不可达，`←选中` = 线上同款打分选出的目标。
- 结束时打印最终覆盖 %、剩余 unknown，并存覆盖图。看图：

```bash
eog /tmp/opencode/replay_coverage.png   # 或 xdg-open
```

原理与已知约束（详见 `todolist/replay_decisions.md` 恢复笔记）：
- 每 0.5s 从当前位置向 360° 发 `FAN_RAYS=1200` 条射线"揭示"已见图，遇障碍即停
  （其后保持 unknown，模拟真实雷达遮挡），射线太稀会留针孔把已知区打碎成假 frontier。
- EDT 重建 + frontier 重检只在必要时刻做（3s 一次/无目标立即），与线上节流一致。
- 假设理想 SLAM 无漂移（当前 `ODOM_NOISE_DEFAULT=False`，459 节点 0 处 map→odom
  跳变），只验证决策逻辑，不验证 ICP/回环。

### 进阶使用

**查看规划路径**

```bash
ros2 topic echo /nav_path
```

**查看实时点云（方案 B）**

在 RViz 中添加 PointCloud2 显示插件：
1. 点击左下角 "Add" 按钮
2. 选择 "PointCloud2"
3. Topic 选择 `/pointcloud`

**手动控制机器人（测试用）**

发送速度指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}"
```

**注意**：手动控制会自动停止导航的巡视模式。

**查看 TF 树**

```bash
ros2 run tf2_tools view_frames
```

生成的 `frames.pdf` 显示完整的坐标系变换关系。

## 文件夹中文件介绍

```
slam/
├── README.md                        # 本文件
├── mapping/                         # 建图与地图处理
│   ├── run_slam_3d.sh               # 3D SLAM 启动脚本（主入口）
│   ├── save_map_3d.sh               # 保存地图脚本
│   ├── rtabmap_3d.launch.py         # rtabmap ROS2 launch 文件
│   ├── rtabmap_params.yaml          # rtabmap ICP-SLAM 参数
│   ├── slam_3d.rviz                 # 3D SLAM RViz 配置
│   ├── frontier_explorer.py         # Frontier 自主探索 + 全局规划 + 局部避障节点
│   ├── replay_decisions.py          # 离线决策回放（成品地图上验证探索决策，无需 ROS）
│   ├── cloud_map_height_filter.py   # 地面点过滤（保留水平表面）
│   ├── project_cloud_to_pgm.py      # 3D 点云补全 2D 栅格障碍轮廓
│   └── save_occupancy_grid.py       # 订阅 /map 保存 2D 栅格（yaml+pgm）
├── navigation/                      # 点到点导航
│   ├── nav_p2p.py                   # 点对点导航：3D地图 + Lazy Theta* 规划 + 纯追踪跟随
│   ├── send_goal.py                 # 发送导航目标点（支持 map/world 坐标系）
│   ├── run_nav.sh                   # 导航启动（在线建图）
│   ├── run_nav_saved.sh             # 导航启动（加载已保存地图）
│   ├── restart_nav.sh               # 重启导航栈
│   ├── view_map.sh                  # 离线查看地图（nav2 map_server + RViz）
│   └── view_map.rviz                # 地图查看 RViz 配置
└── bridge/                          # MuJoCo → ROS 桥接
    ├── bridge_core.py               # 公共 3D 桥接内核
    ├── bridge_ariac.py              # ARIAC 场景独立入口
    ├── local_avoidance.py           # 局部避障 + 脱困状态机（巡逻/探索共用）
    ├── explore_planner.py           # 全局路径规划（无 ROS 依赖，可单测）
    ├── frontier_policy.py           # Frontier 检测/冷却/拉黑策略
    ├── dynamic_dwa.py               # 条件式全向 DWA 局部规划
    ├── pedestrian_motion.py         # 动态行人轨迹
    ├── rm65_ik.py                   # RM65 机械臂 IK
    └── rm65_kinematics.py           # RM65 运动学
```

## 可调节参数

### run_slam_3d.sh 参数说明

| 参数 | 说明 |
|------|------|
| `--view` | 同时打开 MuJoCo 查看器 |
| `--fresh` | 清理遗留进程后启动（推荐首次运行时使用） |
| `--explore` | 使用 Frontier Exploration 自主探索（不走固定航点） |
| `--fast` | 以 2 倍仿真速度运行，缩短实验的墙钟时间 |
| `--sim-speed N` | 自定义仿真速度倍率（0.1~4.0，建议不超过 2.0） |
| `--teleop` | 手动遥控模式，不自动巡视 |
| `--no-rviz` | 不启动 RViz |
| `--scene ariac` | 使用 ARIAC 2025 实验室（默认） |
| `--laps N` | 固定航点模式下巡视圈数（默认 1） |
| `--odom-noise` | 显式注入里程计随机游走，仅用于回环抗漂移压力测试；默认关闭 |

加速探索实验时使用：

```bash
./slam/mapping/run_slam_3d.sh --view --fresh --explore --fast
```

`--fast` 等价于 `--sim-speed 2.0`。它加速 MuJoCo 仿真时间、机器人运动和
所有基于 `/clock` 的探索阶段，同时保持点云的墙钟发布频率约为 10 Hz，避免
RTAB-Map 的每秒计算负载随倍率一起增长。倍率过高会增大相邻点云的位姿间隔，
可能降低 ICP 配准和避障效果，因此常规实验建议使用 2 倍或更低。

### 场景切换

默认场景为 `ariac`（省略 `--scene` 时也是此场景）：

```bash
./slam/mapping/run_slam_3d.sh --view --fresh --explore --scene ariac
```

场景对应的桥接脚本：`ariac` → `slam/bridge/bridge_ariac.py`。

底层共用代码放在 `bridge_core.py`，场景入口只负责绑定自己的场景名与 XML。
动态行人的场景轨迹配置也在内核中。

如需新增场景：
1. 在 `model/scenes/` 创建场景 XML
2. 参考 `model/robot/gen_ariac_robot.py` 注入机器人和 3D lidar
3. 在 `slam/bridge/` 创建对应的场景入口；底层公共实现放在 `bridge_core.py`
4. 在 `run_slam_3d.sh` 的 `SCENE` 分支中添加新场景名

### 建图模式

#### 模式 1：固定航点巡视（默认）

机器人按预设航点路径巡视，使用反应式避障（前方 ±30° 扇区检测）。

航点定义在 `bridge_core.py` 的 `PATROL_WAYPOINTS` 列表中。修改航点：

```python
PATROL_WAYPOINTS = [
    (4.0, 4.6),     # (x, y) 世界坐标
    (0.0, 4.6),
    ...
]
```

适合：已知环境布局，想快速覆盖指定区域。

#### 模式 2：Frontier 自主探索（--explore）

使用 `slam/mapping/frontier_explorer.py` 自主探索建图：

1. 订阅 rtabmap 输出的 2D 栅格地图（`/rtabmap/grid_map` 或 `/rtabmap/map`）
2. 检测已知区域与未知区域的边界（frontier）
3. 对地图障碍做 EDT 膨胀，从机器人位置跑 Dijkstra，只把【可达】的
   frontier 作为候选（墙另一侧不可达的目标自动过滤，避免撞墙/沿墙滑行）
4. 生成可达路径，沿 waypoints 纯追踪跟随
5. 复用 `slam/bridge/local_avoidance.py` 的 BACKUP->TURN->CLEAR->IDLE
   状态机做局部安全层（急停/减速/脱困，避免撞障碍与角落卡死）
6. 进度式卡死检测：目标距离不再下降才判卡住，多次脱困无效则拉黑目标重选
7. 导航途中用新地图取消已经被雷达覆盖的目标，并对访问区域设置短时冷却
8. 到达后停车等待地图更新；仅在 frontier 仍存在时补扫 90°，然后继续探索

适合：陌生环境，不知道布局，需要完整覆盖。

#### 模式 3：手动遥控（--teleop）

不自动移动，通过 `/cmd_vel` 话题手动控制：

```bash
# 另开终端
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Frontier Explorer 参数（`slam/mapping/frontier_explorer.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FRONTIER_MIN_SIZE` | 8 | 最小 frontier 聚类（栅格数），小于此的忽略 |
| `FRONTIER_GOAL_BIAS` | 0.25 | 目标选择偏好：0=纯距离优先，1=纯大小优先 |
| `FRONTIER_REACHED_DIST` | 0.8m | 认为到达 frontier 的距离阈值 |
| `GOAL_REVALIDATE_SEC` | 1.0s | 导航途中检查旧目标是否仍为 frontier 的周期 |
| `VISITED_COOLDOWN_SEC` | 30s | 已扫描目标附近区域不再被选择的时间 |
| `VISITED_COOLDOWN_RADIUS` | 2.0m | 已扫描目标冷却区域的半径 |
| `SCAN_MAP_WAIT_MAX_SEC` | 2.5s | 到达后等待新地图的最长时间 |
| `PLAN_INFLATE` | 0.45m | 路径规划障碍膨胀半径（大于狗碰撞盒半对角线 0.55m 的常用值） |
| `PLAN_WAYPOINT_SPACING` | 0.4m | 路径点间距 |
| `PLAN_MAX_GOAL_DIST` | 25m | 超过此 Dijkstra 距离视为不可达 |
| `GOAL_BLACKLIST_COUNT/RADIUS` | 3 / 2m | 失败目标拉黑，避免死磕同一目标/角落 |
| `STUCK_NO_PROGRESS_SEC` | 4s | 目标距离无下降的卡死判定窗口 |
| `NAV_SPEED_MAX` | 0.30 m/s | 最大前进速度 |
| `NAV_TURN_MAX` | 1.0 rad/s | 最大角速度 |
| `AVOID_EMERGENCY_DIST` | 0.75m | 局部避障紧急净距（同 bridge 巡逻模式） |
| `AVOID_SLOW_DIST` | 1.50m | 局部避障减速区（同 bridge 巡逻模式） |

路径规划核心逻辑在 `slam/bridge/explore_planner.py`（无 ROS 依赖，可单测）；
局部避障复用 `slam/bridge/local_avoidance.py`。两者与 `--patrol` 模式共用同一套
脱困状态机，行为一致。

### 巡视模式参数（由 `bridge_core.py` 实现，场景入口为 `bridge_ariac.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `PATROL_V` | ARIAC 0.70 m/s | 巡视线速度；ARIAC 在 10Hz 扫描下每帧约移动 7cm |
| `PATROL_W` | 0.6 rad/s | 巡视角速度 |
| `OBSTACLE_DIST` | 0.75m | 障碍侵入前向通行走廊时的紧急避障距离 |
| `OBSTACLE_SLOW_DIST` | 1.50m | 通行走廊内开始减速的距离 |
| `front_half_angle_deg` | 42° | 外层动态障碍监测扇形半角，不单独触发恢复 |
| `front_corridor_half_width` | 0.42m | 内层通行走廊半宽（碰撞盒半宽 0.313m） |

ARIAC 固定巡视会沿 `x=19.9` 进入东侧货架连廊，从 `y=-2` 扫描到
`y=17` 后原路退出。该中心线在当前模型中的最小水平净空约为 `0.95m`；不要
把扫描线移到更靠墙的 `x=22.x`，那里会被 `shelf_5/6` 挤窄。

局部避障采用双区域检测：外层扇形保留侧前方障碍信息，用于监测和选择绕行
方向；只有障碍侵入机器人实际扫掠宽度时才触发减速或恢复，避免平行货架在
狭长走廊中造成误触发。恢复转向后会锁定绕行方向，直到原障碍所在侧连续清空，
不会再按固定前进时间直接转回目标路线。

### rtabmap 参数（`slam/mapping/rtabmap_params.yaml`）

ICP-SLAM 核心参数，通常不需要修改。如需调整建图质量：
- 增大 `Grid/CellSize` 可降低分辨率但加速建图
- 调整 `Reg/Strategy` 改变帧间配准策略

默认启动使用无噪声仿真里程计，并限制 ICP 的对应距离和最大修正量，减少后期
回环时点云整体跳变。不要把 `--odom-noise` 用于正常建图；它只用于测试回环
在有随机游走时的容错能力。

### 离线决策回放参数（`slam/mapping/replay_decisions.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--map` | `maps/ariac/ariac_map_3d.yaml` | 成品栅格图（yaml+pgm），ground truth |
| `--start` | `4.0,4.6` | 机器人起点（世界系 x,y），与 `run_slam_3d.sh` ariac 默认起点一致 |
| `--max-steps` | 800 | 最大仿真步数（每步 0.1s），300 步约 1.5 分钟 |
| `--out` | `/tmp/opencode/replay_coverage.png` | 结束时输出的覆盖图（蓝=剩余 unknown） |

### 坐标系说明

场景使用两个坐标系：

- **世界坐标系（MuJoCo）**：ARIAC 场景中狗的起点在 `(4.0, 4.6)` 位置
- **地图坐标系（map）**：狗的起点为原点 `(0, 0)`

转换公式：

```
map = world - (4.0, 4.6)
```

常用目标点示例（map 坐标，用 `--world` 时 send_goal 会自动换算）：

| 目标位置 | map 坐标 | 世界坐标 |
|---------|---------|---------|
| 起点（狗初始位置） | (0, 0) | (4.0, 4.6) |
| 装配桌前 | (4.5, 1.0) | (8.5, 5.6) |
| 巡检走廊中段 | (2.0, 1.0) | (6.0, 5.6) |

### 导航状态与话题

监控导航状态：

```bash
ros2 topic echo /nav_status
```

状态含义：

| 状态 | 含义 |
|------|------|
| `IDLE` | 空闲，等待目标 |
| `PLANNING` | 正在规划路径 |
| `FOLLOWING` | 跟随路径中 |
| `ALIGNING` | 到位后原地调整朝向 |
| `ARRIVED` | 已到达目标 |
| `STUCK` | 被堵，正在重规划 |
| `UNREACHABLE` | 目标不可达（连续 3 次规划失败） |
| `DYNAMIC_AVOID` | 动态障碍绕行中 |

运行时的关键话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pointcloud` | PointCloud2 | 3D lidar 点云（10 Hz） |
| `/odom` | Odometry | 里程计（50 Hz） |
| `/cmd_vel` | Twist | 速度指令 |
| `/nav_goal` | PoseStamped | 导航目标点（map 系） |
| `/nav_path` | Path | 规划路径 |
| `/nav_status` | String | 导航状态 |
| `/rtabmap/map` | OccupancyGrid | 2D 投影栅格 |
| `/cloud_map` | PointCloud2 | 保留水平表面的 3D 点云地图 |
| `/frontier_markers` | MarkerArray | Frontier 可视化（--explore 模式） |

## 备注

### 故障排查

**问题：RViz 显示 "No map received"**

**原因**：导航节点未收到地图数据

**解决**：
1. 确认方案 A 中 `run_nav_saved.sh` 已完整启动
2. 检查终端输出是否有 "加载保存地图" 的日志
3. 如果使用方案 B，等待狗巡视几秒让地图构建起来

**问题：机器人不动**

**原因**：未发送目标点或目标点在当前位置

**解决**：
1. 确认已发送 `/nav_goal` topic
2. 检查 `/nav_status` 是否为 `FOLLOWING`
3. 尝试发送一个离当前位置较远的目标点（如 `x: 4.5, y: 1.0`）

**问题：路径规划失败（状态显示 UNREACHABLE）**

**原因**：目标点被障碍物包围或超出地图范围

**解决**：
1. 在 RViz 中查看地图，确认目标点在白色（自由空间）区域
2. 避免将目标点设置在黑色（障碍物）或灰色（未知）区域
3. 检查目标点坐标是否在地图范围内

**问题：机器人卡在障碍物边缘**

**原因**：膨胀半径不足或动态障碍物（如移动的料车）

**解决**：
1. 导航节点会自动检测被堵并重规划（等待 3 秒）
2. 如果持续卡住，手动发送新目标点绕过障碍
3. 动态障碍物需要使用方案 B 的实时建图模式

**问题：MuJoCo 窗口卡死或黑屏**

**原因**：GPU 驱动或窗口管理器问题

**解决**：
1. 按 `Ctrl+C` 停止所有进程
2. 确认 NVIDIA 驱动正常：`nvidia-smi`
3. 重新启动，如果问题持续可尝试不带 `--view` 参数（无 GUI 模式）

### 技术细节

- **全局规划算法**：Lazy Theta*（any-angle 路径，比传统 A* 更平滑）
- **局部跟随算法**：Pure Pursuit（支持全向底盘侧移）
- **障碍膨胀半径**：0.37m（机器人半径 0.32m + 安全余量 0.05m）
- **重规划触发条件**：偏离路径 > 0.6m 或被堵 > 3s
- **到点判定半径**：0.22m

相关代码文件：
- `slam/navigation/nav_p2p.py` — 导航节点主逻辑
- `slam/bridge/bridge_ariac.py` — MuJoCo 仿真与 ROS 2 桥接
- `slam/mapping/rtabmap_params.yaml` — 3D SLAM 参数配置
- `maps/README.md` — 地图文件说明

### 无 ROS 快速回归

`./test/sh/common/avoidance/run_avoidance_test.sh quick`：
- `python3.8 test/py/common/avoidance/verify_avoidance_code.py`   # 局部避障逻辑
- `python3.8 test/py/common/navigation/test_explore_planner.py`   # 全局规划器 + 局部避障接线
