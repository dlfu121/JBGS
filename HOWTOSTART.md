# 快速启动指南

本文档说明如何启动仿真环境和导航系统，实现"发送目标点坐标 → 机器狗自动导航"的功能。

---

## 方案 A：使用已保存地图导航（推荐新手测试）

**特点**：不需要重新建图，狗站在原地等你发目标点，立即导航。

### 1. 启动仿真 + 导航系统

在终端执行：

```bash
cd /home/ee304/jbgs/Main/ZYXBedrooom/worker_scene
./slam/run_nav_saved.sh --view
```

启动后会打开 3 个窗口：
- **MuJoCo 仿真窗口**：3D 场景，狗站在起点
- **RViz 可视化窗口**：显示地图、机器人位置、规划路径
- **终端输出**：系统状态和使用说明

### 2. 发送导航目标点

另开一个新终端，执行：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

**参数说明**：
- `x: 4.5` — 目标点 X 坐标（map 系，单位：米）
- `y: 1.0` — 目标点 Y 坐标
- `orientation: {w: 1.0}` — 目标朝向（四元数，w=1 表示 0° 朝向）

### 3. 观察导航过程

- **RViz**：绿色路径线显示规划结果，机器人沿路径移动
- **MuJoCo**：3D 仿真窗口实时显示狗的运动
- **终端输出**：显示导航状态（PLANNING/FOLLOWING/ARRIVED/STUCK）

### 4. 监控导航状态

查看当前导航状态：
```bash
source /opt/ros/foxy/setup.bash
ros2 topic echo /nav_status
```

状态含义：
- `IDLE` — 空闲，等待目标
- `PLANNING` — 正在规划路径
- `FOLLOWING` — 跟随路径中
- `ARRIVED` — 已到达目标
- `STUCK` — 被堵，正在重规划
- `UNREACHABLE` — 目标不可达（连续 3 次规划失败）

### 5. RViz 图形化发送目标点

除了命令行，也可以在 RViz 中点击发送目标：

1. 点击工具栏的 **"2D Goal Pose"** 按钮（或按快捷键 `G`）
2. 在地图上点击目标位置
3. 拖动鼠标设置目标朝向，松开即发送

### 6. 停止系统

在启动终端按 `Ctrl+C`，会自动清理所有进程。

---

## 方案 B：实时建图 + 导航

**特点**：狗先自动巡视场景建图，你可以在建图过程中或完成后发送目标点。适合需要更新地图或首次建图的情况。

### 1. 启动 3D SLAM 建图

**终端 1**：
```bash
cd /home/ee304/jbgs/Main/ZYXBedrooom/worker_scene
./slam/run_slam_3d.sh --view
```

狗会自动沿预设航点巡视，实时构建 3D 点云地图并投影为 2D 栅格地图。

### 2. 启动导航节点

**终端 2**（等狗开始巡视几秒后再执行）：
```bash
source /opt/ros/foxy/setup.bash
cd /home/ee304/jbgs/Main/ZYXBedrooom/worker_scene
python3.8 nav_p2p.py
```

### 3. 发送目标点

**终端 3**：
```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

导航节点会在收到目标后，用当前已建立的地图规划路径并导航。

### 4. 保存新地图（可选）

如果想保存新建的地图供后续使用：

```bash
cd /home/ee304/jbgs/Main/ZYXBedrooom/worker_scene
./slam/save_map_3d.sh
```

地图会保存到 `maps/ariac/ariac_map_3d.pgm` 和 `maps/ariac/ariac_map_3d.yaml`。

---

## 坐标系说明

场景使用两个坐标系：

- **世界坐标系（MuJoCo）**：ARIAC 场景中狗的起点在 `(4.0, 4.6)` 位置
- **地图坐标系（map）**：狗的起点在原点 `(0, 0)`

**转换公式**：
```
map_x = world_x + 8.5
map_y = world_y
```

**常用目标点示例**（map 坐标）：

| 目标位置 | map 坐标 | 世界坐标 |
|---------|---------|---------|
| 主工作台（螺丝刀位置） | (0, 0) | (-8.5, 0) |
| 走廊中段 | (4.5, 1.0) | (-4.0, 1.0) |
| 东北角高台 | (10.75, 2.35) | (2.25, 2.35) |
| 西墙货架 | (1.0, 1.4) | (-7.5, 1.4) |

---

## 故障排查

### 问题：RViz 显示 "No map received"
**原因**：导航节点未收到地图数据  
**解决**：
1. 确认方案 A 中 `run_nav_saved.sh` 已完整启动
2. 检查终端输出是否有 "加载保存地图" 的日志
3. 如果使用方案 B，等待狗巡视几秒让地图构建起来

### 问题：机器人不动
**原因**：未发送目标点或目标点在当前位置  
**解决**：
1. 确认已发送 `/nav_goal` topic
2. 检查 `/nav_status` 是否为 `FOLLOWING`
3. 尝试发送一个离当前位置较远的目标点（如 `x: 4.5, y: 1.0`）

### 问题：路径规划失败（状态显示 UNREACHABLE）
**原因**：目标点被障碍物包围或超出地图范围  
**解决**：
1. 在 RViz 中查看地图，确认目标点在白色（自由空间）区域
2. 避免将目标点设置在黑色（障碍物）或灰色（未知）区域
3. 检查目标点坐标是否在地图范围内（通常 x: -3~11, y: -11~3）

### 问题：机器人卡在障碍物边缘
**原因**：膨胀半径不足或动态障碍物（如移动的料车）  
**解决**：
1. 导航节点会自动检测被堵并重规划（等待 3 秒）
2. 如果持续卡住，手动发送新目标点绕过障碍
3. 动态障碍物需要使用方案 B 的实时建图模式

### 问题：MuJoCo 窗口卡死或黑屏
**原因**：GPU 驱动或窗口管理器问题  
**解决**：
1. 按 `Ctrl+C` 停止所有进程
2. 确认 NVIDIA 驱动正常：`nvidia-smi`
3. 重新启动，如果问题持续可尝试不带 `--view` 参数（无 GUI 模式）

---

## 进阶使用

### 查看规划路径

```bash
ros2 topic echo /nav_path
```

### 查看实时点云（方案 B）

在 RViz 中添加 PointCloud2 显示插件：
1. 点击左下角 "Add" 按钮
2. 选择 "PointCloud2"
3. Topic 选择 `/pointcloud`

### 手动控制机器人（测试用）

发送速度指令：
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}"
```

**注意**：手动控制会自动停止导航的巡视模式。

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

生成的 `frames.pdf` 显示完整的坐标系变换关系。

---

## 技术细节

- **全局规划算法**：Lazy Theta*（any-angle 路径，比传统 A* 更平滑）
- **局部跟随算法**：Pure Pursuit（支持全向底盘侧移）
- **障碍膨胀半径**：0.37m（机器人半径 0.32m + 安全余量 0.05m）
- **重规划触发条件**：偏离路径 > 0.6m 或被堵 > 3s
- **到点判定半径**：0.22m

相关代码文件：
- `nav_p2p.py` — 导航节点主逻辑
- `slam_bridge_3d.py` — MuJoCo 仿真与 ROS 2 桥接
- `slam/rtabmap_params.yaml` — 3D SLAM 参数配置
- `maps/README.md` — 地图文件说明

---

**祝测试顺利！** 如有问题，参考终端输出日志或查看上述故障排查部分。
