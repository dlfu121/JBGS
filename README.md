# MuJoCo 3.11.0 x LeRobot 0.6.1 环境配置指南

基于 MuJoCo 物理引擎和 LeRobot 框架的机器人仿真遥操作与数据采集系统。

## 系统要求

| 组件      | 版本           |
| ------- | ------------ |
| OS      | Ubuntu 24.04 |
| Python  | 3.12         |
| MuJoCo  | 3.11.0       |
| LeRobot | 0.6.1        |
| 显示器     | 需要可用桌面环境     |

## 零、在wsl安装Ubantu24.04

在Powershell中输入

```bash
wsl --install -d Ubuntu-24.04
```

设置用户名和密码后会直接进入Ubantu24.04系统，退出wsl，并且重新进入。

在Powershell中继续输入

```bash
wsl --set-default Ubuntu-24.04
```

设置Ubantu24.04为默认版本，之后再输入

```bash
wsl
```

进入Ubantu24.04

## 一、安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  python3.12-venv \
  python3-pip \
  libgl1-mesa-dev \
  libglfw3 \
  libglfw3-dev \
  git
```

> `libglfw3` 和 `libglfw3-dev` 用于 MuJoCo 窗口渲染。

## 二、创建 Python 虚拟环境

```bash
cd /path/to/KeyCollect
python3.12 -m venv .venv
source .venv/bin/activate
python --version
```

## 三、安装 Python 包

```bash
python -m pip install --upgrade pip
python -m pip install matplotlib
python -m pip install mujoco==3.11.0
python -m pip install lerobot==0.6.1
python -m pip install pynput
python -m pip install 'lerobot[dataset]==0.6.1'
python -m pip install lerobot[viz]
python -m pip install -e ./lerobot_robot_mujoco
python -m pip install -e ./lerobot_teleoperator_keyboard_mouse
```

## 四、启动屏幕渲染

```bash
source .venv/bin/activate
export MUJOCO_GL=glfw
python3 scripts/viewer.py
```

默认遥操作使用 `assets/scene/rm65_dexhand_scene.urdf`。单独查看场景时可以指定这个文件：

```bash
python3 scripts/viewer.py assets/scene/rm65_dexhand_scene.urdf
```

也可以指定自己的场景文件：

```bash
python3 scripts/viewer.py assets/scene/your_scene.xml
```

如果你已经在桌面环境里运行，一般不需要再额外设置 `DISPLAY`。

## 五、键盘和鼠标遥操作

直接启动：

```bash
source .venv/bin/activate
export MUJOCO_GL=glfw
python3 scripts/teleop.py
```

按键映射：

| 输入      | 动作       |
| ------- | -------- |
| `W/S`   | 前后       |
| `A/D`   | 左右       |
| `Q/E`   | 上下       |
| `Z/X`   | 手腕旋转     |
| `R/F`   | 夹爪开合     |
| `Space` | 按住才会输出动作 |
| `Esc`   | 退出遥操作    |

当前默认硬件是：

```text
机械臂：assets/arm/RM65-6F.urdf
灵巧手：assets/hand/dexhand021_right_simplified.urdf
组合场景：assets/scene/rm65_dexhand_scene.urdf
```

## 六、更换硬件

当前项目支持把机械臂和末端手爪分别放在 `assets/arm/` 和 `assets/hand/`，再生成一个 MuJoCo 可加载的组合场景。

### 6.1 当前目录约定

```text
assets/
├── arm/
│   ├── RM65-6F.urdf
│   ├── base_link.STL
│   ├── link_1.STL
│   └── ...
├── hand/
│   ├── dexhand021_right_simplified.urdf
│   ├── right_hand_base.STL
│   ├── r_f_link1_1.STL
│   └── ...
└── scene/
    ├── demo_scene.xml
    └── rm65_dexhand_scene.urdf
```

### 6.2 重新生成组合硬件场景

如果替换了机械臂或手，运行：

```bash
python3 scripts/build_hardware_scene.py
```

默认会读取：

```text
assets/arm/RM65-6F.urdf
assets/hand/dexhand021_right_simplified.urdf
```

并生成：

```text
assets/scene/rm65_dexhand_scene.urdf
```

也可以手动指定文件：

```bash
python3 scripts/build_hardware_scene.py \
  --arm assets/arm/your_arm.urdf \
  --hand assets/hand/your_hand.urdf \
  --arm-mount-link link_6 \
  --hand-root-link right_hand_base \
  --hand-mount-xyz 0 0 -0.08 \
  --hand-mount-rpy 0 0 0 \
  --output assets/scene/your_hardware_scene.urdf
```

参数说明：

| 参数                 | 说明                 |
| ------------------ | ------------------ |
| `--arm`            | 机械臂 URDF 路径        |
| `--hand`           | 手或夹爪 URDF 路径       |
| `--arm-mount-link` | 手要挂到机械臂哪个 link 上   |
| `--hand-root-link` | 手模型的根 link         |
| `--hand-mount-xyz` | 手相对机械臂末端的安装偏移，单位米  |
| `--hand-mount-rpy` | 手相对机械臂末端的安装姿态，单位弧度 |
| `--output`         | 生成的组合场景路径          |

如果手和机械臂没有贴紧，优先调 `--hand-mount-xyz`。例如手离机械臂太远，可以继续减小 z 偏移：

```bash
python3 scripts/build_hardware_scene.py --hand-mount-xyz 0 0 -0.12
```

### 6.3 更新遥操作配置

生成新场景后，检查并更新 `config/robot.yaml`：

```yaml
scene_path: assets/scene/rm65_dexhand_scene.urdf

arm_joint_names:
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6

gripper_joint_names:
  - r_f_joint1_1
  - r_f_joint1_2
  - r_f_joint2_1
  - r_f_joint2_2
```

`scripts/teleop.py` 当前默认也使用这套 RM65 + 右手命名。如果换了新的硬件，需要同步改脚本里的：

```text
arm_joints
gripper_joints
--ee-body
```

### 6.4 验证新硬件是否可加载

```bash
python3 - <<'PY'
import mujoco
m = mujoco.MjModel.from_xml_path("assets/scene/rm65_dexhand_scene.urdf")
print("joints", m.njnt, "geoms", m.ngeom, "cameras", m.ncam)
PY
```

如果能正常输出关节和几何数量，就说明 MuJoCo 可以加载该硬件场景。

### 七、摄像头组件

修改摄像头位置：.venv/bin/python tune_camera.py

### 八、数据采集命令

```bash
source .venv/bin/activate

MUJOCO_GL=egl

lerobot-record \
  --robot.type=mujoco \
  --robot.id=rm65_dexhand \
  --robot.calibration_dir=.cache/lerobot/calibration/robots/mujoco \
  --robot.scene_path=assets/scenes/rm65_dexhand_scene.xml \
  --robot.arm_joint_names='["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]' \
  --robot.gripper_joint_names='["r_f_joint1_1","r_f_joint1_2","r_f_joint2_1","r_f_joint2_2","r_f_joint3_1","r_f_joint3_2","r_f_joint4_1","r_f_joint4_2","r_f_joint5_1","r_f_joint5_2"]' \
  --robot.ee_site_name=link_6 \
  --robot.cameras='{"table_camera":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":30},"wrist_overhead_camera":{"type":"opencv","index_or_path":1,"width":640,"height":480,"fps":30}}' \
  --teleop.type=keyboard_mouse \
  --teleop.translation_step_m=0.02 \
  --teleop.rotation_step_rad=0.08 \
  --teleop.gripper_step=0.05 \
  --dataset.repo_id=dlfu121/Industrial \
  --dataset.single_task="这里填上具体任务命令" \
  --dataset.root=data/rm65_dexhand_test \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --play_sounds=false
```

采集后，上传数据：

```bash
hf auth login
```

# ARIAC 实验室场景 - MuJoCo SLAM 仿真

主场景为 ARIAC 2025 实验室，配四足机械狗（双臂双手）进行 3D 激光 SLAM。
当前支持 ARIAC 和 warehouse 两个场景，`run_slam_3d.sh` 默认使用 `ariac`。

## 环境要求

- Python 3.8 + mujoco 3.2.3（ROS Foxy 的 rclpy 只有 cpython-38 扩展）
- ROS 2 Foxy
- `ros-foxy-rtabmap-ros`（3D SLAM，若 apt 无此包需源码编译）

## 快速开始

### 3D 点云 SLAM（rtabmap）

```bash
source /opt/ros/foxy/setup.bash

# ARIAC 资源更新后重新生成 3.2.3 兼容网格和机器人组合场景：
python3.8 model/scenes/build_ariac_compat_meshes.py
python3.8 model/robot/gen_ariac_robot.py

# 一键启动：3D 桥接 + rtabmap + rviz（固定航点巡视）
./slam/run_slam_3d.sh --view --fresh

# Frontier 自主探索建图（无需航点，适用于陌生环境）
./slam/run_slam_3d.sh --view --fresh --explore
```

显式选择场景时使用 `--scene ariac|warehouse`。不传 `--scene` 等同于
`--scene ariac`。只查看纯 ARIAC 场景可运行：

```bash
../../bin/simulate model/scenes/ariac_lab.xml
```

3D 桥接默认发布无噪声仿真里程计，以避免长路径随机游走在回环时造成点云整体
跳变。`--odom-noise` 仅用于回环抗漂移压力测试，不建议正常建图时开启。

`--explore` 模式使用 Frontier Exploration 算法自主建图：
- 自动检测地图中 已知/未知 区域边界（frontier）
- 选择最优 frontier 作为下一个探索目标
- 用 VFH（Vector Field Histogram）局部避障安全导航
- 到达后原地旋转 360° 扫描，再寻找下一个 frontier
- 全部区域覆盖后自动停止

无需手动规划航点，换场景也通用。

巡视/探索完成后保存 3D 地图：

```bash
./slam/save_map_3d.sh           # 默认保存到 ./maps/ariac/
```

输出格式：
- `maps/ariac/ariac_map_3d.yaml` + `.pgm` - 2D 投影栅格
- `maps/ariac/rtabmap.db` - 完整 3D 地图数据库

## 项目目录结构

```
worker_scene/
├── README.md                    # 本文件，项目总览
├── HOWTOSTART.md                # 新手入门指南
├── nav_p2p.py                   # 点对点导航：3D地图 + Lazy Theta* 规划 + 纯追踪跟随
├── send_goal.py                 # 发送导航目标点（支持 map/world 坐标系）
├── view_overview.png            # 场景全局预览图
├── view_top.png                 # 场景俯视图
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
│   │   ├── gen_warehouse_robot.py # 提取机器人注入仓库场景
│   │   ├── gen_ariac_robot.py   # 将机器人和 3D 雷达注入 ARIAC
│   │   ├── ariac_lab_with_robot_3d.xml # SLAM 使用的 ARIAC 组合模型
│   │   ├── robot.xml            # 纯机器人模型
│   │   ├── robot_py38.xml       # Python 3.8 兼容机器人模型
│   │   ├── robot_template_3d_py38.xml   # 通用 3D 机器人模板（Python 3.8 兼容）
│   │   └── warehouse_with_robot_3d.xml   # 仓库 + 机器人 + 3D 雷达
│   └── scenes/                  # 纯场景定义
│       ├── ariac_lab.xml        # 主 ARIAC 纯场景
│       ├── build_ariac_compat_meshes.py # 生成 3.2.3 兼容平面网格
│       └── warehouse_with_obstacles_mujoco.xml  # 仓库场景（含障碍物）
│
├── slam/                        # SLAM 建图与导航相关
│   ├── bridge/                  # MuJoCo → ROS 桥接节点
│   │   ├── bridge_ariac.py      # 默认 ARIAC 场景 3D 桥接
│   │   ├── bridge_warehouse.py  # 仓库场景 3D 桥接
│   │   └── local_avoidance.py   # 局部避障状态机
│   ├── run_slam_3d.sh           # 3D SLAM 一键启动
│   ├── frontier_explorer.py     # Frontier 自主探索 + VFH 避障节点
│   ├── run_nav.sh               # 导航启动（在线建图模式）
│   ├── run_nav_saved.sh         # 导航启动（加载已保存地图）
│   ├── restart_nav.sh           # 重启导航栈
│   ├── save_map_3d.sh           # 保存 3D 地图到 maps/
│   ├── view_map.sh              # 离线查看已保存地图
│   ├── rtabmap_3d.launch.py     # rtabmap ROS2 launch
│   ├── rtabmap_params.yaml      # rtabmap ICP-SLAM 参数
│   ├── slam_3d.rviz             # 3D SLAM RViz 配置
│   └── view_map.rviz            # 地图查看 RViz 配置
│
└── maps/                        # 建图输出（SLAM 生成的地图文件）
    ├── ariac/                   # ARIAC 场景地图
    └── warehouse/               # warehouse 场景地图
│
```

## 工作流说明

### 模型生成（mujoco 3.2.3 环境）

基准模型由各场景的生成脚本预先生成，ROS 侧使用对应的 Python 3.8 兼容 XML。

ARIAC 和 warehouse 的组合 3D 模型分别由 `gen_ariac_robot.py` 和
`gen_warehouse_robot.py` 生成。运行时禁用 XML 中逐束传感器，改用
`mj_multiRay` 批量投射 64 层 × 360 束（23040 束），并验证 MuJoCo 能正常加载。

`build_robot.py` 是为 mujoco ≥3.6 编写的装配脚本，使用 MjSpec 类方法 API。
当前 3.2.3 环境无法运行它，保留作为文档和未来升级参考。

### ROS 话题

**3D 模式：**
- `/pointcloud` (PointCloud2) — 最多 23040 个有效命中点，10 Hz
- `/odom` (Odometry) — 50 Hz
- `/cloud_map` (PointCloud2) — 保留水平表面的 3D 点云地图
- `/rtabmap/map` (OccupancyGrid) — 2D 投影栅格
- TF: `map → odom → base_footprint → base_link → lidar3d`

### 路径规划

建图完成后的地图可直接用于 nav2：

```bash
# 加载已保存的 2D 栅格地图
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=maps/ariac/ariac_map_3d.yaml \
  -p use_sim_time:=true
```

rtabmap 也支持定位模式（不再建图，只做位姿估计）：
在 `slam/rtabmap_params.yaml` 中设置 `Mem/IncrementalMemory: "false"`，
并移除 launch 文件中的 `--delete_db_on_start` 参数。

## 自主巡检能力报表

巡检验收指标、口径和批量报表生成方法见
[`docs/inspection_capability_metrics.md`](docs/inspection_capability_metrics.md)。
使用 `_ultimate_task/inspect/record` 中的拍摄 JSON/PNG 生成可插入技术文档的
图表和表格：

```bash
python3 tools/inspection_report.py --out-dir inspection_report
```

输出 `inspection_dashboard.png`、`inspection_metrics.csv`、
`inspection_summary.md` 和 `inspection_summary.json`。如果同时提供包含
`time_s,x_m,y_m,collision,dynamic_avoid` 列的导航遥测 CSV，报表会追加路径长度、
运行时长、碰撞和动态避障统计。

如需批量展示静态地图上的巡检规划路径，可运行
`python3 tools/inspection_path_sim.py --runs 6`；脚本会在
`inspection_report/path_simulation/` 下生成各次截图和
`inspection_paths_montage.png` 汇总大图。该结果用于离线规划可达性展示，在线
导航和避障效果仍应以带遥测的 ROS 重复实验为准。

# 数据采集流程

四个终端必须按照下面的顺序启动。启动后的终端必须保持运行，不要关闭。

## 终端 1：roscore

加载 ROS Noetic：

```bash
source /opt/ros/noetic/setup.bash
```

启动 ROS Master：

```bash
roscore
```

正常输出包括：

```text
started core service [/rosout]
ROS_MASTER_URI=http://localhost:11311/
```

保持终端 1 运行。

## 终端 2：mocap_joint_publisher

进入动捕 Catkin 工作空间：

```bash
cd "$HOME/dongziyue/mocap_joint_publisher"
```

加载 ROS Noetic：

```bash
source /opt/ros/noetic/setup.bash
```

加载动捕工作空间：

```bash
source "$HOME/dongziyue/mocap_joint_publisher/devel/setup.bash"
```

可选检查包是否可见：

```bash
rospack find mocapapi
```

正常输出：

```text
/home/ee304/dongziyue/mocap_joint_publisher/src/mocapapi
```

启动动捕节点：

```bash
rosrun mocapapi mocap_joint_publisher
```

注意：ROS 包名是 `mocapapi`，节点名才是 `mocap_joint_publisher`。不要使用 `roslaunch mocap_joint_publisher mocap_joint_publisher.launch`。

正常现象：

- 节点持续运行，没有 traceback 或自动退出。
- Axis Studio 开始广播后，节点持续接收数据。
- ROS 中出现 `/right_wrist_pose` 和 `/right_joint_poses`。
- 两个话题的频率约为 50 Hz。

保持终端 2 运行。

## 终端 3：ROSBridge

加载 ROS Noetic：

```bash
source /opt/ros/noetic/setup.bash
```

启动 ROSBridge WebSocket：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

正常输出会说明 WebSocket 已在 `9090` 端口启动或监听。

保持终端 3 运行。

## 终端 4：验证数据并启动 KeyCollect（本终端）

先加载 ROS：

```bash
source /opt/ros/noetic/setup.bash
```

确认两个动捕话题存在：

```bash
rostopic list | grep -E '/right_wrist_pose|/right_joint_poses'
```

正常输出：

```text
/right_joint_poses
/right_wrist_pose
```

检查手腕频率：(也可以不检查，基本没问题)

```bash
rostopic hz /right_wrist_pose
```

正常值约为 50 Hz。看到稳定频率后按 `Ctrl+C` 结束检查。

检查手指频率：

```bash
rostopic hz /right_joint_poses
```

正常值约为 50 Hz。看到稳定频率后按 `Ctrl+C` 结束检查。

检查一帧手指数据：

```bash
rostopic echo -n 1 /right_joint_poses
```

正常情况下会输出一条 `Float32MultiArray` 消息，其 `data` 包含 57 个数值。

确认 ROSBridge 正在监听 9090：

```bash
ss -ltn | grep ':9090'
```

进入 KeyCollect：

```bash
cd "$HOME/dongziyue/KeyCollect"
```

初始化 Conda shell：

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
```

激活 KeyCollect 环境：

```bash
conda activate keycollect
```

确认环境：

```bash
echo "$CONDA_DEFAULT_ENV"
```

正常输出：

```text
keycollect
```

可选检查场景执行器数量：

```bash
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/scenes/rm65_dexhand_scene.xml'); print('actuators =', m.nu)"
```

正常输出：

```text
actuators = 26
```
> ok到这里就开完了所有的终端，可以开始调试啦:)


# 全链路运行指南

> 路径统一使用真实目录 `model.test`（点），不是 `model_test`（下划线）。

## 四个终端启动流程

### 终端 1：启动 MuJoCo 场景 + 导航 + 建图

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
./slam/run_nav_saved.sh --scene ariac --task vla --view
```

### 终端 2：启动 ACT 推理服务

```bash
source /home/ee304/miniforge3/etc/profile.d/conda.sh
conda activate keycollect
cd /home/ee304/dongziyue/KeyCollect
/home/ee304/miniforge3/envs/keycollect/bin/python \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_server.py \
  --checkpoint /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/KeyCollect/outputs/train/act_rm65_dexhand \
  --dataset-root /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/KeyCollect/data/rm65_dexhand_merged \
  --device cuda
```

注意：`--checkpoint` 传的是 `act_rm65_dexhand` 目录本身，脚本会自动拼接
`checkpoints/040000/pretrained_model`；不要再加 `/checkpoints`。

### 终端 3：启动 ACT 执行客户端（自动弹出两路相机）

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3.8 \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_bridge_client.py \
  --execute
```

### 终端 4：发布桌前导航目标

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
python3.8 _ultimate_task/vla/run_vla.py
```

## 相机视角

抓取阶段不需要再增加摄像头。bridge 已经发布两路现有 RGB 图像：

```text
/act/table_image   桌面/第三视角相机
/act/wrist_image   右腕/手眼相机
```

启动 ACT 执行客户端（终端 3）后，会自动弹出这两个窗口。

如果是在无显示器、SSH 或只想后台跑的环境，可以关闭自动窗口：

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3.8 \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_bridge_client.py \
  --execute \
  --no-camera-view
```

确认两路图像正在发布：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic hz /act/table_image
ros2 topic hz /act/wrist_image
```

如果自动窗口没有画面，先确认仿真 bridge 是用 VLA 任务启动的，并且日志里没有
`ACT camera render failed`。

## 常见错误

- 目录写错：是 `model.test`，不是 `model_test`。
- `act_server.py` 路径写错：是 `_ultimate_task/vla/act_server.py`，不要多写
  `worker_scene/_ultimate_task`。
- `--checkpoint` 多写 `/checkpoints`，会导致找不到 `pretrained_model`。
- 终端 3 命令换行断开：`/usr/bin/python3.8`、脚本路径、`--execute` 必须用 `\`
  连成同一条命令，否则会被当成三条命令执行。
