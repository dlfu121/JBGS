# ARIAC Ultimate Task 使用说明

`_ultimate_task` 包含两个相互隔离的上层任务：

- `vla`：导航到装配桌前，使用 `table_camera` 和
  `wrist_overhead_camera` 获取观测，并由 KeyCollect/LeRobot ACT 控制右臂和
  右侧灵巧手完成螺丝刀任务。
- `inspect`：依次前往 cabinet、tank 和 hydrant，控制左臂举起
  `lefthand_camera` 拍摄仪表，随后恢复巡检行走姿态。

两项任务复用 MuJoCo、点云、里程计和 `nav_p2p.py`，但任务配置已经分开：

| 配置 | VLA | inspect |
|---|---|---|
| 启动配置 | `--task vla` | `--task inspect` |
| 任务定义 | `_ultimate_task/vla/task_definition.py` | `_ultimate_task/inspect/task_definition.py` |
| 场景 | 随机螺丝刀 VLA 场景 | 普通 ARIAC 巡检场景 |
| 导航朝向 | 桌前固定 `+90°`，可见机头朝世界 `+Y` | 各点读取 `inspection_route.json` |
| 相机 | `table_camera`、`wrist_overhead_camera` | `lefthand_camera` |
| 机械臂 | 右臂 ACT 微分 IK | 左臂关节空间平滑插值 |
| 控制接口 | `/act/*` | `/inspection/*` |

同一套仿真一次只能选择一个任务配置。不要同时运行 `run_vla.py` 和
`run_inspection.py`，因为二者虽然机械臂与相机接口已经隔离，仍会共用底层
`/nav_goal` 和 `/cmd_vel` 导航接口。

## 环境与工作目录

除非命令中特别说明使用 KeyCollect 环境，worker_scene 侧均使用 ROS 2 Foxy
和 Python 3.8：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
```

## 一、VLA 螺丝刀任务

### 1. 启动 VLA 场景与保存地图导航

终端 1：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
./slam/run_nav_saved.sh --scene ariac --task vla --view
```

启动器会依次完成：

1. 生成 ARIAC 机器人组合场景；
2. 随机生成桌面上的螺丝刀位置和接近 X 轴的工具朝向；
3. 加载 VLA 专用 MuJoCo 场景；
4. 启动 bridge、保存地图导航、`map→odom` 静态 TF 和 RViz2；
5. 只开放 VLA 的 `/act/*` 机械臂与相机接口。

不设置种子时，每次启动都会重新采样螺丝刀。需要复现实验时：

```bash
VLA_SEED=20260906 \
./slam/run_nav_saved.sh --scene ariac --task vla --view
```

`VLA_SEED` 只能与 `--task vla` 一起使用；在 inspect 配置下使用会直接报错。

### 2. 导航到桌前并正对桌面

等待终端 1 提示导航已启动后，在终端 2 运行：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
python3.8 _ultimate_task/vla/run_vla.py
```

`run_vla.py` 会：

1. 计算桌前右臂操作基准；
2. 把 world 坐标换算为保存地图的 map 坐标；
3. 发布带最终朝向的 `/nav_goal`；
4. 等待 `/nav_status` 从规划、跟随进入 `ALIGNING`；
5. 保持 yaw 约束，直到导航器发布 `ARRIVED`；
6. 停车并继续保持节点运行。

当前目标应输出为：

```text
机器人桌前目标: world=(7.225, 9.730), yaw=+90.0 deg
```

机器狗 XML 自带 180° 模型参考旋转。在本工程的导航定义中，`yaw=0°` 对应
可见机头朝世界 `+X`，所以 `yaw=+90°` 才表示可见机头朝世界 `+Y`、正对桌面。
此时右臂基座位于 `(7.400, 9.830, 1.240)`。

如果希望到达后立即退出目标发布器：

```bash
python3.8 _ultimate_task/vla/run_vla.py --exit-on-arrival
```

### 3. VLA 摄像头和机械臂定义

VLA 使用以下两路 `480×640` RGB 图像：

```text
table_camera
wrist_overhead_camera
```

相机名称、局部位置、四元数外参、VLA 初始机械臂姿态、动作限幅及阻尼最小
二乘 IK 参数统一定义在：

```text
_ultimate_task/vla/task_definition.py
```

VLA 初始机械臂姿态为：

```text
[3.141593, -0.401426, -1.727876, 0.0, 0.471239, 3.141593]
```

该配置不读取 inspect 的巡检姿态、相机外参或关节插值参数。

### 4. KeyCollect ACT 环境

worker_scene 保留 Python 3.8、ROS 2 Foxy 和本工程 MuJoCo 环境；LeRobot ACT
在独立的 KeyCollect Python 3.12 环境运行。不要在 ROS Foxy 的 Python 3.8
进程内直接导入 LeRobot 0.6.1。

当前可复用环境：

```text
/home/ee304/miniforge3/envs/keycollect
Python 3.12.13
MuJoCo 3.11.0
LeRobot 0.6.1
PyTorch 2.7.1+cu128
```

激活并检查：

```bash
source /home/ee304/miniforge3/etc/profile.d/conda.sh
conda activate keycollect
cd /home/ee304/dongziyue/KeyCollect

python - <<'PY'
import torch, mujoco, lerobot
print("torch:", torch.__version__)
print("cuda:", torch.cuda.is_available())
print("mujoco:", mujoco.__version__)
print("lerobot:", getattr(lerobot, "__version__", "unknown"))
PY
```

### 5. 单独验证 ACT checkpoint

以下步骤只验证 KeyCollect 自己的场景、checkpoint 和渲染环境，不会控制
worker_scene 中的机器人，因此不能作为最终闭环验收。

GPU/EGL：

```bash
cd /home/ee304/dongziyue/KeyCollect
MUJOCO_GL=egl python scripts/infer_mujoco.py \
  --checkpoint outputs/train/act_rm65_dexhand \
  --dataset-root data/rm65_dexhand_merged \
  --device cuda \
  --render-backend egl \
  --headless \
  --require-gpu-rendering \
  --random-seed 42
```

CPU/OSMesa：

```bash
MUJOCO_GL=osmesa python scripts/infer_mujoco.py \
  --checkpoint outputs/train/act_rm65_dexhand \
  --dataset-root data/rm65_dexhand_merged \
  --device cpu \
  --render-backend osmesa \
  --headless \
  --duration 1 \
  --no-randomize
```

`--device cuda` 要求 `torch.cuda.is_available()` 为真；
`--require-gpu-rendering` 还要求 MuJoCo 实际使用 NVIDIA/EGL。若出现
`gladLoadGL error`、`EGL device` 或 `llvmpipe`，应先检查显示环境和 GPU
驱动，这通常不是 ACT checkpoint 或 worker_scene 控制代码的问题。

### 6. 启动 worker_scene 与 ACT 的闭环 bridge

当前实现由两个进程连接 Python 3.8 与 Python 3.12，使用本机 TCP 和带长度
前缀的 pickle 帧；ACT 服务默认监听 `127.0.0.1:5566`。pickle 只适合本机
可信连接，不应暴露到不可信网络。

确保 VLA 导航栈和 `run_vla.py` 已按前述步骤运行。终端 3 在 KeyCollect
环境先启动 ACT 服务：

```bash
source /home/ee304/miniforge3/etc/profile.d/conda.sh
conda activate keycollect
cd /home/ee304/dongziyue/KeyCollect

/home/ee304/miniforge3/envs/keycollect/bin/python \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_server.py \
  --checkpoint KeyCollect/outputs/train/act_rm65_dexhand \
  --dataset-root Keycollect/data/rm65_dexhand_merged \
  --device cuda
```

无可用 CUDA 时使用 `--device cpu`；有可用 CUDA 时改成 `--device cuda`。
服务启动成功会输出 `ACT server ready`。

终端 4 先启动安全的 dry-run 客户端：

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3.8 \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_bridge_client.py
```

客户端订阅：

```text
/act/observation
/act/table_image
/act/wrist_image
/nav_status
```

它向 ACT 服务发送 39 维状态和两路 RGB 图像，并把服务返回值发布为 26 维
`/act/cartesian_action`。默认 dry-run 会把动作清零，只验证完整通信链路。

确认话题稳定后才允许执行真实动作：

```bash
/usr/bin/python3.8 \
  /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene/_ultimate_task/vla/act_bridge_client.py \
  --execute
```

执行前建议检查：

```bash
ros2 topic list | grep '^/act'
ros2 topic echo --once /act/observation
ros2 topic echo --once /act/cartesian_action
ros2 topic hz /act/table_image
ros2 topic hz /act/wrist_image
```

### 7. ACT 数据接口与安全要求

当前 checkpoint 接口固定为：

```text
state  = 6 arm joint positions
       + 6 arm joint velocities
       + 20 finger joint positions
       + 7 end-effector pose
       = 39 dimensions

action = 6 end-effector deltas + 20 finger deltas
       = 26 dimensions

images = table_camera RGB + wrist_overhead_camera RGB
       = two 480x640 images
```

bridge 只控制右臂 `arm_r/joint_1...joint_6` 和右手 `hand_r`。前 6 维动作
经过 VLA 专用阻尼最小二乘微分 IK，后 20 维积分为手指目标；映射按关节和
执行器名称完成，不依赖 XML 数组顺序，并执行有限值、关节范围和单步变化限幅。

正式验收顺序：

1. 机器人到达桌前且 `/nav_status` 为 `ARRIVED`；
2. worker_scene 能持续发布 39 维状态和两路图像；
3. ACT server 返回形状为 `(26,)` 的有限值动作；
4. dry-run 中动作通信稳定且机械臂不运动；
5. 低速、小动作比例验证坐标方向、尺度和关节映射；
6. 验证工作空间、碰撞风险、相机标定和抓取成功率；
7. 所有安全检查通过后，才允许自动执行完整抓取。

## 二、ARIAC 定点巡检任务

inspect 依次拍摄 cabinet 电压表、tank 压力表和 hydrant 压力表。路线以
ARIAC 起点 `world=(4.0, 4.6)` 作为 `map=(0, 0)` 锚点。

### 1. 巡检路线

| 顺序 | 目标 | 表盘世界坐标 | 停车位 map `(x,y,yaw)` | 地图净距 |
|---:|---|---|---|---:|
| 1 | cabinet 电压表 | `(-3.314, 7.833, 1.951)` | `(-5.814, 3.233, 180°)` | `1.40 m` |
| 2 | tank 压力表 | `(-3.155, -3.500, 1.850)` | `(-5.655, -8.100, 180°)` | `1.40 m` |
| 3 | hydrant 压力表 | `(13.000, -6.799, 3.100)` | `(9.000, -10.000, -90°)` | `1.90 m` |

保存地图按 `0.55 m` 障碍膨胀验证后三点均可达。路线三段长度约为
`7.20 m`、`11.83 m`、`15.49 m`，总计约 `34.51 m`。实际路线、机械臂
拍摄姿态和返回姿态以 `_ultimate_task/inspect/inspection_route.json` 为准。

### 2. 启动巡检场景

终端 1：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
./slam/run_nav_saved.sh --scene ariac --task inspect --view
```

inspect 配置直接加载普通 ARIAC 巡检场景，不执行 VLA 的螺丝刀随机化，只开放
`/inspection/*` 控制和拍摄接口。

### 3. 只验证导航、停车点和朝向

终端 2：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
python3.8 _ultimate_task/inspect/run_inspection.py --navigation-only
```

该模式不会抬臂或拍照，适合逐点确认 cabinet、tank、hydrant 的停车位置和
最终朝向。到达位置后，脚本保留 yaw 四元数约束，等待 `nav_p2p.py` 完成原地
对齐并发布 `ARRIVED`。

### 4. 执行完整巡检

```bash
source /opt/ros/foxy/setup.bash
python3.8 _ultimate_task/inspect/run_inspection.py
```

巡检脚本默认开启 2 个行人，分别位于储罐前和长通道中部。
机械狗进入行人 3.2 m 范围后，该行人沿独立的固定轨迹运动；机械狗停车也
不会冻结横穿进度。行人不会因机械狗接近而停车或让路，机械狗必须通过实时
点云检测行人并主动绕行。

检测到前方障碍后，导航先比较左右绕行通道，再由全向 DWA 跟随局部路径，
表现为向空旷一侧侧移、绕过行人、回到巡检路线。绕行期间前进速度最高
0.55 m/s，静态地图禁行区和实时点云碰撞约束仍然生效；无可通行空间时允许等待。
日志“已触发动态避障”表示进入了 `DYNAMIC_AVOID`，需结合实际位移观察绕行效果。
只验证演示可用 `--dynamic-person --navigation-only`；仅回归静态路线可用
`--no-dynamic-person`。修改避障代码后须重启 bridge 和 `nav_p2p.py`，再运行巡检脚本。

动态行人说明：第二个行人（红色）在 map 系 x=1.0 附近、y≈-9.71 起步向北走上
y≈-8.96 的巡检走廊，随后在该点停住，成为机械狗正前方的静止障碍（轨迹带
`stop_index`，到位后保持不动、不再循环）。进入 3.2 m 触发圈后它正好走到机械狗
前方，机械狗需要主动绕开这个静止行人。

陌生静态障碍测试：第一条巡检路线（出发点 → cabinet）上、map 系约 (-3.0, 1.65)
即世界 (1.0, 6.3) 处放置了一个 `route_obstacle_box`（见 `model/scenes/ariac_lab.xml`
及由它生成的组合场景）。它不在保存的静态地图 `maps/ariac/ariac_map_3d.pgm` 中，
因此全局规划会把该点当作可通行，机械狗只能靠实时点云 + 条件式 DWA 主动绕开，
用于验证对未知静态障碍的避障能力。若以后用 `--fresh` 重新建图，箱子会被并入
地图而变成“已知障碍”，需先从 `ariac_lab.xml` 删除该箱体再建图。

巡检脚本使用以下接口：

- `/nav_goal` (`PoseStamped`)：带最终 yaw 的停车目标；
- `/inspection/left_arm_pose` (`Float64MultiArray`)：左臂六关节目标；
- `/inspection/left_arm_status` (`String`)：bridge 返回实际关节到位状态；
- `/inspection/camera_preview` (`String`)：打开或关闭腕部相机预览；
- `/inspection/capture` (`String`)：包含点位、目标、相机和表盘坐标的拍摄请求；
- `/inspection/capture_done` (`String`)：bridge 返回存图结果；
- `/inspection/dynamic_person`、`/inspection/dynamic_person_seed`：巡检动态行人测试。

每个巡检点的动作顺序为：

1. 导航到停车位；
2. 保留目标 yaw，原地旋转正对拍摄目标；
3. 持续发布零速度并保持机器狗静止；
4. 打开 `lefthand_camera` 预览；
5. 左臂沿关节空间平滑、直接移动到该点拍摄姿态；
6. bridge 根据六关节实际位置确认到位；
7. 额外稳定至少 3 秒后拍摄；
8. 保存 PNG 和同名 JSON 元数据；
9. 左臂恢复统一行走姿态；
10. 确认归位并关闭预览后，再前往下一个点；
11. 完成最后一个点后，根据路线配置返回起点。

机械臂到位等待上限为 60 秒，以兼容开启相机预览后仿真速度较慢的机器。任何
关节限位、到位、渲染或写文件错误都会终止本轮，避免在未拍摄成功时继续前进。

### 5. 巡检照片和元数据

输出目录：

```text
_ultimate_task/inspect/record/
```

每次拍摄会生成 PNG 以及同名 JSON，例如：

```text
cabinet_voltage_20260902_143015_123.png
cabinet_voltage_20260902_143015_123.json
```

三组 `left_arm_pose` 已按对应停车位和 `lefthand_camera` 外参校准。消防栓表盘
高度约 `3.1 m`，相机使用抬臂仰拍姿态。

### 6. 巡检专用定义

以下配置位于 `_ultimate_task/inspect/task_definition.py`：

- `lefthand_camera` 名称和相机外参；
- inspect 初始机械臂姿态；
- 左臂关节插值速度；
- 实际关节到位容差；
- inspect 专用 yaw 到四元数转换；
- inspect 左臂平滑插值解算。

inspect 当前初始/统一行走姿态为：

```text
[0.000000, 1.200000, 0.600000, 0.0, 0.900000, 0.000000]
```

修改这些巡检参数不会改变 VLA 的右臂初始姿态、ACT IK 或两路 VLA 相机。

## 三、常见问题

### 机器人到点后没有旋转到目标朝向

检查 `/nav_status`：机器人到达位置后应进入 `ALIGNING`，完成旋转后才会进入
`ARRIVED`。不要在 `ALIGNING` 时用零范数四元数重新发布目标，否则会清除 yaw
约束。

```bash
ros2 topic echo /nav_status
```

### VLA 机器人背对桌面

确认终端输出的 VLA 目标为 `yaw=+90.0 deg`，并确认使用了：

```bash
./slam/run_nav_saved.sh --scene ariac --task vla --view
```

本模型的可见机头在导航 `yaw=+90°` 时朝世界 `+Y`；`yaw=-90°` 会朝世界
`-Y`，从而背对桌面。

### 找不到 VLA 图像或 ACT 话题

首先确认启动时选择了 `--task vla`。inspect 配置不会创建 `/act/*` 发布器或
订阅器，这是任务隔离的预期行为。

```bash
ros2 topic list | grep '^/act'
```

### 巡检脚本提示机械臂或拍摄接口未接入

确认第一个终端使用 `--task inspect`。VLA 配置不会创建 `/inspection/*` 机械臂
和拍摄接口。

```bash
ros2 topic list | grep '^/inspection'
```

### `/nav_goal` 有订阅者但机器人仍不移动

导航器还依赖实时点云。确认 bridge 正在发布 `/pointcloud`，且启动命令没有使用
`--no-lidar`：

```bash
ros2 topic hz /pointcloud
```

### 任务参数应该修改在哪里

- VLA 桌前位置、朝向、摄像头和 ACT/IK：
  `_ultimate_task/vla/task_definition.py`；
- 螺丝刀随机范围：`model/scenes/randomize_ariac_grasp.py`；
- inspect 路线和每点左臂拍摄姿态：
  `_ultimate_task/inspect/inspection_route.json`；
- inspect 相机、初始姿态、关节插值和到位容差：
  `_ultimate_task/inspect/task_definition.py`；
- 底层通用导航：`nav_p2p.py`；
- 通用 MuJoCo/ROS bridge：`slam/bridge/bridge_core.py`。

修改某个任务时，应优先修改它自己的定义文件，不要把任务专属参数重新放回共享
bridge。
