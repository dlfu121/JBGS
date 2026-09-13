# VLA 螺丝刀 ACT 闭环任务运行说明

本文介绍如何把 KeyCollect/LeRobot ACT 接到 ARIAC 狗载右臂上，完成
“导航到桌前 → 启动 ACT 推理 → 螺丝刀脱离桌面 → 结束 ACT 推理”的闭环流程，
以及对应的原生 KeyCollect 验证方式。

> 运行前提：机器上有 ROS 2 Foxy（python3.8）与 KeyCollect conda 环境
> （python3.12，见 `../README.md` 第 4 节）。checkpoint 与数据统一使用
> `worker_scene/KeyCollect/` 下的拷贝。

## 当前结论（重要）

链路本身是通的：

1. 导航：机器人能到桌前并 `ARRIVED`；
2. ACT：`/act/task_status` 会从 `IDLE` 进入 `ACTIVE`，26 维动作持续下发，
   bridge 用阻尼最小二乘微分 IK 驱动右臂 `arm_r/*`；
3. 终止条件：bridge 检测到螺丝刀离桌后自动冻结机械臂并置 `LIFTED`，
   客户端随即停止推理。

**尚未打通的是“抓取本身”**：目前原生 KeyCollect 场景下
`outputs/train/act_rm65_dexhand/checkpoints/last` 连续 3 次失败重试都未把
螺丝刀抬起（Z 全程约 0.718 m 静止）。因此不要把这套说明当作“已验证能抓起”
的验收结果，先用于复现现象、换 checkpoint、排查版本一致性。

---

## 一、控制链与终止条件

控制逻辑：

```text
导航到桌子前面 -> 启动 ACT 推理 -> 螺丝刀脱离桌面 -> 结束 ACT 推理
```

各环节落点：

| 环节 | 承担者 | 触发/判定 |
|---|---|---|
| 导航到桌前 | `run_vla.py` 或 `run_act_e2e.py` 发布 `/nav_goal` | `/nav_status == ARRIVED` |
| 启动推理 | `act_bridge_client.py --execute` | ARRIVED 后开始请求 ACT |
| 螺丝刀离桌判定 | `slam/bridge/bridge_core.py` | 螺丝刀体 Z ≥ 静止高度 + 0.06 m，持续 0.3 s |
| 结束推理 | bridge 冻结 + client 停止请求 | `/act/task_status` 置 `LIFTED` |

`/act/task_status`（String，JSON，约 10 Hz）字段：

```text
time             仿真时间
phase            IDLE / ACTIVE / LIFTED
screwdriver_z    螺丝刀体当前世界 Z
screwdriver_rest_z  初始静止高度（bridge 启动时读取）
ee_xyz           末端 link_6 世界坐标
arm_pos          右臂 6 关节角
act_enabled      是否仍接受并执行 ACT 指令
```

## 二、任务配置

VLA 独有参数集中在：

- `task_definition.py`：桌面目标、朝向、两路相机外参、ACT/IK 限幅与初始臂态；
- 螺丝刀随机范围：`../model/scenes/randomize_ariac_grasp.py`（以及 `../vla/randomize_ariac_grasp.py` 的启动入口）；
- bridge 离桌阈值常量：`slam/bridge/bridge_core.py` 顶部
  `ACT_SCREWDRIVER_LIFT_MARGIN` / `ACT_SCREWDRIVER_SUSTAIN_SEC`。

VLA 初始右臂姿态：

```text
[3.141593, -0.401426, -1.727876, 0.0, 0.471239, 3.141593]
```

## 三、手动多窗口运行

四个终端都必须从 `worker_scene` 根目录、并先 `source /opt/ros/foxy/setup.bash`。

### 终端 1：仿真 + 导航栈

```bash
./slam/run_nav_saved.sh --scene ariac --task vla --view
```

- 会重新生成 ARIAC 组合场景并随机螺丝刀，加载 VLA 专用 XML，起
  bridge + 保存地图导航 + RViz，只开放 `/act/*` 接口。
- 固定复现：`VLA_SEED=20260909 ./slam/run_nav_saved.sh --scene ariac --task vla --view`

### 终端 2：导航到桌前（可选）

```bash
python3.8 _ultimate_task/vla/run_vla.py
```

发布带最终朝向的 `/nav_goal`，等 `ARRIVED` 后保持节点运行。
期望输出：

```text
机器人桌前目标: world=(7.225, 9.730), yaw=+90.0 deg
```

### 终端 3：ACT 推理服务（keycollect 环境）

```bash
source /home/ee304/miniforge3/etc/profile.d/conda.sh
conda activate keycollect

python _ultimate_task/vla/act_server.py \
  --checkpoint KeyCollect/outputs/train/act_rm65_dexhand/checkpoints/last \
  --dataset-root KeyCollect/data/rm65_dexhand_merged \
  --device cuda
```

看到 `ACT server ready` 即成功。`--device cpu` 用于无 CUDA 场合。

### 终端 4：ACT 客户端

```bash
python3.8 _ultimate_task/vla/act_bridge_client.py --execute
```

- 不传 `--execute` 是 dry-run（动作清零，只验通信）；
- 订阅 `/act/observation`、`/act/table_image`、`/act/wrist_image`、
  `/act/task_status`、`/nav_status`；
- `ARRIVED` 后自动开始推理；`LIFTED` 后自动停止并保持机械臂。

调试辅助参数：`--dump-dir <目录>` 会保存首帧两路图、39 维状态和动作统计。

### 观察与监控

```bash
ros2 topic echo /act/task_status       # 阶段与螺丝刀/末端 Z
ros2 topic list | grep '^/act'
ros2 topic hz /act/table_image
ros2 topic echo /nav_status
```

## 四、一键 E2E（无头，适合自动验收）

单个终端即可跑完整个闭环，默认使用独立 ROS 域（`ROS_DOMAIN_ID=71`），
不影响其它正在运行的仿真：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash

python3.8 _ultimate_task/vla/run_act_e2e.py \
  --checkpoint KeyCollect/outputs/train/act_rm65_dexhand/checkpoints/last \
  --dataset-root KeyCollect/data/rm65_dexhand_merged \
  --device cuda --seed 20260909
```

流程：

1. 生成并随机 ARIAC VLA 场景；
2. 起 bridge（EGL 无头）+ map→odom 静态 TF + 保存地图导航；
3. 起 ACT server（等待 `ACT server ready`）；
4. 起 ACT client（`--execute`）；
5. 发布桌前导航目标并等待 `ARRIVED`；
6. 等待 `/act/task_status == LIFTED`（失败/超时则清理退出）。

参数与退出码：

| 参数 | 说明 |
|---|---|
| `--seed` | 螺丝刀随机种子 |
| `--nav-timeout` / `--grasp-timeout` | 导航 / 抓取等待上限（秒） |
| `--view` | 打开 MuJoCo 查看器（否则 EGL） |
| `--device cpu|cuda` | ACT 推理设备 |
| `--trace-out FILE.csv` | 逐条记录 task_status 遥测 |

退出码 `0` = 离桌成功；`2` = 失败或超时。日志尾部会附 ACT server/client 摘要。

## 五、原生 KeyCollect 验证与失败重试

排查“是 checkpoint 问题还是跨环境迁移问题”时，先在 KeyCollect 自己的场景里
验证。以下脚本在 `worker_scene/KeyCollect` 目录、keycollect 环境下执行。

### 5.1 单次原生推理（含录制）

```bash
MUJOCO_GL=egl python scripts/infer_mujoco.py \
  --checkpoint outputs/train/act_rm65_dexhand/checkpoints/last \
  --dataset-root data/rm65_dexhand_merged \
  --device cuda --render-backend egl --headless \
  --require-gpu-rendering --random-seed 7 --save-dir /tmp/kc_infer
```

`--save-dir` 会保存 `infer.mp4`（table 相机）。

### 5.2 失败自动复位重试（推荐）

`scripts/_native_retry_grasp.py`：每次从 home 复位 + 重新随机螺丝刀，跑有限步
数；未离桌则 `policy.reset()` + `robot.reset_simulation()` 再试。

```bash
MUJOCO_GL=egl python scripts/_native_retry_grasp.py \
  --checkpoint outputs/train/act_rm65_dexhand/checkpoints/last \
  --dataset-root data/rm65_dexhand_merged \
  --device cuda \
  --max-attempts 4 --steps-per-attempt 350 \
  --report /tmp/native_retry.json
```

判定方式：直接读 `screwdriver_red` 体世界 Z（静止高度 +0.06 m、持续 0.3 s），
不依赖图像。退出码 `0` = 至少一次离桌成功；汇总见 `--report`。

> 机器繁忙时原生 ACT 控制环只能跑 ~4.6 Hz（目标 30 Hz），一次 4 连试约
> 10 分钟。建议在空闲机器或关闭其它 viewer 后运行。

## 六、常见问题

### `/act/task_status` 长时间停在 `IDLE`

- 机器人还没 `ARRIVED`，或客户端没收到两路图像；
- EGL 渲染失败时 bridge 只发状态不发图，查 bridge 日志中的
  `ACT camera render failed`。

### `ACTIVE` 但不离桌

- 目前即现状：ACT 有动作输出但未形成抓取。优先做第五节的原生重试验证；
- 若原生能抓而 worker 不能，检查 39 维状态里 `ee_pose` 是否按训练坐标系给出
  （当前 bridge 发的是 worker 世界坐标，量级 ~7.x/10.x，偏离训练分布）。

### 端口/进程冲突

- ACT server 默认监听 `127.0.0.1:5566`，同一时间只起一个；
- `run_act_e2e.py` 用 `/proc/<pid>/environ` 精确清理“本 ROS 域”的
  bridge/nav/act 进程，不会误杀其它域的仿真。

### 换 checkpoint 试

`outputs/train/` 下的候选：

```text
act_rm65_dexhand/checkpoints/020000|040000|last
act_rm65_dexhand_1/checkpoints/...
```

不同训练版本（RGB vs RGB+Depth、旧/新场景相机）不可混用，换 checkpoint 时
`--dataset-root` 必须与它对应。
