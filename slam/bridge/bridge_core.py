#!/usr/bin/env python3.8
"""把 MuJoCo 仿真桥接到 ROS 2，发布 3D 激光点云（PointCloud2）。

使用多层 3D 激光雷达（类 VLP-16）发布点云，供 RTAB-Map SLAM 和动态避障使用。
发布:
  /pointcloud  sensor_msgs/PointCloud2       64层×360束（过滤后为有效命中点）
  /pointcloud_visual sensor_msgs/PointCloud2 RViz 实时层（使用最新 TF，不供 SLAM）
  /odom        nav_msgs/Odometry             底盘里程计
  /clock       rosgraph_msgs/Clock           仿真时间
  TF  odom -> base_footprint -> base_link -> lidar3d
订阅:
  /cmd_vel     geometry_msgs/Twist           底盘速度指令

用法:
  source /opt/ros/foxy/setup.bash
  python3.8 slam_bridge_3d.py                 # headless
  python3.8 slam_bridge_3d.py --view          # 开 MuJoCo 查看器
  python3.8 slam_bridge_3d.py --patrol        # 自动巡视
"""
import argparse
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import cv2

import mujoco

try:
    from .local_avoidance import AvoidanceConfig, LocalAvoidance
    from .pedestrian_motion import PedestrianTraffic
except ImportError:
    from local_avoidance import AvoidanceConfig, LocalAvoidance
    from pedestrian_motion import PedestrianTraffic

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64MultiArray, Int64, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from model.robot.initial_pose import ARM_READY_QPOS, reset_to_ready
from _ultimate_task.inspect.task_definition import (
    ARM_ARRIVAL_TOL as INSPECTION_ARM_ARRIVAL_TOL,
    CAMERA as INSPECTION_CAMERA,
    CAMERA_EXTRINSICS as INSPECTION_CAMERA_EXTRINSICS,
    INITIAL_ARM_QPOS as INSPECTION_INITIAL_ARM_QPOS,
    next_arm_control as next_inspection_arm_control,
)
from _ultimate_task.vla.task_definition import (
    ACT_ARM_COMMAND_SPEED,
    ACT_CAMERAS,
    ACT_HAND_STEP,
    CAMERA_EXTRINSICS as VLA_CAMERA_EXTRINSICS,
    DEMO_ARM_ARRIVAL_TOL,
    DEMO_ARM_RETURN_TIMEOUT_SEC,
    DEMO_GRASP_POS,
    DEMO_GRASP_QUAT_WXYZ,
    DEMO_HOLD_SEC,
    INITIAL_ARM_QPOS as VLA_INITIAL_ARM_QPOS,
    cartesian_action_delta,
    damped_ik_joint_delta,
)

# KeyCollect's manufacturer closed-form RM65 IK, copied verbatim so the ACT
# action pipeline matches infer_mujoco.py.
try:
    from .rm65_ik import (
        RM65IKContinuitySelector,
        RM65Kinematics,
        pose_matrix,
    )
    from .rm65_kinematics import (
        CartesianPoseTarget,
        rotation_vector_to_matrix,
    )
except ImportError:
    from rm65_ik import (
        RM65IKContinuitySelector,
        RM65Kinematics,
        pose_matrix,
    )
    from rm65_kinematics import (
        CartesianPoseTarget,
        rotation_vector_to_matrix,
    )

# VLA ACT screwdriver-lift success detection (world frame).  The screwdriver
# body rests with its origin ACT_SCREWDRIVER_REST_ABOVE_TOP m above the 0.86 m
# tabletop (19 mm body radius + 1 mm clearance).  A grasp that raises the tool
# at least ACT_SCREWDRIVER_LIFT_MARGIN above its resting origin, sustained for
# ACT_SCREWDRIVER_SUSTAIN_SEC of simulated time, marks the ACT task as LIFTED
# and freezes the arm/hand.  Nudging the tool along the tabletop does not reach
# the margin; knocking it onto the floor drops it below the resting origin.
ACT_SCREWDRIVER_BODY = "screwdriver_on_assembly_table"
ACT_SCREWDRIVER_LIFT_MARGIN = 0.06
ACT_SCREWDRIVER_SUSTAIN_SEC = 0.30

# KeyCollect training-frame reference.  The checkpoint was trained with
# ``ee_pose`` in the KeyCollect world frame, where base_link sits at
# (-0.7, 0, 0.6) with identity orientation and +X points from the arm base
# towards the table.  The ARIAC right arm is mounted 180 deg about its base
# relative to that setup, so the arm-base-frame pose is mirrored across Z and
# re-anchored at this base before it is sent to the policy.  The same mirror
# turns the trained world translation (arm forward = +X) into ARIAC world
# (arm forward = +Y).
ACT_KC_ARM_BASE = np.array([-0.7, 0.0, 0.6])
ACT_KC_MIRROR = np.diag([-1.0, -1.0, 1.0])
ACT_KC_WORLD_TO_ARIAC = np.array([[0.0, -1.0, 0.0],
                                  [1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0]])
ACT_KC_JOINT1_OFFSET = math.pi
# KeyCollect's analytic RM65 IK clamps the Cartesian target to this reach
# radius (rm65/rm65_ik.py consumers use the same value).
ACT_RM65_SAFE_REACH_RADIUS_M = 0.5785

XML_3D = os.path.join(PROJECT_ROOT, "model", "robot", "warehouse_with_robot_3d.xml")
XML_FALLBACK = os.path.join(PROJECT_ROOT, "model", "robot", "robot_template_3d_py38.xml")
SCENE_NAME = os.environ.get("SLAM_SCENE_NAME", "warehouse")
TASK_PROFILE = os.environ.get(
    "ARIAC_TASK_PROFILE", "inspect" if SCENE_NAME == "ariac" else "generic")
if TASK_PROFILE not in ("generic", "inspect", "vla"):
    raise ValueError("ARIAC_TASK_PROFILE must be generic, inspect, or vla")
XML = os.environ.get("MUJOCO_SCENE_XML",
                     XML_3D if os.path.exists(XML_3D) else XML_FALLBACK)

INSPECTION_RECORD_DIR = os.path.join(
    PROJECT_ROOT, "_ultimate_task", "inspect", "record")
INSPECTION_CAMERA_WIDTH = 640
INSPECTION_CAMERA_HEIGHT = 480
ACT_CONTROL_HZ = 30.0
# Wrist-camera preview rate.  Twelve FPS is responsive on the normal EGL/GLFW
# setups while still leaving enough time for the 50 Hz simulation timer.  It
# can be overridden for slower/faster machines with INSPECTION_PREVIEW_HZ.
try:
    INSPECTION_PREVIEW_HZ = min(
        30.0, max(1.0, float(os.environ.get("INSPECTION_PREVIEW_HZ", "12"))))
except (TypeError, ValueError):
    INSPECTION_PREVIEW_HZ = 12.0
# Limit the moving target itself, not just the actuator response.  Keep the
# original inspection motion speed requested by the route operator.

# 3D 雷达参数。
#
# 注意：射线不再走 XML 里的 2880 个 <rangefinder> 传感器，而是每帧用
# mj_multiRay 批量投射（见 _scan）。原因有两个：
#   1) 性能：2880 个 rangefinder 占了 89% 的 mj_step 时间（0.18x 实时），
#      同样 2880 条射线用 mj_multiRay 只要 1.3 ms（10 Hz 下 1.3% CPU）。
#      这让我们能负担得起下面大得多的视场。
#   2) 视场：原来 ±15° 的垂直视场对地面机器人根本不够——雷达在 0.95 m 高，
#      最低层 -15° 要到 3.67 m 外才碰到地面，0.22 m 高的箱子在 2.72 m 内
#      完全隐形。而避障阈值是 0.75/1.50 m，全部落在盲区里，所以狗看不见
#      自己即将撞上的矮障碍，近处地面也永远建不出来。
#      -60° 把地面环拉到 0.55 m，矮箱子 0.42 m 就能看见。
VERTICAL_ANGLES_DEG = np.concatenate([
    # Coarse steep-down coverage for the floor and very low obstacles.
    np.linspace(-60.0, -20.0, 12, endpoint=False),
    # Dense near-horizontal coverage is what makes horizontal tabletops visible.
    np.linspace(-20.0, 5.0, 42, endpoint=False),
    np.linspace(5.0, 20.0, 10),
])
NUM_LAYERS = len(VERTICAL_ANGLES_DEG)
NUM_H_RAYS = 360
NUM_TOTAL_RAYS = NUM_LAYERS * NUM_H_RAYS  # 23040
RANGE_MIN = 0.12
RANGE_MAX = 8.0
V_MIN = math.radians(float(VERTICAL_ANGLES_DEG[0]))
V_MAX = math.radians(float(VERTICAL_ANGLES_DEG[-1]))
SCAN_HZ = 10.0
ODOM_HZ = 50.0
# 动态行人的轨迹使用 map/odom 坐标（原点在 dog_home），而 mocap 的坐标在
# tick() 中再转换到 MuJoCo world 坐标。ARIAC 场景中的行人会有意穿过巡检
# 路线，用于检验动态避障。ARIAC 行人不会因机器狗接近而停车或让路；
# 机器狗自身的实时点云与导航必须主动绕开行人。
PERSON_ROBOT_HARD_CLEARANCE = 0.95
# Keep pedestrians slow enough for the robot's lidar/DWA loop to observe a
# crossing for several frames.  The previous 3.5x multiplier made the first
# crossing pass in a fraction of a second at simulation speed and was easy to
# miss between two point-cloud callbacks.
PERSON_SPEED_SCALE = 1.0

# 点云高度裁剪（世界系，米）。
# 关键：必须保留地板点。rtabmap 用 Grid/MaxGroundHeight 把点分成"地面"和
# "障碍"两层，地面点是它填充空闲栅格（RViz 里的白色区域）的依据。
# 旧代码把 z<0.05 的点全滤掉，等于抽掉了地面层，rtabmap 只能靠 ray tracing
# 猜空闲区，于是地面就随着狗移动而闪烁/消失。
Z_MIN_WORLD = -0.05          # 略低于地板，容纳测距噪声
Z_MAX_WORLD = 2.60           # 高于墙顶(2.8m 的墙取其下部即可)，滤掉天花板

# 噪声参数。默认使用仿真底盘的无噪声里程计，避免 70 m 路径上的随机游走
# 在回环时被 ICP 一次性拉回。需要验证回环抗漂移能力时显式传 --odom-noise。
ODOM_NOISE_DEFAULT = False
ODOM_TRANS_NOISE = 0.01
ODOM_ROT_NOISE = 0.006
SCAN_NOISE_STD = 0.012

# 避障参数
OBSTACLE_DIST = 0.75       # 雷达到障碍的水平净距阈值（米）- 紧急避障
OBSTACLE_SLOW_DIST = 1.50  # 开始减速的距离
OBSTACLE_FRONT_AZ = 42     # 前方扇区半角（单位：度，与射线数无关）

# 巡视航点。除外圈外，增加三条中央扫描线，补齐仅沿墙巡视时看不到的货架区。
#
# 这条航线是用 clearance 检查过的：每一段的最小间隙 >= 0.99 m（狗半径约
# 0.35 m）。旧航线有三段直接穿过实体：
#   (4,4)->(8.5,4)   撞 shipping_container_conveyor_ariac（x 6.13..7.23,
#                    y -3.82..4.32 的 8 米长传送带，整条挡死）
#   (8.5,0)->(8.5,-4) 撞 warehouse_dumpster (x 7.08..9.08, y -3.10..-1.66)
#   (4,8.5)->(4,4)    终点距 warehouse_cone_0 只有 0.32 m，小于紧急避障阈值
# 传送带东侧到东墙之间是唯一通道，dumpster 又占了 x<9.08，所以南北向必须
# 走 x=10.2 这条窄廊（间隙 1.12 m）。
WAREHOUSE_PATROL_WAYPOINTS = [
    # 西侧和北侧外圈
    (-8.5, -8.5),   # 起点：西南角
    (-8.5, -4.0),
    (-8.5, 0.0),
    (-8.5, 4.0),
    (-8.5, 8.5),    # 西北角
    (-4.0, 8.5),
    (0.0, 8.5),
    (4.0, 8.5),

    # 北部扫描线：位于两排横向货架之间
    (5.5, 5.5),     # 传送带北端(y=4.32)外侧
    (2.2, 5.0),
    (-3.5, 5.0),
    (-7.5, 5.0),

    # 绕货架西端进入中央中部扫描线
    (-8.2, 4.0),
    (-8.2, 1.8),
    (-7.5, 1.8),
    (-3.5, 1.8),
    (1.0, 1.8),

    # 经开阔竖向通道进入中央南部扫描线
    (1.0, -3.0),
    (-3.5, -3.0),
    (-7.5, -3.0),

    # 南侧外圈：西 -> 东
    (-8.5, -4.0),
    (-8.5, -8.5),
    (-4.0, -8.5),
    (0.0, -8.5),
    (4.0, -8.5),
    (8.5, -8.5),

    # 东侧窄廊：南 -> 北（夹在传送带/dumpster 与东墙之间）
    (10.2, -4.5),
    (10.2, 0.0),
    (10.2, 5.5),
]
# 巡视航点（ARIAC 场景，2026-08 新场景：实验室实际只有北墙 y=20.9 和东墙
# x=23.5(y 0.4~17)，西/南开放）。
#
# 这条航线用 MuJoCo 场景几何做了逐段 clearance 校验。东侧货架和东墙之间
# 可连续通过的扫描线位于 x=19.9；沿 y=-5..17 精细采样的最小水平净空约
# 0.95m，高于 0.75m 紧急避障阈值。贴近东墙的 x=22.x 会被 shelf_5/6
# 挤窄，不能作为机器人巡航线。
# 注意：3-4 号货架列之间(y 2.9~3.3 处有箱子，走廊被挤到 1.72m)避障穿不过，
# 所以本航线完全绕开那条走廊。
ARIAC_PATROL_WAYPOINTS = [
    (4.0, 4.6),      # 0  起点（初始位姿）
    (0.0, 4.6),      # 1  西行，绕开检查区簇(x0.9~6, y1~3.5)
    (0.0, -4.5),     # 2  沿西侧下行
    (-3.0, -5.0),    # 3  西南角
    (20.0, -5.0),    # 4  东侧连廊南入口
    (19.9, -2.0),    # 5  进入东侧连廊中心线
    (19.9, 4.0),     # 6  扫描 shelf_5 南段
    (19.9, 8.0),     # 7  扫描两组东侧货架之间
    (19.9, 12.0),    # 8  扫描 shelf_6 南段
    (19.9, 17.0),    # 9  东侧连廊北端
    # 原路退出可保证所有连接段仍在同一条已校验的可通行带中。
    (19.9, 12.0),    # 10
    (19.9, 8.0),     # 11
    (19.9, 4.0),     # 12
    (19.9, -2.0),    # 13
    (20.0, -5.0),    # 14 回到南入口
    (12.0, -5.0),    # 15 沿南侧西返
    # 从南侧进入 shelf_1/shelf_3 通道，再从 y=8 横向通道退出。按当前
    # MuJoCo 激光几何每 0.1m 检查，沿线最小可见障碍距离约 0.91m。
    (16.2, -1.8),    # 16 shelf_1 通道南侧入口
    (16.2, 8.0),     # 17 沿 shelf_1 东侧扫描并从北侧退出
    (12.0, 8.0),     # 18 中部扫描线 y=8 东端
    (-2.0, 8.0),     # 19 y=8 西行
    (4.5, 8.0),      # 20 回东到 x=4.5（装配区以西）
    (4.5, 15.7),     # 21 上行到 y=15.7 走廊
    (12.0, 15.7),    # 22 y=15.7 东行
    (12.0, 17.5),    # 23 上行到北侧通道东段
    (21.5, 17.5),    # 24 北通道东端，斜射扫东墙上部
    (15.0, 17.5),    # 25 北墙扫描 东→西
    (12.0, 17.5),    # 26 回退到 x=12
    (12.0, 15.7),    # 27 下到 y=15.7，绕开 module_shelves(x 7~9.75)
    (4.5, 15.7),     # 28 y=15.7 西行
    (4.5, 17.0),     # 29 上到北通道西段
    (-2.0, 17.0),    # 30 北墙西段扫描
    (4.5, 17.0),     # 31 东返回
    (4.5, 15.7),     # 32 下回 y=15.7
    (10.5, 15.7),    # 33 东移到装配区以东
    (10.5, 4.6),     # 34 下行回中部
    (4.0, 4.6),      # 35 回起点，闭环
]
PATROL_WAYPOINTS = (ARIAC_PATROL_WAYPOINTS if SCENE_NAME == "ariac"
                    else WAREHOUSE_PATROL_WAYPOINTS)
PATROL_V = (0.70 if SCENE_NAME == "ariac" else 0.50)
# ARIAC 在 10Hz 扫描下每帧约移动 7cm；warehouse 保持原来的 5cm。
PATROL_W = 0.6              # 降低角速度使转弯更平稳
PATROL_TOL = 0.50           # 增大容差，避免反复调整
PATROL_LOOP = False
PATROL_MAX_LAPS = 1            # 巡视 1 圈后自动停止

# 航点进展检测。只计算到目标的距离是否下降；后退和左右摆动不再被误判为
# "有进展"。多次主动脱困仍无进展时才跳过不可达航点。
STUCK_WINDOW = 9.0
PROGRESS_DIST = 0.30
MAX_RECOVERY_ATTEMPTS = 3


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def apply_task_camera_extrinsics(model, camera_extrinsics):
    """Apply the selected task's camera calibration to the loaded model."""
    for name, extrinsic in camera_extrinsics.items():
        camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera_id < 0:
            raise RuntimeError("%s task requires camera %s" %
                               (TASK_PROFILE, name))
        model.cam_pos[camera_id] = np.asarray(extrinsic["pos"], dtype=float)
        quat = np.asarray(extrinsic["quat_wxyz"], dtype=float)
        norm = float(np.linalg.norm(quat))
        if quat.shape != (4,) or norm < 1e-12:
            raise ValueError("invalid camera quaternion for %s" % name)
        model.cam_quat[camera_id] = quat / norm
        if "fovy" in extrinsic:
            model.cam_fovy[camera_id] = float(extrinsic["fovy"])


class SlamBridge3D(Node):
    def __init__(self, view=False, patrol=False, seed=0, no_lidar=False,
                 max_laps=PATROL_MAX_LAPS, odom_noise=ODOM_NOISE_DEFAULT,
                 dynamic_person=False, sim_speed=1.0,
                 inspection_fps=INSPECTION_PREVIEW_HZ):
        super().__init__("mujoco_slam_bridge_3d")
        self.no_lidar = no_lidar
        self.odom_noise = bool(odom_noise)
        self.model = mujoco.MjModel.from_xml_path(XML)
        camera_extrinsics = {
            "inspect": INSPECTION_CAMERA_EXTRINSICS,
            "vla": VLA_CAMERA_EXTRINSICS,
        }.get(TASK_PROFILE, {})
        apply_task_camera_extrinsics(self.model, camera_extrinsics)
        self.data = mujoco.MjData(self.model)
        self.task_profile = TASK_PROFILE
        initial_arm_pose = {
            "inspect": INSPECTION_INITIAL_ARM_QPOS,
            "vla": VLA_INITIAL_ARM_QPOS,
        }.get(self.task_profile, ARM_READY_QPOS)
        reset_to_ready(
            mujoco, self.model, self.data, pose=initial_arm_pose)
        self.rng = np.random.RandomState(seed)
        # Pedestrians can be enabled at startup (--dynamic-person) or toggled
        # at runtime by the inspection runner.  Keep the mocap bodies in the
        # model even while hidden so a late toggle does not require a restart.
        self.dynamic_person = bool(dynamic_person)

        m = self.model
        self.dt = m.opt.timestep
        self.sim_speed = float(sim_speed)
        self.inspection_fps = min(30.0, max(1.0, float(inspection_fps)))
        self._steps_per_tick = max(
            1, int(round(self.sim_speed / (ODOM_HZ * self.dt))))

        # XML 里那 2880 个 <rangefinder> 不再使用（我们自己投射射线），
        # 禁用传感器计算，mj_step 快 10.6 倍（0.13x -> 1.34x 实时）。
        m.opt.disableflags = (int(m.opt.disableflags)
                              | int(mujoco.mjtDisableBit.mjDSBL_SENSOR))

        # 预计算每根射线的方向向量（雷达局部系）
        self._build_direction_table()

        # mj_multiRay 的输出缓冲 + 排除狗自身的 body
        self._ray_geomid = np.zeros(NUM_TOTAL_RAYS, dtype=np.int32)
        self._ray_dist = np.zeros(NUM_TOTAL_RAYS, dtype=np.float64)
        self._dog_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        # mj_multiRay's excludebody is not consistent across articulated
        # descendants in the MuJoCo versions used by this project.  Keep an
        # explicit geom set so links, hands and the lidar mast cannot self-hit.
        self._robot_bodies = self._descendant_bodies(self._dog_body)
        self._robot_geom_ids = np.asarray([
            geom for geom in range(m.ngeom)
            if int(m.geom_bodyid[geom]) in self._robot_bodies
        ], dtype=np.int32)
        self._ranges = np.full((NUM_LAYERS, NUM_H_RAYS), -1.0)

        # --- 执行器/关节索引 ---
        self.act = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + n)
                    for n in ("base_x", "base_y", "base_yaw")}
        self.qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}
        self.vadr = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}
        self._left_arm_actuators = np.asarray([
            mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                "act_arm_l/joint_%d" % index)
            for index in range(1, 7)
        ], dtype=np.int32)
        self._left_arm_joints = np.asarray([
            mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT,
                "arm_l/joint_%d" % index)
            for index in range(1, 7)
        ], dtype=np.int32)
        if np.any(self._left_arm_actuators < 0) or np.any(
                self._left_arm_joints < 0):
            raise RuntimeError("inspection requires all six left-arm joints")
        self._left_arm_target = np.asarray([
            self.data.ctrl[index] for index in self._left_arm_actuators
        ], dtype=float)
        self._left_arm_qpos = np.asarray([
            m.jnt_qposadr[index] for index in self._left_arm_joints
        ], dtype=np.int32)
        self._inspection_arm_status_pending = False
        self._inspection_arm_stable_ticks = 0
        # ACT bridge (explicitly opt-in with --act-bridge).  The bridge uses
        # ROS messages so the Python 3.12 policy process never imports rclpy.
        self._act_enabled = False
        self._act_target = None
        self._act_last_t = -1.0
        self._act_arm_actuators = np.asarray([
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              "act_arm_r/joint_%d" % i)
            for i in range(1, 7)], dtype=np.int32)
        self._act_arm_qpos = np.asarray([
            m.jnt_qposadr[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT, "arm_r/joint_%d" % i)]
            for i in range(1, 7)], dtype=np.int32)
        self._act_arm_joints = np.asarray([
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                              "arm_r/joint_%d" % i)
            for i in range(1, 7)], dtype=np.int32)
        self._act_arm_dofs = np.asarray([
            m.jnt_dofadr[j] for j in self._act_arm_joints], dtype=np.int32)
        self._act_hand_actuators = np.asarray([
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              "act_hand_r/r_f_joint%d_%d" % (finger, joint))
            for finger in range(1, 6) for joint in range(1, 5)], dtype=np.int32)
        self._act_hand_qpos = np.asarray([
            m.jnt_qposadr[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT,
                "hand_r/r_f_joint%d_%d" % (finger, joint))]
            for finger in range(1, 6) for joint in range(1, 5)], dtype=np.int32)
        self._act_hand_joints = np.asarray([
            mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT,
                "hand_r/r_f_joint%d_%d" % (finger, joint))
            for finger in range(1, 6) for joint in range(1, 5)], dtype=np.int32)
        if np.any(self._act_arm_actuators < 0) or np.any(self._act_arm_qpos < 0):
            raise RuntimeError("ACT bridge requires right arm joints/actuators")
        self._act_target = np.r_[self.data.ctrl[self._act_arm_actuators],
                                 self.data.ctrl[self._act_hand_actuators]]
        self._act_ee_body = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, "arm_r/link_6")
        if self._act_ee_body < 0:
            raise RuntimeError("ACT bridge requires arm_r/link_6")
        self._act_arm_base_body = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, "arm_r/base_link")
        if self._act_arm_base_body < 0:
            raise RuntimeError("ACT bridge requires arm_r/base_link")
        # Closed-form RM65 IK matching KeyCollect's infer_mujoco.py action
        # pipeline: a persistent Cartesian target is integrated from the ACT
        # deltas and solved with the manufacturer's analytic IK (continuity
        # selection + reach clamp) instead of a differential-IK step.
        self._act_ik_robot = RM65Kinematics("RM65-6F")
        self._act_ik_selector = RM65IKContinuitySelector(
            self._act_ik_robot, sample_period=1.0 / ACT_CONTROL_HZ)
        self._act_cartesian_target = None
        # Screwdriver lift detection.  The tool body is only present in the
        # ARIAC/VLA scenes; other profiles disable the success gate.
        self._act_screwdriver_body = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, ACT_SCREWDRIVER_BODY)
        if TASK_PROFILE == "vla" and self._act_screwdriver_body < 0:
            raise RuntimeError(
                "vla task requires %s body in the scene" % ACT_SCREWDRIVER_BODY)
        if self._act_screwdriver_body >= 0:
            self._act_screwdriver_rest_z = float(
                self.data.xpos[self._act_screwdriver_body][2])
        else:
            self._act_screwdriver_rest_z = None
        self._act_phase = "IDLE"
        self._act_done = False
        self._act_lift_start = None
        self._act_status_pub = None
        # Scripted teleport-grasp demo.  It stays idle until run_vla.py
        # publishes /act/demo_grasp once navigation reports ARRIVED.
        self._demo_active = False
        self._demo_state = "IDLE"
        self._demo_state_t0 = None
        self._demo_initial_arm = np.asarray(VLA_INITIAL_ARM_QPOS, dtype=float)
        self._demo_hand_base_body = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, "hand_r/right_hand_base")
        self._demo_screwdriver_geoms = (
            np.asarray([geom for geom in range(m.ngeom)
                        if int(m.geom_bodyid[geom]) == self._act_screwdriver_body],
                       dtype=np.int32)
            if self._act_screwdriver_body >= 0
            else np.asarray([], dtype=np.int32))
        person_names = (("dynamic_person", "dynamic_person_2")
                        if SCENE_NAME == "ariac" else
                        ("dynamic_person", "dynamic_person_2",
                         "dynamic_person_3"))
        self._person_mocaps = []
        for person_name in person_names:
            person_body = mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_BODY, person_name)
            mocap_id = (m.body_mocapid[person_body]
                        if person_body >= 0 else -1)
            self._person_mocaps.append(int(mocap_id))
        self._person_control_available = all(
            index >= 0 for index in self._person_mocaps)
        if self.dynamic_person and not self._person_control_available:
            raise RuntimeError(
                "dynamic pedestrians require the scene's mocap bodies")
        self._person_tracks = (self._build_person_tracks()
                               if self._person_control_available else [])
        self._person_last_log = -float("inf")
        self._person_traffic = None
        self._person_motion_time = 0.0

        # 狗初始世界位置
        dogb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self.dog_home = self.data.xpos[dogb][:2].copy()

        # 3D 雷达相对 base_link 的静态外参
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        self._lidar_site = sid
        lp_world = self.data.site_xpos[sid].copy()
        yaw0 = self._base_yaw_world()
        Rb = np.array([[math.cos(yaw0), -math.sin(yaw0)],
                       [math.sin(yaw0), math.cos(yaw0)]])
        rel = Rb.T @ (lp_world[:2] - self.dog_home)
        self.lidar_xyz = (float(rel[0]), float(rel[1]), float(lp_world[2]))

        # --- ROS 接口 ---
        pc_qos = QoSProfile(depth=5,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_pc = self.create_publisher(PointCloud2, "pointcloud", pc_qos)
        visual_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_pc_visual = self.create_publisher(
            PointCloud2, "pointcloud_visual", visual_qos)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self._inspection_arm_status_pub = None
        self._inspection_done_pub = None
        self._act_state_pub = None
        self._act_table_pub = None
        self._act_wrist_pub = None
        if self.task_profile == "inspect":
            self.create_subscription(
                Float64MultiArray, "/inspection/left_arm_pose",
                self._on_inspection_arm_pose, 10)
            self.create_subscription(
                String, "/inspection/camera_preview",
                self._on_inspection_preview, 10)
            self._inspection_arm_status_pub = self.create_publisher(
                String, "/inspection/left_arm_status", 10)
            self.create_subscription(
                String, "/inspection/capture", self._on_inspection_capture, 10)
            person_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                Bool, "/inspection/dynamic_person", self._on_dynamic_person,
                person_qos)
            self.create_subscription(
                Int64, "/inspection/dynamic_person_seed",
                self._on_dynamic_person_seed, person_qos)
            self._inspection_done_pub = self.create_publisher(
                String, "/inspection/capture_done", 10)
        elif self.task_profile == "vla":
            self.create_subscription(
                Float64MultiArray, "/act/joint_action",
                self._on_act_joint_action, 10)
            self.create_subscription(
                Float64MultiArray, "/act/cartesian_action",
                self._on_act_cartesian_action, 10)
            self._act_state_pub = self.create_publisher(
                String, "/act/observation", 2)
            self._act_table_pub = self.create_publisher(
                Image, "/act/table_image", 2)
            self._act_wrist_pub = self.create_publisher(
                Image, "/act/wrist_image", 2)
            self._act_status_pub = self.create_publisher(
                String, "/act/task_status", 2)
            self.create_subscription(
                String, "/act/demo_grasp", self._on_demo_grasp, 10)
        else:
            self._act_status_pub = None
        completion_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/mapping_complete",
                                 self._on_mapping_complete, completion_qos)

        self._publish_static_tf()
        # Foxy/Fast DDS can fail to replay a transient-local TF sample when
        # RTAB-Map starts several seconds after this bridge.  Re-publishing the
        # unchanged extrinsics keeps late joiners from dropping every scan.
        self.create_timer(1.0, self._publish_static_tf)

        self.cmd = (0.0, 0.0, 0.0)
        self.patrol = patrol
        self._mapping_complete = False
        self._map_saved = False
        self.max_laps = max_laps
        self._last_cmd_t = -10.0   # 看门狗：最近一次收到外部 /cmd_vel 的仿真时间
        self.wp = 1
        self.lap = 0
        self._avoider = LocalAvoidance(AvoidanceConfig(
            range_min=RANGE_MIN,
            range_max=RANGE_MAX,
            emergency_dist=OBSTACLE_DIST,
            slow_dist=OBSTACLE_SLOW_DIST,
            front_half_angle_deg=OBSTACLE_FRONT_AZ,
            max_turn_rate=PATROL_W,
        ))
        self._progress_wp = self.wp
        self._progress_best = float("inf")
        self._progress_t = self.data.time
        self._recovery_attempts = 0
        self.odom_xy = np.zeros(2)
        self.odom_yaw = 0.0
        self._last_truth = self._truth_pose()

        self.viewer = None
        if view:
            import importlib
            mj_viewer = importlib.import_module("mujoco.viewer")
            self.viewer = mj_viewer.launch_passive(self.model, self.data)
            # 只隐藏黄色射线；mj_multiRay 计算和 /pointcloud 发布保持启用。
            try:
                self.viewer.opt.flags[
                    mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
            except Exception:
                pass

        self._inspection_renderer = None
        self._inspection_frame = None
        self._inspection_camera = INSPECTION_CAMERA
        self._inspection_preview_active = False
        self._inspection_window_open = False
        self._inspection_render_every = max(
            1, int(round(ODOM_HZ / self.inspection_fps)))
        self._inspection_scene_option = mujoco.MjvOption()
        self._inspection_scene_option.flags[
            mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self._act_renderer = None
        self._act_renderer_failed = False

        self._scan_every = max(1, int(round(ODOM_HZ / SCAN_HZ)))
        self._n = 0

        # 先扫一帧，避免第一个 tick 的避障读到全 -1 的空矩阵而误判为"无障碍"
        self._scan()

        # 预发 clock 让其他 use_sim_time 节点先同步仿真时间，避免 TF_OLD_DATA
        for _ in range(10):
            ck = Clock()
            ck.clock.sec, ck.clock.nanosec = self._stamp()
            self.pub_clock.publish(ck)

        self.create_timer(1.0 / ODOM_HZ, self.tick)
        self.get_logger().info(
            "3D bridge up: lidar3d at base_link (%.3f, %.3f, %.3f), "
            "%d layers x %d rays = %d total, patrol=%s, odom_noise=%s, "
            "sim_speed=%.2fx, task_profile=%s, inspection_fps=%.1f"
            % (self.lidar_xyz + (NUM_LAYERS, NUM_H_RAYS, NUM_TOTAL_RAYS,
                                 patrol, self.odom_noise, self.sim_speed,
                                 self.task_profile, self.inspection_fps)))

    def _act_delta_to_world(self, task_delta):
        """Map KeyCollect-world policy deltas into the ARIAC world frame."""
        task_delta = np.asarray(task_delta, dtype=float).copy()
        ee_rot = self.data.xmat[self._act_ee_body].reshape(3, 3)
        # KeyCollect's infer_mujoco.py integrates translation deltas in its
        # world frame, where the arm's forward is +X.  In ARIAC the same
        # operational direction is world +Y, so rotate the translation into the
        # ARIAC world before solving IK.
        task_delta[0:3] = ACT_KC_WORLD_TO_ARIAC.dot(task_delta[0:3])
        # Rotation deltas are applied in the end-effector local frame, which is
        # independent of the base mounting; only convert local -> world.
        task_delta[3:6] = ee_rot.dot(task_delta[3:6])
        return task_delta

    def _act_ee_pose_in_arm_base(self):
        """Return the EE pose in the KeyCollect training world frame.

        The arm-base-frame pose is mirrored across Z (180 deg about the base)
        and re-anchored at KeyCollect's base_link so the 39-D state stays inside
        the distribution the ACT checkpoint was trained on.
        """
        base_pos = self.data.xpos[self._act_arm_base_body]
        base_rot = self.data.xmat[self._act_arm_base_body].reshape(3, 3)
        ee_pos_world = self.data.xpos[self._act_ee_body]
        ee_rot_world = self.data.xmat[self._act_ee_body].reshape(3, 3)
        ee_pos = ACT_KC_MIRROR.dot(base_rot.T.dot(ee_pos_world - base_pos))
        ee_pos = ee_pos + ACT_KC_ARM_BASE
        ee_rot = ACT_KC_MIRROR.dot(base_rot.T.dot(ee_rot_world))
        ee_quat = np.empty(4, dtype=float)
        mujoco.mju_mat2Quat(ee_quat, ee_rot.reshape(9))
        return ee_pos, ee_quat

    def _on_act_joint_action(self, msg):
        """Receive 26 direct joint targets for the opt-in smoke test.

        This topic is deliberately named ``joint_action``: it is not the
        KeyCollect Cartesian ACT output.  A future policy adapter must convert
        ACT's first six Cartesian deltas through IK before publishing here.
        """
        if self._act_done:
            return
        values = np.asarray(msg.data, dtype=float)
        if values.shape != (26,) or not np.all(np.isfinite(values)):
            self.get_logger().error("/act/joint_action requires 26 finite values")
            return
        arm_joint_ids = np.asarray([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                              "arm_r/joint_%d" % i) for i in range(1, 7)])
        arm = np.clip(values[:6], self.model.jnt_range[arm_joint_ids, 0],
                      self.model.jnt_range[arm_joint_ids, 1])
        self._act_target[:6] = arm
        if self._act_hand_actuators.size:
            self._act_target[6:] = values[6:]
        self._act_enabled = True
        if self._act_phase == "IDLE":
            self._act_phase = "ACTIVE"
        self._act_last_t = self.data.time

    def _on_act_cartesian_action(self, msg):
        """Convert KeyCollect's 6D Cartesian + 20D hand delta to targets.

        Mirrors infer_mujoco.py: integrate a persistent world-frame Cartesian
        target from the deltas, clamp it to the RM65 reach radius, then solve
        the manufacturer's closed-form IK expressed in the arm base frame.
        """
        if self._act_done:
            return
        action = np.asarray(msg.data, dtype=float)
        if action.shape != (26,) or not np.all(np.isfinite(action)):
            self.get_logger().error(
                "/act/cartesian_action requires 26 finite values")
            return
        delta = cartesian_action_delta(action)
        # Translation is defined in the KeyCollect world; map it to ARIAC
        # world.  Rotation is applied in the end-effector local frame.
        translation_world = ACT_KC_WORLD_TO_ARIAC.dot(delta[0:3])
        rotation_local = delta[3:6]

        ee_pos = self.data.xpos[self._act_ee_body].copy()
        ee_rot = self.data.xmat[self._act_ee_body].reshape(3, 3).copy()
        if self._act_cartesian_target is None:
            self._act_cartesian_target = CartesianPoseTarget.from_pose(
                ee_pos, ee_rot)
        self._act_cartesian_target.integrate(translation_world, rotation_local)

        base_pos = self.data.xpos[self._act_arm_base_body]
        base_rot = self.data.xmat[self._act_arm_base_body].reshape(3, 3)
        offset = self._act_cartesian_target.position_world - base_pos
        distance = float(np.linalg.norm(offset))
        if distance > ACT_RM65_SAFE_REACH_RADIUS_M:
            self._act_cartesian_target.position_world[:] = base_pos + offset * (
                ACT_RM65_SAFE_REACH_RADIUS_M / distance)

        target_world = pose_matrix(
            self._act_cartesian_target.position_world,
            self._act_cartesian_target.rotation_world)
        world_from_base = np.eye(4)
        world_from_base[:3, :3] = base_rot
        world_from_base[:3, 3] = base_pos
        target_base = np.linalg.inv(world_from_base).dot(target_world)
        try:
            solution = self._act_ik_selector.solve(
                target_base, initial_seed=self.data.qpos[self._act_arm_qpos])
        except Exception as exc:
            self.get_logger().warning(
                "RM65 analytic IK failed, holding arm target: %s" % exc)
            return
        self._act_target[:6] = np.clip(
            solution.joints,
            self.model.jnt_range[self._act_arm_joints, 0],
            self.model.jnt_range[self._act_arm_joints, 1])
        current_hand = self._act_target[6:]
        hand = current_hand + np.clip(
            action[6:], -ACT_HAND_STEP, ACT_HAND_STEP)
        if self._act_hand_actuators.size:
            hand_joint_ids = np.asarray([
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT,
                    "hand_r/r_f_joint%d_%d" % (finger, joint))
                for finger in range(1, 6) for joint in range(1, 5)])
            hand = np.clip(hand, self.model.jnt_range[hand_joint_ids, 0],
                           self.model.jnt_range[hand_joint_ids, 1])
        self._act_target[6:] = hand
        self._act_enabled = True
        if self._act_phase == "IDLE":
            self._act_phase = "ACTIVE"
        self._act_last_t = self.data.time

    # ------------------------------------------------------------------
    # Scripted teleport-grasp demo (no ACT)
    # ------------------------------------------------------------------
    def _on_demo_grasp(self, msg):
        """Start the scripted fist/teleport/return demo on /act/demo_grasp."""
        if self.task_profile != "vla":
            return
        command = str(msg.data).strip().lower()
        if command not in ("start", "grasp", "demo"):
            self.get_logger().warning("未知 /act/demo_grasp 指令: %r" % msg.data)
            return
        if self._demo_active:
            self.get_logger().debug("演示已在进行中，忽略重复触发")
            return
        if self._act_screwdriver_body < 0 or self._demo_hand_base_body < 0:
            self.get_logger().error("场景缺少螺丝刀或右手 body，无法执行演示")
            return
        self._demo_active = True
        self._demo_state = "HOLD"
        # Wall-clock holds: "5 s" must mean five real seconds even when the
        # headless renderer keeps the simulation below real time.
        self._demo_state_t0 = time.monotonic()
        # Take over the right arm: block further ACT callbacks and the lift
        # detector, but keep the shared arm/hand controller applying targets.
        self._act_done = True
        self._act_enabled = True
        if self._act_target is None:
            self._act_target = np.r_[
                self.data.ctrl[self._act_arm_actuators],
                self.data.ctrl[self._act_hand_actuators]]
        self._act_phase = "DEMO"
        self._demo_disable_screwdriver_collision()
        self.get_logger().info(
            "演示开始：保持右臂 %.1f s，随后握拳并把螺丝刀吸附到拳心"
            % DEMO_HOLD_SEC)

    def _demo_disable_screwdriver_collision(self):
        """Stop the static screwdriver geoms from pushing the closing hand."""
        for geom in self._demo_screwdriver_geoms:
            self.model.geom_contype[geom] = 0
            self.model.geom_conaffinity[geom] = 0

    def _demo_fist_target(self):
        """Return the 20 hand joint upper limits (fully closed fist)."""
        return self.model.jnt_range[self._act_hand_joints, 1].astype(float)

    def _update_act_demo(self):
        """Advance the demo state machine once per bridge tick."""
        if not self._demo_active:
            return
        now = time.monotonic()
        if self._demo_state == "HOLD":
            if now - self._demo_state_t0 >= DEMO_HOLD_SEC:
                self._act_target[6:] = self._demo_fist_target()
                self._demo_state = "ATTACH"
                self._demo_state_t0 = now
                self._act_phase = "DEMO_ATTACHED"
                self.get_logger().info("右机械臂握拳，螺丝刀瞬移吸附到拳心")
        elif self._demo_state == "ATTACH":
            if now - self._demo_state_t0 >= DEMO_HOLD_SEC:
                self._act_target[:6] = self._demo_initial_arm
                self._demo_state = "RETURN"
                self._demo_state_t0 = now
                self.get_logger().info("吸附保持完成，右机械臂开始回到初始位置")
        elif self._demo_state == "RETURN":
            current = self.data.qpos[self._act_arm_qpos]
            settled = float(np.max(np.abs(current - self._demo_initial_arm)))
            if (settled <= DEMO_ARM_ARRIVAL_TOL
                    or now - self._demo_state_t0 >= DEMO_ARM_RETURN_TIMEOUT_SEC):
                self._demo_state = "DONE"
                self._demo_state_t0 = now
                self._act_phase = "DEMO_DONE"
                self.get_logger().info(
                    "右机械臂已回到初始位置（关节误差 %.3f rad），演示结束"
                    % settled)

    def _demo_follow_screwdriver(self):
        """Pin the screwdriver to the palm while the demo holds it."""
        if (not self._demo_active
                or self._demo_state not in ("ATTACH", "RETURN", "DONE")):
            return
        hand_pos = self.data.xpos[self._demo_hand_base_body]
        hand_rot = self.data.xmat[self._demo_hand_base_body].reshape(3, 3)
        world_pos = hand_pos + hand_rot.dot(
            np.asarray(DEMO_GRASP_POS, dtype=float))
        grasp_mat = np.zeros(9, dtype=float)
        mujoco.mju_quat2Mat(
            grasp_mat, np.asarray(DEMO_GRASP_QUAT_WXYZ, dtype=float))
        world_rot = hand_rot.dot(grasp_mat.reshape(3, 3))
        body = self._act_screwdriver_body
        if self.model.body_jntnum[body] > 0:
            # A future scene may give the tool a free joint; drive its qpos.
            joint = self.model.body_jntadr[body]
            adr = self.model.jnt_qposadr[joint]
            self.data.qpos[adr:adr + 3] = world_pos
            quat = np.empty(4, dtype=float)
            mujoco.mju_mat2Quat(quat, world_rot.reshape(9))
            self.data.qpos[adr + 3:adr + 7] = quat
        else:
            parent = self.model.body_parentid[body]
            parent_pos = self.data.xpos[parent]
            parent_rot = self.data.xmat[parent].reshape(3, 3)
            self.model.body_pos[body] = parent_rot.T.dot(world_pos - parent_pos)
            local_rot = parent_rot.T.dot(world_rot)
            local_quat = np.empty(4, dtype=float)
            mujoco.mju_mat2Quat(local_quat, local_rot.reshape(9))
            self.model.body_quat[body] = local_quat
        mujoco.mj_forward(self.model, self.data)

    def _sample_screwdriver_z(self):
        """Return the current screwdriver body origin z (world, metres)."""
        if self._act_screwdriver_body is None or self._act_screwdriver_body < 0:
            return None
        return float(self.data.xpos[self._act_screwdriver_body][2])

    def _update_act_lift_detection(self):
        """Freeze the arm and mark the task LIFTED once the tool stays raised."""
        if self._act_done or self._act_screwdriver_body is None:
            return
        if self._act_screwdriver_body < 0 or not self._act_enabled:
            self._act_lift_start = None
            return
        z = self.data.xpos[self._act_screwdriver_body][2]
        if z - self._act_screwdriver_rest_z >= ACT_SCREWDRIVER_LIFT_MARGIN:
            if self._act_lift_start is None:
                self._act_lift_start = self.data.time
                self.get_logger().info(
                    "螺丝刀开始离桌 (z=%.3f m, 静止参考 %.3f m)..."
                    % (z, self._act_screwdriver_rest_z))
            elif (self.data.time - self._act_lift_start
                    >= ACT_SCREWDRIVER_SUSTAIN_SEC):
                self._act_done = True
                self._act_enabled = False
                self._act_phase = "LIFTED"
                self.get_logger().info(
                    "ACT 任务成功：螺丝刀已脱离桌面并保持 %.2f s (z=%.3f m)，机械臂已冻结"
                    % (ACT_SCREWDRIVER_SUSTAIN_SEC, z))
        else:
            self._act_lift_start = None

    def _publish_act_task_status(self):
        """Publish ACT phase plus screwdriver/EE z for the task runner."""
        if self._act_status_pub is None:
            return
        msg = String()
        ee_xyz = self.data.xpos[self._act_ee_body].astype(float).tolist()
        arm_pos = self.data.qpos[self._act_arm_qpos].astype(float).tolist()
        msg.data = json.dumps({
            "time": float(self.data.time),
            "phase": self._act_phase,
            "screwdriver_z": self._sample_screwdriver_z(),
            "screwdriver_rest_z": self._act_screwdriver_rest_z,
            "ee_xyz": ee_xyz,
            "arm_pos": arm_pos,
            "ee_z": ee_xyz[2],
            "act_enabled": bool(self._act_enabled),
        }, separators=(",", ":"))
        self._act_status_pub.publish(msg)

    def _update_act_arm(self):
        if not self._act_enabled or self._act_target is None:
            return
        max_change = ACT_ARM_COMMAND_SPEED * self.dt * self._steps_per_tick
        current = self.data.ctrl[self._act_arm_actuators]
        self.data.ctrl[self._act_arm_actuators] = current + np.clip(
            self._act_target[:6] - current, -max_change, max_change)
        if self._act_hand_actuators.size:
            self.data.ctrl[self._act_hand_actuators] = self._act_target[6:]

    def _publish_act_observation(self):
        """Publish a lightweight state-only observation for bridge bring-up."""
        arm_pos = self.data.qpos[self._act_arm_qpos].astype(float).tolist()
        arm_vel = self.data.qvel[[self.model.jnt_dofadr[j]
                                  for j in self._act_arm_joints]].astype(float).tolist()
        hand_pos = self.data.qpos[self._act_hand_qpos].astype(float).tolist()
        # joint_1 differs by pi between the two arm mountings; report the
        # KeyCollect-equivalent angle so the policy stays in distribution.
        arm_pos[0] -= ACT_KC_JOINT1_OFFSET
        ee_pos, ee_quat = self._act_ee_pose_in_arm_base()
        # MuJoCo gives wxyz; KeyCollect state uses xyzw.
        ee_pose = ee_pos.tolist() + [float(ee_quat[1]), float(ee_quat[2]),
                                     float(ee_quat[3]), float(ee_quat[0])]
        msg = String()
        msg.data = json.dumps({"time": float(self.data.time),
                               "arm_pos": arm_pos, "arm_vel": arm_vel,
                               "hand_pos": hand_pos, "ee_pose": ee_pose},
                              separators=(",", ":"))
        self._act_state_pub.publish(msg)

        # Rendering is optional for bridge bring-up.  If the host has no
        # usable EGL/GL context, disable it after the first failure instead of
        # flooding the ROS log on every simulation tick; state-only ACT
        # messages can still be consumed for diagnostics.
        if self._act_renderer_failed:
            return
        try:
            if self._act_renderer is None:
                self._act_renderer = mujoco.Renderer(
                    self.model, height=480, width=640)
            for camera, publisher in zip(
                    ACT_CAMERAS, (self._act_table_pub, self._act_wrist_pub)):
                self._act_renderer.update_scene(self.data, camera=camera,
                                                scene_option=self._inspection_scene_option)
                frame = np.ascontiguousarray(self._act_renderer.render())
                image = Image()
                image.header.stamp.sec, image.header.stamp.nanosec = self._stamp()
                image.height, image.width = frame.shape[:2]
                image.encoding = "rgb8"
                image.is_bigendian = False
                image.step = int(frame.shape[1] * 3)
                image.data = frame.tobytes()
                publisher.publish(image)
        except Exception as exc:
            self._act_renderer_failed = True
            self.get_logger().error("ACT camera render failed: %s" % exc)

    def _build_person_tracks(self):
        """Create deterministic, bounded pedestrian paths.

        The ARIAC paths create one opposing encounter before tank and, in the
        tank->hydrant long aisle, a walker that enters the dog's lane from the
        side and then stops in front of it as a stationary obstacle. They contain
        fixed bends, run at different speeds, and are traversed
        forwards/backwards forever.  The warehouse scene has no
        cabinet/tank/hydrant route, so it gets separate outer-aisle paths
        instead of accidentally using ARIAC coordinates.  No random values are
        used; ``--seed`` cannot change these paths.
        """
        if SCENE_NAME == "ariac":
            # Separate encounter areas, all in the open aisle. Each person
            # starts once the dog approaches, then runs on its own clock.
            # No position follows the dog, even when navigation pauses.
            lanes = (
                {
                    "points": ((-5.05, -5.8), (-5.05, -4.8),
                               (-3.8, -4.8), (-3.8, -6.8)),
                    "speed_mps": 0.22, "offset": 0.0,
                    "curve_amp": 0.06, "curve_sign": 1.0,
                },
                {
                    # Red pedestrian steps from the aisle side into the dog's
                    # own lane and then STOPS in front of the dog.  The
                    # tank->hydrant planner lane is the straight segment
                    # stop2(-5.65,-8.1) -> stop3(9.0,-10.0):
                    # y(x)=-8.1-(1.9/14.65)*(x+5.65), so at x=1.0 it sits at
                    # y~=-8.96.  The walker waits below the lane (spawn, out of
                    # the DWA swept band), steps north onto the lane, and
                    # ``stop_index`` holds it there so it becomes a stationary
                    # frontal obstacle the dog must actively avoid.  The
                    # remaining points are never reached once the hold starts.
                    "points": ((1.0, -9.71), (1.0, -8.96),
                               (2.4, -9.14), (3.8, -9.33),
                               (5.2, -9.51), (6.6, -9.69)),
                    "speed_mps": 0.42, "offset": 0.0,
                    "curve_amp": 0.04, "curve_sign": 1.0,
                    "stop_index": 1,
                },
            )
        else:
            lanes = (
                # Warehouse-only outer aisles; no inspection-route clearance
                # or ARIAC stop coordinates are applied in this scene.
                {
                    "points": ((-10.6, -12.5), (-10.6, -5.0),
                               (-9.6, -2.5), (-10.6, 2.0), (-10.6, 7.4)),
                    "speed_mps": 0.45, "offset": 0.0,
                    "curve_amp": 0.28, "curve_sign": 1.0,
                },
                {
                    "points": ((12.0, -12.5), (11.4, -8.0),
                               (12.0, -3.5), (11.4, 1.0), (12.0, 7.4)),
                    "speed_mps": 0.68, "offset": 7.0,
                    "curve_amp": 0.24, "curve_sign": -1.0,
                },
                {
                    "points": ((3.0, 0.0), (4.0, 2.0), (3.0, 4.0),
                               (4.0, 6.0), (3.0, 8.0)),
                    "speed_mps": 0.92, "offset": 13.0,
                    "curve_amp": 0.20, "curve_sign": 1.0,
                },
            )
        tracks = []
        for lane in lanes:
            points = np.asarray(lane["points"], dtype=np.float64)
            speed = float(lane["speed_mps"]) * PERSON_SPEED_SCALE
            if speed <= 0.0 or len(points) < 2:
                raise ValueError("dynamic pedestrian lane has invalid speed")
            # Segment durations are derived from each lane's fixed speed, so
            # the pedestrians remain visibly different without random timing.
            # The smoothstep interpolation still produces gentle
            # acceleration/deceleration at every turn.
            forward_durations = (np.linalg.norm(np.diff(points, axis=0), axis=1)
                                 / speed)
            # ``stop_index`` turns a lane into a one-shot walker: it advances
            # to that waypoint and then holds position forever instead of
            # looping.  This is used to park a pedestrian in the dog's lane.
            stop_index = lane.get("stop_index")
            stop_time = None
            if stop_index is not None:
                stop_index = int(stop_index)
                if not 0 <= stop_index < len(points):
                    raise ValueError("dynamic pedestrian stop_index out of range")
                stop_time = float(np.sum(forward_durations[:stop_index]))
            # Append the interior points and the first point in reverse order.
            # This creates a closed out-and-back route with an explicit final
            # segment back to the starting point (no period-end teleport).
            points = np.vstack((points, points[-2::-1]))
            durations = np.concatenate((forward_durations,
                                        forward_durations[::-1]))
            track = {
                "points": points,
                "durations": durations,
                "period": float(np.sum(durations)),
                "speed_mps": speed,
                "offset": float(lane["offset"]),
                "curve_amp": float(lane["curve_amp"]),
                "curve_sign": float(lane["curve_sign"]),
            }
            if stop_time is not None:
                track["stop_time"] = stop_time
            tracks.append(track)
        return tracks

    @staticmethod
    def _smoothstep(value):
        value = max(0.0, min(1.0, float(value)))
        return value * value * (3.0 - 2.0 * value)

    def _person_position(self, track, now):
        elapsed = float(now) + track["offset"]
        stop_time = track.get("stop_time")
        if stop_time is None:
            elapsed = elapsed % track["period"]
        elif elapsed >= stop_time:
            # One-shot walker: hold the stop waypoint instead of looping.
            elapsed = stop_time
        for index, duration in enumerate(track["durations"]):
            if elapsed <= duration:
                u = self._smoothstep(elapsed / max(float(duration), 1e-6))
                start, end = track["points"][index:index + 2]
                position = start + u * (end - start)
                # Smooth lateral bow.  It is zero at both waypoints, so the
                # loop remains bounded and does not jump at segment changes.
                tangent = end - start
                length = float(np.linalg.norm(tangent))
                if length > 1e-6:
                    normal = np.asarray((-tangent[1], tangent[0])) / length
                    position = position + (
                        track.get("curve_sign", 1.0)
                        * track.get("curve_amp", 0.0)
                        * math.sin(math.pi * u) * normal)
                return position
            elapsed -= float(duration)
        return track["points"][-1]

    def _update_people(self, robot_map):
        now = float(self.data.time)
        if self._person_traffic is None:
            starts = [self._person_position(track, 0.0)
                      for track in self._person_tracks]
            # Enabling at an occupied spawn waits for the dog to clear it;
            # do not materialise a person inside the robot or project/teleport
            # one into a wall. This also handles restarting a demo mid-route.
            if any(np.linalg.norm(start - robot_map) <
                   PERSON_ROBOT_HARD_CLEARANCE for start in starts):
                self._hide_people()
                return
            # In the ARIAC inspection the pedestrian owns its trajectory and
            # never yields to the dog.  A zero robot gap disables only that
            # yield rule; lidar/DWA remains responsible for collision
            # avoidance. Warehouse traffic retains the conservative behavior.
            robot_gap = (0.0 if SCENE_NAME == "ariac" else
                         PERSON_ROBOT_HARD_CLEARANCE)
            self._person_traffic = PedestrianTraffic(
                starts, robot_gap=robot_gap)
            self._person_motion_time = now
        positions = self._person_traffic.step(
            now - self._person_motion_time,
            lambda index, elapsed: self._person_position(
                self._person_tracks[index], elapsed),
            robot=robot_map,
            trigger_distance=3.2 if SCENE_NAME == "ariac" else None)
        self._person_motion_time = now
        for mocap_id, position in zip(self._person_mocaps, positions):
            self.data.mocap_pos[mocap_id] = (
                float(self.dog_home[0] + position[0]),
                float(self.dog_home[1] + position[1]), 0.0)

    def _hide_people(self):
        if not self._person_control_available:
            return
        for mocap_id in self._person_mocaps:
            self.data.mocap_pos[mocap_id] = (-20.0, -20.0, 0.0)

    def _on_dynamic_person(self, msg):
        if not self._person_control_available:
            self.get_logger().warning(
                "当前 MuJoCo 模型没有 dynamic_person mocap，忽略动态行人开关")
            return
        enabled = bool(msg.data)
        if enabled == self.dynamic_person:
            return
        self.dynamic_person = enabled
        self._person_traffic = None
        if not enabled:
            self._hide_people()
        self.get_logger().info("动态行人 %s" % ("已开启" if enabled else "已关闭"))

    def _on_dynamic_person_seed(self, msg):
        if not self._person_control_available:
            return
        # Keep the topic for backwards compatibility with older runners, but
        # intentionally do not use it: pedestrian trajectories are fixed.
        seed = int(msg.data)
        self.get_logger().info("动态行人采用固定折线路径（忽略随机种子 %d）" % seed)

    def _build_direction_table(self):
        """预计算每根射线在雷达局部系中的单位方向向量。"""
        dirs = np.zeros((NUM_LAYERS, NUM_H_RAYS, 3), dtype=np.float32)
        for layer in range(NUM_LAYERS):
            phi = math.radians(float(VERTICAL_ANGLES_DEG[layer]))
            cp, sp = math.cos(phi), math.sin(phi)
            for az in range(NUM_H_RAYS):
                theta = -math.pi + 2 * math.pi * az / NUM_H_RAYS
                dirs[layer, az] = [cp * math.cos(theta),
                                   cp * math.sin(theta), sp]
        self._dirs = dirs                      # (NUM_LAYERS, NUM_H_RAYS, 3)
        self._dirs_flat = dirs.reshape(-1, 3).astype(np.float64)

    def _descendant_bodies(self, root):
        """Return root and every articulated body below it."""
        result = set()
        for body in range(self.model.nbody):
            current = body
            while current > 0 and current != root:
                current = int(self.model.body_parentid[current])
            if current == root:
                result.add(body)
        return result

    def _scan(self):
        """用 mj_multiRay 投射全部射线，返回 (NUM_LAYERS, NUM_H_RAYS) 距离矩阵。

        未命中或超出 RANGE_MAX 的射线置 -1（与原 rangefinder 的约定一致）。
        方向表是雷达局部系，需按当前 base_yaw 旋到世界系再投射。
        """
        d = self.data
        origin = d.site_xpos[self._lidar_site].copy()
        yaw = self._base_yaw_world()
        c, s = math.cos(yaw), math.sin(yaw)
        # 只有绕 z 的旋转，直接手写比构造 3x3 再乘更省
        vx = c * self._dirs_flat[:, 0] - s * self._dirs_flat[:, 1]
        vy = s * self._dirs_flat[:, 0] + c * self._dirs_flat[:, 1]
        world = np.empty((NUM_TOTAL_RAYS, 3))
        world[:, 0] = vx
        world[:, 1] = vy
        world[:, 2] = self._dirs_flat[:, 2]

        mujoco.mj_multiRay(
            self.model, d, origin, world.flatten(),
            None,               # geomgroup: 全部组
            1,                  # flg_static: 包含静态几何体（墙、地板、货架）
            self._dog_body,     # 排除狗自身，避免自打击
            self._ray_geomid, self._ray_dist,
            NUM_TOTAL_RAYS, RANGE_MAX)

        self_hit = np.isin(self._ray_geomid, self._robot_geom_ids)
        self._ray_geomid[self_hit] = -1
        self._ray_dist[self_hit] = -1.0

        # mj_multiRay 的 cutoff 只用于加速剪枝，返回值仍可能 > cutoff，
        # 而且未命中时返回 -1。两种情况都要判成无效。
        r = self._ray_dist.reshape(NUM_LAYERS, NUM_H_RAYS)
        np.copyto(self._ranges, r)
        self._ranges[(r < 0) | (r > RANGE_MAX)] = -1.0
        return self._ranges

    # ---------------- 位姿 ----------------
    def _base_yaw_world(self):
        return float(self.data.qpos[self.qadr["base_yaw"]])

    def _truth_pose(self):
        return np.array([
            self.dog_home[0] + float(self.data.qpos[self.qadr["base_x"]]),
            self.dog_home[1] + float(self.data.qpos[self.qadr["base_y"]]),
            self._base_yaw_world()])

    def _integrate_odom(self):
        cur = self._truth_pose()
        d = cur - self._last_truth
        self._last_truth = cur
        dyaw = math.atan2(math.sin(d[2]), math.cos(d[2]))
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        yaw_prev = cur[2] - dyaw
        cb, sb = math.cos(yaw_prev), math.sin(yaw_prev)
        dbody = np.array([cb * d[0] + sb * d[1], -sb * d[0] + cb * d[1]])
        if self.odom_noise:
            dist = float(np.linalg.norm(dbody))
            dbody += self.rng.randn(2) * ODOM_TRANS_NOISE * math.sqrt(max(dist, 1e-9))
            dyaw += self.rng.randn() * ODOM_ROT_NOISE * math.sqrt(abs(dyaw) + 1e-9)
        self.odom_xy += np.array([c * dbody[0] - s * dbody[1],
                                  s * dbody[0] + c * dbody[1]])
        self.odom_yaw = math.atan2(math.sin(self.odom_yaw + dyaw),
                                   math.cos(self.odom_yaw + dyaw))

    # ---------------- ROS 输出 ----------------
    def _stamp(self):
        t = self.data.time
        msg_sec = int(t)
        msg_nsec = int((t - msg_sec) * 1e9)
        return msg_sec, msg_nsec

    def _publish_static_tf(self):
        out = []
        s, ns = 0, 0
        t1 = TransformStamped()
        t1.header.stamp.sec, t1.header.stamp.nanosec = s, ns
        t1.header.frame_id = "base_footprint"
        t1.child_frame_id = "base_link"
        t1.transform.rotation.w = 1.0
        out.append(t1)

        t2 = TransformStamped()
        t2.header.stamp.sec, t2.header.stamp.nanosec = s, ns
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "lidar3d"
        t2.transform.translation.x = self.lidar_xyz[0]
        t2.transform.translation.y = self.lidar_xyz[1]
        t2.transform.translation.z = self.lidar_xyz[2]
        t2.transform.rotation.w = 1.0
        out.append(t2)
        self.tf_static.sendTransform(out)

    def publish_odom(self):
        s, ns = self._stamp()
        vx = float(self.data.qvel[self.vadr["base_x"]])
        vy = float(self.data.qvel[self.vadr["base_y"]])
        wz = float(self.data.qvel[self.vadr["base_yaw"]])
        yaw = self._base_yaw_world()
        c, sn = math.cos(yaw), math.sin(yaw)
        vxb = c * vx + sn * vy
        vyb = -sn * vx + c * vy

        od = Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = s, ns
        od.header.frame_id = "odom"
        od.child_frame_id = "base_footprint"
        od.pose.pose.position.x = float(self.odom_xy[0])
        od.pose.pose.position.y = float(self.odom_xy[1])
        od.pose.pose.orientation = yaw_to_quat(self.odom_yaw)
        od.twist.twist.linear.x = vxb
        od.twist.twist.linear.y = vyb
        od.twist.twist.angular.z = wz
        for i, v in ((0, 0.02), (7, 0.02), (35, 0.05)):
            od.pose.covariance[i] = v
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp.sec, tf.header.stamp.nanosec = s, ns
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = float(self.odom_xy[0])
        tf.transform.translation.y = float(self.odom_xy[1])
        tf.transform.rotation = yaw_to_quat(self.odom_yaw)
        self.tf.sendTransform(tf)

    def publish_pointcloud(self):
        """把 self._ranges（由 tick 刷新）打包成 PointCloud2 发布。"""
        s, ns = self._stamp()
        ranges = self._ranges

        # 无效射线为 -1；必须在加噪声前判定，否则噪声会把 -1 抬进有效区间。
        valid = (ranges > RANGE_MIN) & (ranges < RANGE_MAX)

        # 加噪声（只对有效点）
        if SCAN_NOISE_STD > 0:
            noise = self.rng.randn(NUM_LAYERS, NUM_H_RAYS) * SCAN_NOISE_STD
            ranges = ranges + noise * valid.astype(float)

        # 距离 × 方向 = XYZ（雷达局部系）
        pts = ranges[:, :, np.newaxis] * self._dirs
        pts_valid = pts[valid]

        # 高度裁剪：把世界系阈值换算到雷达系（雷达装在 z=lidar_z 处，无俯仰）。
        # 保留地板点——rtabmap 需要它们来标记空闲栅格，见 Z_MIN_WORLD 注释。
        lidar_z = self.lidar_xyz[2]
        z = pts_valid[:, 2]
        pts_valid = pts_valid[(z > Z_MIN_WORLD - lidar_z)
                              & (z < Z_MAX_WORLD - lidar_z)]

        # 构建 PointCloud2 消息
        msg = PointCloud2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = s, ns
        msg.header.frame_id = "lidar3d"
        msg.height = 1
        msg.width = len(pts_valid)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # 打包二进制数据
        if msg.width > 0:
            buf = np.zeros((msg.width, 4), dtype=np.float32)
            buf[:, :3] = pts_valid.astype(np.float32)
            buf[:, 3] = 1.0  # intensity 固定为 1.0
            msg.data = buf.tobytes()
        else:
            msg.data = b''

        # SLAM/避障必须保留精确采样时间；RViz 实时层则只需显示最新一帧。
        # RTAB-Map 约 1 Hz 更新 map->odom，地图变大时一次更新可能超过
        # RViz 固定的 10 帧 TF 等待队列，导致旧点云以 reason=Unknown 刷屏。
        # 单独发布零时间戳的显示副本，让 RViz 明确使用最新完整 TF；原始
        # /pointcloud 的时间戳和建图语义完全不变。
        self.pub_pc.publish(msg)
        visual = PointCloud2()
        visual.header.frame_id = msg.header.frame_id
        visual.height = msg.height
        visual.width = msg.width
        visual.fields = msg.fields
        visual.is_bigendian = msg.is_bigendian
        visual.point_step = msg.point_step
        visual.row_step = msg.row_step
        visual.data = msg.data
        visual.is_dense = msg.is_dense
        self.pub_pc_visual.publish(visual)

    # ---------------- 控制 ----------------
    def _on_inspection_arm_pose(self, msg):
        if len(msg.data) != 6:
            self.get_logger().error("左臂目标必须包含 6 个关节值")
            return
        target = np.asarray(msg.data, dtype=float)
        limits = self.model.jnt_range[self._left_arm_joints]
        if not np.all(np.isfinite(target)):
            self.get_logger().error("左臂目标包含非有限值")
            return
        if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
            self.get_logger().error("左臂目标超出 RM65 关节限位")
            return
        self._left_arm_target = target
        self._inspection_arm_status_pending = True
        self._inspection_arm_stable_ticks = 0
        self._inspection_preview_active = True
        self.get_logger().info(
            "开始平滑移动左臂并打开 %s 预览" % self._inspection_camera)

    def _on_inspection_preview(self, msg):
        command = msg.data.strip().lower()
        if command == "open":
            self._inspection_preview_active = True
            # Render on the next simulation tick.  Initializing/rendering here
            # would block the executor before the queued arm target is handled.
        elif command == "close":
            self._inspection_preview_active = False
            self.close_inspection_camera()
        else:
            self.get_logger().warning(
                "未知相机预览命令 %r（应为 open/close）" % msg.data)

    def _ensure_inspection_renderer(self):
        if self._inspection_renderer is not None:
            return True
        try:
            self._inspection_renderer = mujoco.Renderer(
                self.model, height=INSPECTION_CAMERA_HEIGHT,
                width=INSPECTION_CAMERA_WIDTH)
            if os.environ.get("DISPLAY"):
                cv2.namedWindow(
                    "Inspection Camera - Left Hand", cv2.WINDOW_AUTOSIZE)
                self._inspection_window_open = True
            return True
        except Exception as exc:
            self.get_logger().error("巡检相机初始化失败: %s" % exc)
            return False

    def _render_inspection_camera(self):
        if not self._ensure_inspection_renderer():
            return None
        # The preview is an inspection aid; hide the wrist/hand meshes so they
        # cannot cover the gauge while the arm is travelling into position.
        hidden = self._inspection_geom_visibility(False)
        try:
            self._inspection_renderer.update_scene(
                self.data, camera=self._inspection_camera,
                scene_option=self._inspection_scene_option)
            self._inspection_frame = self._inspection_renderer.render().copy()
        finally:
            for gid, alpha in hidden:
                self.model.geom_rgba[gid, 3] = alpha
        if self._inspection_window_open:
            cv2.imshow(
                "Inspection Camera - Left Hand",
                cv2.cvtColor(self._inspection_frame, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        return self._inspection_frame

    def _inspection_geom_visibility(self, visible):
        """Hide the arm/hand visual meshes only while making a camera frame."""
        names = ("arm_l/", "hand_l/")
        changed = []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if name.startswith(names):
                if visible:
                    self.model.geom_rgba[gid, 3] = 1.0
                else:
                    changed.append((gid, float(self.model.geom_rgba[gid, 3])))
                    self.model.geom_rgba[gid, 3] = 0.0
        return changed

    def _render_target_capture(self):
        """Render the fixed-direction wrist camera without arm occlusion."""
        hidden = self._inspection_geom_visibility(False)
        try:
            return self._render_inspection_camera()
        finally:
            for gid, alpha in hidden:
                self.model.geom_rgba[gid, 3] = alpha

    def _publish_capture_result(self, stop_id, success, path="", error=""):
        result = String()
        result.data = json.dumps({
            "stop_id": stop_id,
            "success": bool(success),
            "path": path,
            "error": error,
        }, ensure_ascii=False, separators=(",", ":"))
        self._inspection_done_pub.publish(result)

    def _on_inspection_capture(self, msg):
        try:
            request = json.loads(msg.data)
            stop_id = str(request["stop_id"])
            camera = str(request.get("camera", INSPECTION_CAMERA))
            if mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
                raise ValueError("camera does not exist: %s" % camera)
            self._inspection_camera = camera
            frame = self._render_target_capture()
            if frame is None:
                raise RuntimeError("camera renderer is unavailable")

            os.makedirs(INSPECTION_RECORD_DIR, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            millis = int((time.time() % 1.0) * 1000)
            basename = "%s_%s_%03d" % (stop_id, stamp, millis)
            image_path = os.path.join(
                INSPECTION_RECORD_DIR, basename + ".png")
            metadata_path = os.path.join(
                INSPECTION_RECORD_DIR, basename + ".json")
            written = cv2.imwrite(
                image_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            if not written:
                raise IOError("cv2.imwrite failed")
            metadata = dict(request)
            metadata.update({
                "image": os.path.basename(image_path),
                "captured_at": "%s.%03d" % (stamp, millis),
                "simulation_time": float(self.data.time),
            })
            with open(metadata_path, "w", encoding="utf-8") as stream:
                json.dump(metadata, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            self._publish_capture_result(stop_id, True, image_path)
            self.get_logger().info("巡检图像已保存: %s" % image_path)
        except Exception as exc:
            stop_id = locals().get("stop_id", "unknown")
            self._publish_capture_result(stop_id, False, error=str(exc))
            self.get_logger().error("巡检拍摄失败: %s" % exc)

    def _update_inspection_arm(self):
        current = self.data.ctrl[self._left_arm_actuators]
        elapsed_sec = self.dt * self._steps_per_tick
        self.data.ctrl[self._left_arm_actuators] = next_inspection_arm_control(
            current, self._left_arm_target, elapsed_sec)

    def _publish_arm_arrival_if_ready(self):
        if not self._inspection_arm_status_pending:
            return
        actual = self.data.qpos[self._left_arm_qpos]
        error = np.abs(actual - self._left_arm_target)
        max_error = float(np.max(error))
        if max_error > INSPECTION_ARM_ARRIVAL_TOL:
            self._inspection_arm_stable_ticks = 0
            return
        # Require several consecutive simulation ticks inside the tolerance;
        # a single crossing of the threshold is not considered settled.
        self._inspection_arm_stable_ticks += 1
        if self._inspection_arm_stable_ticks < 5:
            return
        status = String()
        status.data = json.dumps({
            "state": "arrived",
            "target": self._left_arm_target.tolist(),
            "actual": actual.tolist(),
            "max_error": max_error,
        }, separators=(",", ":"))
        self._inspection_arm_status_pub.publish(status)
        self._inspection_arm_status_pending = False
        self.get_logger().info(
            "左臂实际关节已到位，最大误差 %.4f rad" % max_error)

    def close_inspection_camera(self):
        if self._inspection_renderer is not None:
            self._inspection_renderer.close()
            self._inspection_renderer = None
        if self._inspection_window_open:
            cv2.destroyWindow("Inspection Camera - Left Hand")
            cv2.waitKey(1)
            self._inspection_window_open = False

    def on_cmd_vel(self, msg):
        if self._mapping_complete:
            self.cmd = (0.0, 0.0, 0.0)
            return
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.patrol = False
        self._last_cmd_t = self.data.time

    def _on_mapping_complete(self, msg):
        if msg.data:
            self._finish_mapping("Frontier 探索完成")

    def _finish_mapping(self, reason):
        """Stop accepting scans, then save one stable final map."""
        if self._mapping_complete:
            return
        self._mapping_complete = True
        self.patrol = False
        self.cmd = (0.0, 0.0, 0.0)
        self.get_logger().info(
            "%s：已停车并停止实时点云，等待 RTAB-Map 排空后保存" % reason)
        self._auto_save_map()

    def _auto_save_map(self):
        """巡视完成后自动保存地图（异步执行，不阻塞仿真）。"""
        if self._map_saved:
            return
        self._map_saved = True

        def _save():
            # RTAB-Map 以 1 Hz 消费点云；先让最后一帧及回调队列处理完。
            time.sleep(1.5)
            script = os.path.join(PROJECT_ROOT, "slam", "save_map_3d.sh")
            if os.path.isfile(script):
                result = subprocess.run(
                    ["bash", script, "--scene", SCENE_NAME, "--finalize"],
                    cwd=PROJECT_ROOT)
                if result.returncode == 0:
                    self.get_logger().info(
                        "地图已冻结并保存到 maps/%s/ 目录" % SCENE_NAME)
                else:
                    self.get_logger().error(
                        "地图自动保存失败（退出码 %d），请检查上方日志"
                        % result.returncode)
            else:
                self.get_logger().warn("找不到 save_map_3d.sh，请手动保存")

        threading.Thread(target=_save, daemon=True).start()

    def _front_obstacle_dist(self):
        """Return horizontal front clearance and obstacle-side indication.

        The old implementation returned 3D slant range. For a low box that
        value is dominated by the 0.95 m lidar height and can still be large
        when the robot is almost touching the box.
        """
        scan = self._avoider.analyze(
            self._ranges, self._dirs, self.lidar_xyz[2])
        # Positive means the right route has less clearance (obstacle on right).
        side = scan.left_score - scan.right_score
        return scan.path_front, side

    def _reset_progress(self, dist=float("inf")):
        self._progress_wp = self.wp
        self._progress_best = dist
        self._progress_t = self.data.time
        self._recovery_attempts = 0

    def _patrol_cmd(self):
        if self.wp >= len(PATROL_WAYPOINTS):
            if not PATROL_LOOP:
                self._finish_mapping("固定巡视完成")
                return (0.0, 0.0, 0.0)
            self.wp = 1
            self.lap += 1
            self.get_logger().info("lap %d done, looping" % self.lap)
            if self.max_laps > 0 and self.lap >= self.max_laps:
                self.patrol = False
                self.cmd = (0.0, 0.0, 0.0)
                self._finish_mapping("巡视 %d 圈完成" % self.max_laps)
                return (0.0, 0.0, 0.0)
        p = self._truth_pose()
        tx, ty = PATROL_WAYPOINTS[self.wp]
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        if dist < PATROL_TOL:
            self.wp += 1
            self._avoider.reset()
            self._reset_progress()
            self.get_logger().info("waypoint %d/%d reached"
                                   % (self.wp, len(PATROL_WAYPOINTS)))
            return (0.0, 0.0, 0.0)
        want = math.atan2(dy, dx)
        err = math.atan2(math.sin(want - p[2]), math.cos(want - p[2]))
        wz = max(-PATROL_W, min(PATROL_W, 1.8 * err))

        now = self.data.time
        scan = self._avoider.analyze(
            self._ranges, self._dirs, self.lidar_xyz[2])

        # Progress means getting closer to this waypoint. Lateral oscillation
        # and backing up no longer reset the stuck timer.
        if self._progress_wp != self.wp:
            self._reset_progress(dist)
        elif dist < self._progress_best - PROGRESS_DIST:
            self._progress_best = dist
            self._progress_t = now
            self._recovery_attempts = 0

        force_recovery = False
        if (not self._avoider.active
                and now - self._progress_t >= STUCK_WINDOW):
            if self._recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                self.get_logger().warn(
                    "航点 %d(%.1f,%.1f) 连续 %d 次脱困后仍无进展，安全跳过"
                    % (self.wp, tx, ty, self._recovery_attempts))
                self.wp += 1
                self._avoider.reset()
                self._reset_progress()
                return (0.0, 0.0, 0.0)
            self._recovery_attempts += 1
            self._progress_t = now
            force_recovery = True
            self.get_logger().warn(
                "到航点距离 %.2fm 已有 %.0fs 未下降，启动脱困 %d/%d"
                % (dist, STUCK_WINDOW, self._recovery_attempts,
                   MAX_RECOVERY_ATTEMPTS))

        preferred = 1.0 if err >= 0.0 else -1.0
        previous_phase = self._avoider.phase
        recovery = self._avoider.recovery_command(
            now, scan, preferred=preferred, force=force_recovery)
        if self._avoider.phase != previous_phase:
            phase_text = {
                LocalAvoidance.IDLE: "恢复航点导航",
                LocalAvoidance.BACKUP: "后退拉开距离",
                LocalAvoidance.TURN: "锁定方向转向",
                LocalAvoidance.CLEAR: "沿新方向越过障碍",
            }[self._avoider.phase]
            self.get_logger().info(
                "[避障状态] %s，通行走廊 %.2fm，监测扇形 %.2fm，绕行方向=%s"
                % (phase_text, scan.path_front, scan.front,
                   "左" if self._avoider.turn_dir > 0.0 else "右"))
        if recovery is not None:
            return recovery

        caution = self._avoider.caution_command(
            now, scan, goal_error=err, cruise_speed=PATROL_V)
        if caution is not None:
            return caution

        # === 正常导航：无障碍物，按目标点导航 ===
        if abs(err) > 0.35:
            return (0.0, 0.0, wz)
        return (PATROL_V * max(0.25, 1.0 - abs(err)), 0.0, wz)

    def tick(self):
        if self.dynamic_person:
            # Convert the current MuJoCo world pose back to map/odom
            # coordinates.  This is deliberately sampled every bridge tick,
            # rather than relying on the last odometry message, so a person
            # reacts even when the robot is braking or recovering.
            robot_world = self.data.xpos[self._dog_body][:2]
            robot_map = np.asarray(robot_world, dtype=np.float64) - self.dog_home
            self._update_people(robot_map)
            if self.data.time - self._person_last_log >= 5.0:
                self._person_last_log = self.data.time
                positions = [self.data.mocap_pos[index][:2]
                             for index in self._person_mocaps]
                self.get_logger().info(
                    "动态行人位置: %s" % ", ".join(
                        "(%.1f,%.1f)" % (p[0], p[1]) for p in positions))
        else:
            self._hide_people()

        if self._mapping_complete:
            self.cmd = (0.0, 0.0, 0.0)
        elif self.patrol:
            self.cmd = self._patrol_cmd()
        elif self.data.time - self._last_cmd_t > 0.5:
            # 看门狗：超过 0.5s 没有新速度指令就停车，避免狗带着最后一条指令跑飞
            self.cmd = (0.0, 0.0, 0.0)
        vxb, vyb, wz = self.cmd
        yaw = self._base_yaw_world()
        c, s = math.cos(yaw), math.sin(yaw)
        self.data.ctrl[self.act["base_x"]] = c * vxb - s * vyb
        self.data.ctrl[self.act["base_y"]] = s * vxb + c * vyb
        self.data.ctrl[self.act["base_yaw"]] = wz
        if self.task_profile == "inspect":
            self._update_inspection_arm()
        elif self.task_profile == "vla":
            self._update_act_demo()
            self._update_act_arm()

        for _ in range(self._steps_per_tick):
            mujoco.mj_step(self.model, self.data)
        if self.task_profile == "vla":
            self._demo_follow_screwdriver()
            self._update_act_lift_detection()
        if self.task_profile == "inspect":
            self._publish_arm_arrival_if_ready()

        self._integrate_odom()

        # 关键：先发 TF 和传感器数据，最后发 clock。
        # 原因：rtabmap 用 use_sim_time，收到 clock 才更新内部时间。
        # 如果先发 clock 再发 TF，rtabmap 时间已经推进到 T，
        # 而 TF(T) 还没到 buffer 里，下一轮 clock(T+dt) 到了后
        # TF(T) 才到，就变成 "来自过去的数据" → TF_OLD_DATA。
        # 先发 TF 确保 buffer 里已有数据，再用 clock 推进时间。
        self.publish_odom()
        if (self.task_profile == "vla"
                and self._n % max(1, int(round(ODOM_HZ / ACT_CONTROL_HZ))) == 0):
            self._publish_act_observation()
            self._publish_act_task_status()
        self._n += 1
        if not self._mapping_complete and self._n % self._scan_every == 0:
            # 在 mj_step 之后扫描，保证点云与本帧发布的 TF/时间戳严格对应。
            # 避障（下一轮 tick 开头）复用这一帧，最多滞后 1/SCAN_HZ 秒。
            self._scan()
            if not self.no_lidar:
                self.publish_pointcloud()

        # clock 最后发，让所有 use_sim_time 订阅者在时间推进前已收到数据
        ck = Clock()
        ck.clock.sec, ck.clock.nanosec = self._stamp()
        self.pub_clock.publish(ck)

        if self.viewer is not None:
            self.viewer.sync()
        if (self.task_profile == "inspect"
                and self._inspection_preview_active
                and self._n % self._inspection_render_every == 0):
            self._render_inspection_camera()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="同时开 MuJoCo 查看器")
    ap.add_argument("--patrol", action="store_true", help="自动沿航点巡视")
    ap.add_argument("--no-lidar", action="store_true", help="不发布点云（用于离线导航）")
    ap.add_argument("--laps", type=int, default=PATROL_MAX_LAPS,
                    help="巡视几圈后自动停止并提示（0=无限循环）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--odom-noise", action="store_true",
                    help="注入里程计随机游走（仅用于回环/漂移压力测试）")
    ap.add_argument("--dynamic-person", action="store_true",
                    help="启用固定轨迹测试行人（ARIAC 2 个，warehouse 3 个）")
    ap.add_argument("--sim-speed", type=float, default=1.0, metavar="FACTOR",
                    help="仿真时间相对墙钟时间的倍率（推荐 1.0~2.0）")
    ap.add_argument("--inspection-fps", type=float,
                    default=INSPECTION_PREVIEW_HZ, metavar="FPS",
                    help="腕部相机预览刷新率（1~30 FPS，默认 %.1f；仅影响预览，不影响拍摄）"
                    % INSPECTION_PREVIEW_HZ)
    args = ap.parse_args()
    if not 0.1 <= args.sim_speed <= 4.0:
        ap.error("--sim-speed 必须在 0.1 到 4.0 之间")
    if not 1.0 <= args.inspection_fps <= 30.0:
        ap.error("--inspection-fps 必须在 1 到 30 之间")

    rclpy.init()
    node = SlamBridge3D(view=args.view, patrol=args.patrol, seed=args.seed,
                        no_lidar=args.no_lidar, max_laps=args.laps,
                        odom_noise=args.odom_noise,
                        dynamic_person=args.dynamic_person,
                        sim_speed=args.sim_speed,
                        inspection_fps=args.inspection_fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_inspection_camera()
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
