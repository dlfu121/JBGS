#!/usr/bin/env python3
"""把「四足机械狗 + 双机械臂(RM65-6F) + 双灵巧手(dexhand021)」组装成一个 MuJoCo 机器人模型。

注意: 本脚本需要 mujoco >= 3.6（MjSpec 类方法 API: from_file, attach, 迭代器等）。
      当前环境若为 mujoco 3.2.3，请使用已生成的 Python 3.8 兼容 3D XML。

结构（父 → 子）:
    dog_base (固定, 无自由度)
      ├─ arm_l/base_link → link_1..6   (左臂, 6 DOF)  → hand_l/left_hand_base  → 20 关节
      └─ arm_r/base_link → link_1..6   (右臂, 6 DOF)  → hand_r/right_hand_base → 20 关节

要点:
  - 狗 STL 单位是 mm，缩放到 m；平移使足底贴 z=0、脚印中心对齐 body 原点。
  - 狗视觉用原始高精 STL；碰撞用一个简化长方体（避免 390 万面拖垮仿真）。
  - 两臂在狗背顶面(约 1.46 m)左右对称肩装，base 竖直朝上。
  - 用 MjSpec.attach 把各 URDF 子模型按前缀合并进主模型，避免手写坐标错配。
  - 每个转动关节配 position 执行器，便于后续控制。

用法:
    python3 build_robot.py            # 写出 robot.xml 并校验编译
    python3 build_robot.py --view     # 组装后打开查看器
"""
import argparse
import os
import re

import math

import mujoco

try:
    from .initial_pose import ARM_JOINT_NAMES, ARM_READY_QPOS, reset_to_ready
except ImportError:
    from initial_pose import ARM_JOINT_NAMES, ARM_READY_QPOS, reset_to_ready

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.join(HERE, "..", "assets")
URDF_DIR = os.path.join(ASSET_DIR, "urdf")


def quat_x(angle):
    """绕 x 轴旋转 angle(rad) 的四元数 [w,x,y,z]。"""
    return [math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0]


def quat_z(angle):
    """绕 z 轴旋转 angle(rad) 的四元数 [w,x,y,z]。"""
    return [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]

# --- 手背腕相机（eye-in-hand）参数，与 model/camera 项目保持一致 -----------
CAM_RES = (640, 480)                   # 相机分辨率
CAM_FOVY = 68.0                        # 垂直视场角(度)，确保整块表盘带边距入镜
CAM_POS = [-0.06, 0.0, 0.06]           # 手基座局部：手背(-X)侧 6cm，沿手指(+Z) 6cm

# --- 仿真稳定性调参 -------------------------------------------------------
HAND_ARMATURE = 0.002                  # 手部小关节加转动惯量，抑制高频抖动
ARM_KV = 15.0                          # 双臂位置执行器阻尼(biasprm[2] = -ARM_KV)

# 只给机器人(臂/手)body 开重力补偿，场景其余部分(螺丝刀、料车、扫掠杆)保持真实重力。
# 原因：位置执行器 kp=40 顶不住臂+手自重，ready 姿态下 joint_2 会下坠约 22°
# (kp*(1.2-1.5798) 正好等于重力矩 -15.19 N·m)，手背相机跟着偏离目标。
# 1.0=完全补偿(臂精确保持指令姿态，等价真实机械臂的重力前馈)；0.0=真实下坠。
ROBOT_GRAVCOMP = 1.0

# --- 狗几何参数（由 STL 包围盒算出，单位换算见下）-------------------------
DOG_SCALE = 0.001                      # mm -> m
# 原始 STL 包围盒(mm): min(532.97,28.81,21.17) max(1674.38,2909.43,1484.58)
DOG_BB_MIN = (532.9708, 28.81336, 21.168306)
DOG_BB_MAX = (1674.3787, 2909.4346, 1484.5803)
# 修改后 STL
DOG_HULL_CENTER = (0.0, 0.0, 0.772)    # 局部系中心
DOG_HULL_HALF = (0.380, 0.313, 0.580)  # 半尺寸 → 实际 0.76 × 0.63 × 1.16 m

# --- 移动底盘 -------------------------------------------------------------
# 等效差速轮式底盘的运动学（可原地转向）。SLAM 只需里程计+激光，不必模拟步态。
BASE_DRIVE = True
BASE_VEL_KV = 1200.0                   # 平移速度执行器增益
BASE_YAW_KV = 400.0                    # 转向速度执行器增益
BASE_DAMPING = 6.0
BASE_YAW_DAMPING = 2.0

DOG_MASS = 60.0
DOG_INERTIA = (8.5, 10.5, 5.0)         # Ixx, Iyy, Izz (kg·m²)

# --- 2D 兼容激光雷达（生成基础模型时保留，3D 桥接运行时不使用）--------
LIDAR_NAME = "lidar"
# 注意坐标系：狗 STL 修正后，狗正面是 dog_base 局部 -x；初始姿态下 dog_base
# 绕 z 转 180°，所以狗正面朝世界 +x。故"机身后方"是局部 +x。
LIDAR_POS = (-0.454, 0.0, 0.9)          # 狗局部系：机身后方略微伸出、低位
LIDAR_NUM_RAYS = 360                   # 1°/束
LIDAR_RANGE_MAX = 12.0                 # 房间对角约 16 m，取 12 m 够用
LIDAR_RANGE_MIN = 0.12

# --- 3D 激光雷达（多层 rangefinder 阵列，类 VLP-16）-----------------------
LIDAR3D_NAME = "lidar3d"
LIDAR3D_POS = (-0.454, 0.0, 0.95)      # 狗局部系，略高于 2D 雷达
LIDAR3D_H_RAYS = 180                    # 水平束数（2°/束，覆盖 360°）
LIDAR3D_V_LAYERS = 16                   # 垂直层数
LIDAR3D_V_MIN = math.radians(-15.0)     # 最低仰角(rad)
LIDAR3D_V_MAX = math.radians(15.0)      # 最高仰角(rad)
LIDAR3D_RANGE_MAX = 8.0
LIDAR3D_RANGE_MIN = 0.12


def _dog_mesh_offset():
    """让脚印中心对齐 body 原点、足底贴 z=0 的网格平移量(m)。"""
    cx = (DOG_BB_MIN[0] + DOG_BB_MAX[0]) / 2 * DOG_SCALE
    cy = (DOG_BB_MIN[1] + DOG_BB_MAX[1]) / 2 * DOG_SCALE
    tz = -DOG_BB_MIN[2] * DOG_SCALE
    return (-cx, -cy, tz)


def _dog_size():
    return tuple((DOG_BB_MAX[i] - DOG_BB_MIN[i]) * DOG_SCALE for i in range(3))


# ---------------------------------------------------------------------------
# URDF -> MjSpec：修正 mesh 路径后加载（资源均来自本项目 assets）
# ---------------------------------------------------------------------------
def load_arm_spec():
    """RM65-6F 臂。原 URDF 无 <mujoco> 标签且用 package:// 引用，需注入 meshdir。"""
    src = os.path.join(URDF_DIR, "RM65-6F.urdf")
    txt = open(src, encoding="utf-8").read()
    txt = txt.replace("package://RM65-6F/meshes/", "")
    meshdir = os.path.join(ASSET_DIR, "arm") + "/"
    inject = (f'<robot name="RM65-6F">\n'
              f'  <mujoco><compiler meshdir="{meshdir}" '
              f'balanceinertia="true" discardvisual="false"/></mujoco>')
    txt = re.sub(r'<robot\s+name="RM65-6F">', inject, txt, count=1)
    return mujoco.MjSpec.from_string(txt)


def load_hand_spec(side):
    """dexhand021 手。原 meshdir 指向不存在的目录，改指 assets/hand。
    side='right' → 右手模型；side='left' → 左手模型。"""
    src = os.path.join(URDF_DIR, f"dexhand021_{side}_simplified.urdf")
    txt = open(src, encoding="utf-8").read()
    meshdir = os.path.join(ASSET_DIR, "hand") + "/"
    txt = re.sub(r'meshdir="[^"]*"', f'meshdir="{meshdir}"', txt)
    txt = txt.replace("../meshes/dexhand021_simplified/", "")
    return mujoco.MjSpec.from_string(txt)


# ---------------------------------------------------------------------------
# 手背腕相机（eye-in-hand）
# ---------------------------------------------------------------------------
def _cam_local_quat():
    """相机装在手背、光轴沿手指(reach, 局部 +Z)方向的局部四元数[w,x,y,z]。

    MuJoCo 相机看向自身 -Z，+X 右，+Y 上。令：
      光轴(view) = -cam_z = 手 +Z(reach)  → cam_z = -手Z
      cam_y(上)  = 手 -X(保持手背相机画面正立)
      cam_x      = cam_y × cam_z
    列向量(相机轴在手基座局部系中的坐标)组成旋转矩阵。"""
    import numpy as np
    hand_x = np.array([1.0, 0, 0])   # 掌心
    hand_z = np.array([0, 0, 1.0])   # 手指/reach
    cam_z = -hand_z
    cam_y = -hand_x
    cam_x = np.cross(cam_y, cam_z)
    R = np.column_stack([cam_x, cam_y, cam_z])   # 相机->手 的旋转
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.flatten())
    return quat.tolist()


def _add_hand_cameras(spec):
    """左手保留巡检用的 lefthand_camera；右臂只保留 ACT 的 table/wrist 相机。"""
    quat = _cam_local_quat()
    body = spec.body("hand_l/left_hand_base")
    cam = body.add_camera()
    cam.name = "lefthand_camera"
    cam.pos = list(CAM_POS)
    cam.quat = quat
    cam.fovy = CAM_FOVY
    cam.resolution = list(CAM_RES)


def _add_lidar(spec, dog):
    """在狗身上装一圈 rangefinder，构成 2D 激光雷达。

    MuJoCo 的 rangefinder 沿 site 的局部 **+z** 方向投射并返回到最近几何的距离
    （无命中返回 -1）。要得到水平扫描平面，就得把每个 site 绕自身翻转，让 +z
    指向水平方位角 θ：先绕 y 轴转 90° 把 +z 压到水平面(+x)，再绕 z 轴转 θ。

    θ 定义在狗局部系中，0 = 局部 +x，逆时针增大，覆盖 [-π, π)。
    """
    import numpy as np
    # 关键：所有 site 和外壳 geom 都直接挂在 **dog_base** 上，不另建子 body。
    # MuJoCo 的 rangefinder 内部按「site 所属 body」做 bodyexclude —— 只排除自己
    # 那一个 body 的 geom。挂在 dog_base 上，狗的视觉网格和碰撞盒就都被排除了；
    # 若另建子 body，狗身反而会被自己的激光打到：减面网格里那 22 个离群顶点会在
    # 0.33 m 处形成 20 束假回波（实测），会被当成贴着车身的障碍物。
    # 真实雷达驱动同样要滤掉自车回波，所以这也符合物理。

    # 传感器系遵循 ROS REP-103：+x = 车辆前进方向、+z 向上。
    # dog_base 局部 -x 是狗正面，故雷达系相对狗要绕 z 转 180°。这样 θ=0 的射线
    # 朝正前方、θ 逆时针增大，与 LaserScan 的 angle_min/angle_increment 语义一致。
    q_hub = quat_z(math.pi)

    def _compose(q_local):
        """把 hub 朝向叠加到射线的局部朝向上（sites 直接挂 dog_base，需自己合成）。"""
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, np.array(q_hub), np.array(q_local))
        return out

    # 一个小圆柱做外观，纯视觉不参与碰撞
    gv = dog.add_geom()
    gv.name = LIDAR_NAME + "_case"
    gv.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    gv.size = [0.038, 0.028, 0]
    gv.pos = list(LIDAR_POS)
    gv.rgba = [0.10, 0.10, 0.12, 1.0]
    gv.contype = 0
    gv.conaffinity = 0
    gv.group = 2

    # 参考 site：即雷达坐标系本身(+x 为 0° 方位)，供发布 TF / frame_id 用
    ref = dog.add_site()
    ref.name = LIDAR_NAME + "_frame"
    ref.pos = list(LIDAR_POS)
    ref.quat = _compose([1, 0, 0, 0]).tolist()
    ref.size = [0.01, 0.01, 0.01]
    ref.rgba = [0, 1, 1, 0.0]

    # 绕 y 转 +90°：+z → +x（rangefinder 沿 site 局部 +z 投射）
    q_y90 = [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0]
    for i in range(LIDAR_NUM_RAYS):
        theta = -math.pi + 2 * math.pi * i / LIDAR_NUM_RAYS
        q_local = np.zeros(4)
        mujoco.mju_mulQuat(q_local, np.array(quat_z(theta)), np.array(q_y90))
        s = dog.add_site()
        s.name = f"{LIDAR_NAME}_s{i:03d}"
        s.pos = list(LIDAR_POS)
        s.quat = _compose(q_local).tolist()
        s.size = [0.002, 0.002, 0.002]
        s.rgba = [1, 0, 0, 0.0]          # 不可见，避免 360 个红点糊住画面

        sen = spec.add_sensor()
        sen.name = f"{LIDAR_NAME}_r{i:03d}"
        sen.type = mujoco.mjtSensor.mjSENS_RANGEFINDER
        sen.objtype = mujoco.mjtObj.mjOBJ_SITE
        sen.objname = s.name
        sen.cutoff = LIDAR_RANGE_MAX
        # add_sensor() 默认 intprm[0]=0，但编译器要求 >0（XML 路径下默认是 1）
        sen.intprm = [1, 0, 0]


def _add_lidar_3d(spec, dog):
    """在狗身上装多层 rangefinder 阵列，模拟 VLP-16 风格 3D 激光雷达。

    共 LIDAR3D_V_LAYERS 层 × LIDAR3D_H_RAYS 束 = 2880 根射线。
    每根射线由一个 site（朝向编码方位角 θ 和仰角 φ）+ rangefinder sensor 组成。
    传感器命名: lidar3d_r{layer:02d}_{az:03d}，layer=0 为最低仰角层。
    """
    import numpy as np

    q_hub = quat_z(math.pi)  # 与 2D 相同的 hub 朝向修正

    def _compose(q_local):
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, np.array(q_hub), np.array(q_local))
        return out

    # 外壳视觉 geom
    gv = dog.add_geom()
    gv.name = LIDAR3D_NAME + "_case"
    gv.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    gv.size = [0.05, 0.04, 0]
    gv.pos = list(LIDAR3D_POS)
    gv.rgba = [0.08, 0.08, 0.10, 1.0]
    gv.contype = 0
    gv.conaffinity = 0
    gv.group = 2

    # 参考 site（雷达坐标系原点）
    ref = dog.add_site()
    ref.name = LIDAR3D_NAME + "_frame"
    ref.pos = list(LIDAR3D_POS)
    ref.quat = _compose([1, 0, 0, 0]).tolist()
    ref.size = [0.01, 0.01, 0.01]
    ref.rgba = [0, 1, 1, 0.0]

    # 仰角间距
    if LIDAR3D_V_LAYERS > 1:
        v_step = (LIDAR3D_V_MAX - LIDAR3D_V_MIN) / (LIDAR3D_V_LAYERS - 1)
    else:
        v_step = 0.0

    # Ry(α): 绕 y 轴转 α 的四元数
    def quat_y(a):
        return [math.cos(a / 2), 0.0, math.sin(a / 2), 0.0]

    for layer in range(LIDAR3D_V_LAYERS):
        phi = LIDAR3D_V_MIN + layer * v_step  # 仰角
        for az in range(LIDAR3D_H_RAYS):
            theta = -math.pi + 2 * math.pi * az / LIDAR3D_H_RAYS
            # site +z 需指向 (θ, φ): Rz(θ) × Ry(90°-φ)
            q_elev = quat_y(math.pi / 2 - phi)
            q_az = quat_z(theta)
            q_local = np.zeros(4)
            mujoco.mju_mulQuat(q_local, np.array(q_az), np.array(q_elev))

            s = dog.add_site()
            s.name = f"{LIDAR3D_NAME}_s{layer:02d}_{az:03d}"
            s.pos = list(LIDAR3D_POS)
            s.quat = _compose(q_local).tolist()
            s.size = [0.001, 0.001, 0.001]
            s.rgba = [0, 0, 1, 0.0]

            sen = spec.add_sensor()
            sen.name = f"{LIDAR3D_NAME}_r{layer:02d}_{az:03d}"
            sen.type = mujoco.mjtSensor.mjSENS_RANGEFINDER
            sen.objtype = mujoco.mjtObj.mjOBJ_SITE
            sen.objname = s.name
            sen.cutoff = LIDAR3D_RANGE_MAX
            sen.intprm = [1, 0, 0]


def _flip_dog_mesh(spec):
    """把 dog_base 下已存在的狗视觉/碰撞几何绕 z 轴转 180°。

    只处理当前已建好的狗 body 几何，不动之后才添加的臂、手、雷达与 site。
    这样可以沿用 camera 项目里正确的狗朝向，但不影响 SLAM 的 lidar 坐标系。
    """
    qz180 = quat_z(math.pi)
    for g in spec.geoms:
        b = g.parent
        if b is not None and b.name == "dog_base":
            g.pos = [-g.pos[0], -g.pos[1], g.pos[2]]
            g.quat = qz180


def _add_base_actuators(spec):
    """给底盘三个自由度配速度执行器，ctrl 即 (vx, vy, wz) 指令。"""
    for jn, kv in (("base_x", BASE_VEL_KV),
                   ("base_y", BASE_VEL_KV),
                   ("base_yaw", BASE_YAW_KV)):
        a = spec.add_actuator()
        a.name = "act_" + jn
        a.set_to_velocity(kv=kv)
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.target = jn


def _stabilize(spec):
    """仿真稳定性调参：隐式积分器 + 手关节 armature + 双臂执行器阻尼。"""
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for j in spec.joints:
        if j.name.startswith(("hand_l/", "hand_r/")):
            j.armature = HAND_ARMATURE
    for a in spec.actuators:
        if a.name.startswith(("act_arm_l/", "act_arm_r/")):
            bp = list(a.biasprm)      # biasprm 是长度10数组，只改 kv(index 2)
            bp[2] = -ARM_KV
            a.biasprm = bp
    if ROBOT_GRAVCOMP:
        for b in spec.bodies:
            if b.name.startswith(("arm_l/", "arm_r/", "hand_l/", "hand_r/")):
                b.gravcomp = ROBOT_GRAVCOMP


# ---------------------------------------------------------------------------
# 主装配
# ---------------------------------------------------------------------------
def build(spec=None):
    """组装机器人。spec 为 None 时新建独立模型；否则把机器人装入已有场景 spec。"""
    standalone = spec is None
    if standalone:
        spec = mujoco.MjSpec()
        spec.modelname = "dog_dual_arm"
    spec.compiler.degree = False          # 用弧度，和 URDF 一致

    # 狗视觉网格作为 asset：原 STL 有 389 万面，超 MuJoCo 单网格 20 万面上限，
    # 已减面并按 <20 万面切成多块（见 decimate 步骤），逐块作为独立 mesh。
    import glob
    dog_parts = sorted(glob.glob(os.path.join(ASSET_DIR, "dog", "dog_visual_*.STL")))
    if not dog_parts:
        raise FileNotFoundError("缺少减面后的狗视觉网格 assets/dog/dog_visual_*.STL")
    for i, fp in enumerate(dog_parts):
        m = spec.add_mesh()
        m.name = f"dog_mesh_{i}"
        m.file = fp
        m.scale = [DOG_SCALE, DOG_SCALE, DOG_SCALE]

    mat = spec.add_material()
    mat.name = "dog_mat"
    mat.rgba = [0.35, 0.37, 0.40, 1.0]

    # --- 狗本体 ---
    # BASE_DRIVE 时挂在 dog_chassis 下并带 3 个平面自由度；否则保持原来的焊死行为。
    dog = spec.worldbody.add_body()
    dog.name = "dog_base"
    dog.pos = [-8.5, 0.0, 0.0]
    dog.quat = quat_z(math.pi)

    # 显式惯性参数，覆盖 inertiafromgeom 从视觉网格算出的 1494 kg（见 DOG_MASS 注释）
    dog.explicitinertial = True
    dog.mass = DOG_MASS
    dog.inertia = list(DOG_INERTIA)
    dog.ipos = [0.0, 0.0, DOG_HULL_CENTER[2]]
    dog.iquat = [1, 0, 0, 0]

    if BASE_DRIVE:
        # 关节都加在 dog_base 自身上：slide 沿世界轴、hinge 绕竖直轴。
        # 注意 slide 轴要用【世界方向】表达，但 joint axis 是在 body 局部系里
        # 解释的，而 dog_base 已绕 z 转 180°，故局部 -x = 世界 +x、局部 -y = 世界 +y。
        jx = dog.add_joint()
        jx.name = "base_x"
        jx.type = mujoco.mjtJoint.mjJNT_SLIDE
        jx.axis = [-1, 0, 0]             # 局部 -x → 世界 +x
        jx.damping = [BASE_DAMPING, 0, 0]
        jy = dog.add_joint()
        jy.name = "base_y"
        jy.type = mujoco.mjtJoint.mjJNT_SLIDE
        jy.axis = [0, -1, 0]             # 局部 -y → 世界 +y
        jy.damping = [BASE_DAMPING, 0, 0]
        jt = dog.add_joint()
        jt.name = "base_yaw"
        jt.type = mujoco.mjtJoint.mjJNT_HINGE
        jt.axis = [0, 0, 1]
        jt.damping = [BASE_YAW_DAMPING, 0, 0]

    ox, oy, oz = _dog_mesh_offset()
    dsx, dsy, dsz = _dog_size()

    # 视觉：减面后的多块网格（不参与碰撞）
    for i in range(len(dog_parts)):
        gv = dog.add_geom()
        gv.name = f"dog_visual_{i}"
        gv.type = mujoco.mjtGeom.mjGEOM_MESH
        gv.meshname = f"dog_mesh_{i}"
        gv.pos = [ox, oy, oz]
        gv.material = "dog_mat"
        gv.contype = 0
        gv.conaffinity = 0
        gv.group = 2

    # 碰撞：简化长方体。尺寸取稳健包围盒（见 DOG_HULL_* 注释），不能用 STL
    # 包围盒 —— 那会因 22 个离群顶点得到 2.88 m 长的盒子，狗在走廊里转不开身。
    gc = dog.add_geom()
    gc.name = "dog_collision"
    gc.type = mujoco.mjtGeom.mjGEOM_BOX
    gc.size = list(DOG_HULL_HALF)
    gc.pos = list(DOG_HULL_CENTER)
    gc.rgba = [0.4, 0.4, 0.45, 0.0]       # 透明，仅用于碰撞
    gc.group = 3
    gc.contype = 2                        # 机器人碰撞组：只与环境(conaffinity 含 bit0)相碰
    gc.conaffinity = 1

    # 狗朝向沿用 camera 项目的正确版本：只翻转狗自己的视觉/碰撞几何，
    # 机械臂装配位姿和后续雷达 site 仍按 dog_base 坐标系保留。
    _flip_dog_mesh(spec)

    # --- 肩部安装点（均在狗 body 局部坐标系，数值来自减面网格实测 AABB）---
    # 狗 body 绕 z 转 180°，狗正面朝世界 +x。从狗身后面向 +x 看，世界 +y
    # 是左侧、世界 -y 是右侧；对应狗局部坐标分别是 -y 和 +y。
    # 实测背部上表面(狗局部)：X∈[-0.47,0.47], Y∈[-0.29,0.29], 顶面 z≈1.35~1.46。
    shoulder_z = 1.32                     # 贴合背部上表面
    shoulder_sep = 0.175                   # 沿狗局部 Y 对称安装
    shoulder_ctr = -0.1                   # 局部 X 居中(背部前后中央)
    ARM_TILT = 0.0                        # 肩部外倾角(rad)，0=竖直朝上
    SHOULDER_YAW = 0.0                    # 与 camera 项目的臂装配 yaw 保持一致

    def mount_arm(prefix, sep_local_y, hand_spec, hand_prefix, tilt_sign,
                  hand_mount_z=0.0):
        arm = load_arm_spec()            # 每臂新建，attach 会消费子 spec
        fr = dog.add_frame()
        fr.pos = [shoulder_ctr, sep_local_y, shoulder_z]
        fr.quat = quat_z(SHOULDER_YAW)
        spec.attach(arm, prefix=prefix + "/", frame=fr)
        # 臂末端 link_6 接手
        tip = spec.body(prefix + "/link_6")
        hf = tip.add_frame()
        hf.pos = [0, 0, hand_mount_z]
        hf.quat = [1, 0, 0, 0]
        spec.attach(hand_spec, prefix=hand_prefix + "/", frame=hf)

    # 左臂：狗局部 -Y = 世界 +Y，并安装左手 URDF。
    # 右臂：狗局部 +Y = 世界 -Y，并安装右手 URDF。
    mount_arm("arm_l", -shoulder_sep, load_hand_spec("left"), "hand_l", +1)
    # 右手基座与 KeyCollect 训练模型一致：沿 link_6 局部 -Z 偏移 8 cm。
    mount_arm("arm_r", +shoulder_sep, load_hand_spec("right"), "hand_r", -1,
              hand_mount_z=-0.08)

    # attach 后，子模型 mesh 的 file 只剩相对名（顶层 compiler 无对应 meshdir），
    # 逐个解析成绝对路径：臂网格在 assets/arm，手网格在 assets/hand。
    arm_dir = os.path.join(ASSET_DIR, "arm")
    hand_dir = os.path.join(ASSET_DIR, "hand")
    for m in spec.meshes:
        if os.path.isabs(m.file):
            continue                      # 狗网格已是绝对路径
        fn = os.path.basename(m.file)
        if os.path.exists(os.path.join(arm_dir, fn)):
            m.file = os.path.join(arm_dir, fn)
        elif os.path.exists(os.path.join(hand_dir, fn)):
            m.file = os.path.join(hand_dir, fn)
        else:
            raise FileNotFoundError(f"找不到 mesh: {fn}")

    # --- 执行器：仅给机器人（臂/手）关节配 position 控制，不碰场景里已有关节 ---
    robot_prefixes = ("arm_l/", "arm_r/", "hand_l/", "hand_r/")
    for j in spec.joints:
        if not j.name.startswith(robot_prefixes):
            continue
        if j.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            a = spec.add_actuator()
            a.name = "act_" + j.name
            a.set_to_position(kp=40.0, kv=4.0)
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.target = j.name

    # --- 碰撞分组：臂/手所有 geom 设为“机器人组”，彼此不自碰，只与环境相撞 ---
    # contype=2, conaffinity=1 → 机器人 geom 之间(2&2 无 bit0)不接触；
    # 与环境默认 geom(contype=1,conaffinity=1)：2&1? 机器人conaff=1 含 bit0，环境contype=1 → 相撞。
    robot_body_prefixes = ("arm_l/", "arm_r/", "hand_l/", "hand_r/")
    for g in spec.geoms:
        b = g.parent
        bn = b.name if b is not None else ""
        if bn.startswith(robot_body_prefixes):
            g.contype = 2
            g.conaffinity = 1

    # --- 手背腕相机 + 激光雷达 + 底盘执行器 + 稳定性调参 ---
    # 都要在臂/手执行器建好之后：_stabilize 按名字前缀筛执行器，
    # 底盘执行器不带 act_arm_ 前缀所以不受影响。
    _add_hand_cameras(spec)
    _add_lidar(spec, dog)
    _add_lidar_3d(spec, dog)
    if BASE_DRIVE:
        _add_base_actuators(spec)
    _stabilize(spec)

    # --- RM65 参考场景 home 姿态（双臂使用相同关节角） ---
    # 六关节角直接来自 assets/ariac/rm65_dexhand_scene.xml 的 home keyframe。
    arm_pose = dict(zip(ARM_JOINT_NAMES, ARM_READY_QPOS))
    _add_ready_keyframe(spec, arm_pose)

    return spec


def _add_ready_keyframe(spec, arm_pose):
    """按 arm_pose 给两臂关节设初值，写成 keyframe，并让执行器 ctrl 保持该姿态。"""
    # 先编译一份拿到关节/执行器地址布局
    model = spec.compile()
    qpos = model.qpos0.copy()
    ctrl = None
    import numpy as np
    ctrl = np.zeros(model.nu)
    for side in ("arm_l", "arm_r"):
        for jn, val in arm_pose.items():
            full = f"{side}/{jn}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full)
            if jid < 0:
                continue
            qadr = model.jnt_qposadr[jid]
            qpos[qadr] = val
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + full)
            if aid >= 0:
                ctrl[aid] = val
    key = spec.add_key()
    key.name = "ready"
    key.qpos = qpos.tolist()
    key.ctrl = ctrl.tolist()


def build_into_scene(scene_xml):
    """把机器人装进已有场景 XML，返回合并后的 spec。"""
    spec = mujoco.MjSpec.from_file(scene_xml)
    return build(spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "robot.xml"))
    ap.add_argument("--scene", default=None,
                    help="已有场景 XML；给出则把机器人装入该场景后输出")
    ap.add_argument("--view", action="store_true")
    args = ap.parse_args()

    if args.scene:
        spec = build_into_scene(args.scene)
    else:
        spec = build()
    model = spec.compile()
    print(f"组装成功: nbody={model.nbody}, njnt={model.njnt}, "
          f"nu={model.nu}, ngeom={model.ngeom}, nmesh={model.nmesh}")

    xml = spec.to_xml()
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(xml)
    print(f"已写出 {args.out}")

    # ROS 侧（slam_bridge.py）必须跑在 python3.8 上，因为 Foxy 的 rclpy 只有
    # cpython-38 扩展；而 py3.8 能装的最高版本是 mujoco 3.2.3，它不认识
    # mujoco 3.10 写出的 texture colorspace 属性。这里同时输出一份去掉该属性的
    # 副本给 ROS 侧用（其余内容逐字节相同，3.2.3 实测可正常加载并读出雷达）。
    compat = re.sub(r'\s+colorspace="[^"]*"', '', xml)
    compat_path = os.path.splitext(args.out)[0] + "_py38.xml"
    with open(compat_path, "w", encoding="utf-8") as f:
        f.write(compat)
    print(f"已写出 {compat_path}（python3.8 / mujoco 3.2.3 兼容版，供 ROS 桥接用）")

    if args.view:
        import mujoco.viewer
        data = mujoco.MjData(model)
        reset_to_ready(mujoco, model, data)
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
