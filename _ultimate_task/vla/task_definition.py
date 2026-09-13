"""VLA-only robot, navigation, camera and motion definitions.

Keep task-level values here instead of in the shared bridge.  The MuJoCo
camera bodies remain in the dedicated VLA XML; these names are the contract
used by the ACT observation pipeline.
"""

import math

import numpy as np


TASK_PROFILE = "vla"

# Saved-map/world relationship and table approach.  The KeyCollect work table
# is placed so the tabletop centre sits TABLE_APPROACH_FORWARD (0.70 m) in
# front of the right-arm base, matching KeyCollect's arm/table geometry.
MAP_ORIGIN_WORLD = (4.0, 4.6)
TABLE_CENTER_WORLD = (7.4, 10.53)
TABLE_APPROACH_FORWARD = 0.70
VISIBLE_ROBOT_YAW_DEG = 90.0       # visible head faces world +Y
ROBOT_MODEL_REFERENCE_YAW = math.pi
RIGHT_ARM_BASE_LOCAL_XY = (-0.100, 0.175)
DOG_BASE_Z = -0.080
RIGHT_ARM_BASE_Z_LOCAL = 1.320

# VLA cameras.  Do not reuse the inspection camera setting here.
TABLE_CAMERA = "table_camera"
WRIST_CAMERA = "wrist_overhead_camera"
ACT_CAMERAS = (TABLE_CAMERA, WRIST_CAMERA)
# Local camera extrinsics in their owning MuJoCo bodies.  Quaternions use
# MuJoCo's wxyz order.  Keeping these values in the VLA profile prevents an
# inspection-camera calibration from silently changing ACT observations.
CAMERA_EXTRINSICS = {
    # The ACT checkpoint was trained with KeyCollect's table_camera, a fixed
    # world camera at (-0.62, 0, 0.80) with fovy=100 and xyaxes
    # (-0.136637,-0.990621,0, 0.134110,-0.018498,0.990794); relative to its
    # base_link (-0.7,0,0.6) that is (+0.080, 0, +0.200).  The ARIAC right arm
    # is mounted 180 deg about its base, so the same view is obtained by
    # mirroring that pose across the base (x -> -x, Rz(pi) on the orientation)
    # and keeping fovy=100.
    TABLE_CAMERA: {
        "pos": (-0.080, 0.000, 0.200),
        "quat_wxyz": (-0.5680040917, -0.4956711251,
                      -0.4319957069, -0.4950365610),
        "fovy": 100.0,
    },
    WRIST_CAMERA: {
        "pos": (0.020175, 0.001232, 0.016614),
        "quat_wxyz": (0.1418047170, 0.6927419593,
                      -0.6917736824, -0.1464553595),
        "fovy": 100.0,
    },
}

# VLA/ACT starts from its own arm posture and owns its own differential-IK
# limits.  Future inspection tuning must not change these values.
INITIAL_ARM_QPOS = (
    3.100000,
    -0.401426,
    -1.727876,
    0.0,
    0.471239,
    3.141593,
)
ACT_TRANSLATION_STEP = 0.01
ACT_ROTATION_STEP = 0.05
ACT_IK_DAMPING = 1e-3
ACT_JOINT_STEP = 0.10
ACT_HAND_STEP = 0.05
# The position servos follow KeyCollect's 24 Hz Cartesian target stream.  A
# low slew cap here makes every ACT chunk visibly lag behind the policy.
ACT_ARM_COMMAND_SPEED = 2.4

# Scripted teleport-grasp demo (no ACT).  ``run_vla.py`` triggers it through
# ``/act/demo_grasp`` once navigation reports ARRIVED.  The bridge holds the
# right arm, waits DEMO_HOLD_SEC, closes the hand to a fist, snaps the
# screwdriver into the palm and keeps it rigidly attached while the arm returns
# to INITIAL_ARM_QPOS (the fingers stay closed).
DEMO_HOLD_SEC = 5.0
DEMO_ARM_ARRIVAL_TOL = 0.05
DEMO_ARM_RETURN_TIMEOUT_SEC = 30.0
# Screwdriver grasp pose expressed in hand_r/right_hand_base: position (m) and
# orientation (wxyz).  The screwdriver's long axis is its local +X, so the
# default 90-degree Z rotation lays it across the fingers; the -0.05 m y offset
# roughly centres the 0.36 m tool in the palm.
DEMO_GRASP_POS = (0.010, -0.050, 0.130)
DEMO_GRASP_QUAT_WXYZ = (0.7071067812, 0.0, 0.0, 0.7071067812)


def yaw_degrees_to_quaternion_zw(yaw_deg):
    """Return the planar ROS quaternion components for a VLA heading."""
    half = math.radians(float(yaw_deg)) / 2.0
    return math.sin(half), math.cos(half)


def cartesian_action_delta(action):
    """Clamp one VLA Cartesian action using only VLA-owned limits."""
    values = np.asarray(action, dtype=float)
    return np.r_[
        np.clip(values[:3], -ACT_TRANSLATION_STEP, ACT_TRANSLATION_STEP),
        np.clip(values[3:6], -ACT_ROTATION_STEP, ACT_ROTATION_STEP),
    ]


def damped_ik_joint_delta(jacobian, task_delta):
    """Solve the VLA differential IK step with damped least squares."""
    jacobian = np.asarray(jacobian, dtype=float)
    task_delta = np.asarray(task_delta, dtype=float)
    delta = jacobian.T.dot(np.linalg.solve(
        jacobian.dot(jacobian.T) + ACT_IK_DAMPING * np.eye(6),
        task_delta))
    return np.clip(delta, -ACT_JOINT_STEP, ACT_JOINT_STEP)


def table_approach_target():
    """Return the VLA mobile-base pose and right-arm grasp baseline."""
    baseline_x = TABLE_CENTER_WORLD[0]
    baseline_y = TABLE_CENTER_WORLD[1] - TABLE_APPROACH_FORWARD
    model_yaw = ROBOT_MODEL_REFERENCE_YAW + math.radians(
        VISIBLE_ROBOT_YAW_DEG)
    c, s = math.cos(model_yaw), math.sin(model_yaw)
    local_x, local_y = RIGHT_ARM_BASE_LOCAL_XY
    arm_offset_x = local_x * c - local_y * s
    arm_offset_y = local_x * s + local_y * c
    return {
        "baseline_x": baseline_x,
        "baseline_y": baseline_y,
        "arm_base_x": baseline_x,
        "arm_base_y": baseline_y,
        "arm_base_z": DOG_BASE_Z + RIGHT_ARM_BASE_Z_LOCAL,
        "dog_x": baseline_x - arm_offset_x,
        "dog_y": baseline_y - arm_offset_y,
        "dog_z": DOG_BASE_Z,
        "dog_yaw": math.radians(VISIBLE_ROBOT_YAW_DEG),
    }
