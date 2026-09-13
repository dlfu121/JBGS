"""Inspection-only camera, arm and navigation definitions."""

import math

import numpy as np


TASK_PROFILE = "inspect"
MAP_ORIGIN_WORLD = (4.0, 4.6)
CAMERA = "lefthand_camera"
CAMERA_EXTRINSICS = {
    CAMERA: {
        "pos": (-0.060, 0.000, 0.060),
        "quat_wxyz": (0.0, 0.7071067812, -0.7071067812, 0.0),
    },
}

# This posture is intentionally independent of the VLA/ACT initial posture.
# It matches inspection_route.json's forward-facing travel pose.
INITIAL_ARM_QPOS = (
    0.0,
    1.2,
    0.6,
    0.0,
    0.9,
    0.0,
)
ARM_COMMAND_SPEED = 0.8
ARM_ARRIVAL_TOL = 0.035


def yaw_degrees_to_quaternion_zw(yaw_deg):
    """Return the planar ROS quaternion components for an inspection stop."""
    half = math.radians(float(yaw_deg)) / 2.0
    return math.sin(half), math.cos(half)


def next_arm_control(current, target, elapsed_sec):
    """Advance an inspection joint target without using VLA IK settings."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    max_change = ARM_COMMAND_SPEED * float(elapsed_sec)
    return current + np.clip(target - current, -max_change, max_change)
