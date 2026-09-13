"""Shared initial posture for the mobile robot's two RM65 arms.

Edit ``ARM_READY_QPOS`` when the common debug/runtime posture changes, then
regenerate robot XML files with ``build_robot.py`` / ``gen_ariac_robot.py``.
Runtime tools also apply these values by joint name, so composed scenes remain
correct even when environment joints change the qpos layout.
"""

ARM_JOINT_NAMES = tuple("joint_%d" % index for index in range(1, 7))

# joint_1 .. joint_6, radians. Both arms use the same posture.
ARM_READY_QPOS = (0.0, 1.2, 0.6, 0.0, 0.9, 0.0)

# Right DexHand home posture, matching KeyCollect's recorded start: the thumb
# is flexed (50/30 deg) while the other four fingers stay open.  Leaving the
# thumb out of the palm keeps the ACT wrist camera unobstructed at reset.
HAND_JOINT_NAMES = tuple(
    "r_f_joint%d_%d" % (finger, joint)
    for finger in range(1, 6) for joint in range(1, 5))
HAND_READY_QPOS = (0.872665, 0.523599, 0.0, 0.0) + (0.0,) * 16


def apply_hand_pose(mujoco, model, data, pose=HAND_READY_QPOS, strict=False):
    """Apply the right DexHand home posture by joint/actuator name."""
    applied = 0
    for joint_basename, target in zip(HAND_JOINT_NAMES, pose):
        joint_name = "hand_r/%s" % joint_basename
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + joint_name)
        if joint_id < 0:
            if strict:
                raise RuntimeError("initial pose requires joint %s" % joint_name)
            continue
        data.qpos[model.jnt_qposadr[joint_id]] = float(target)
        if actuator_id >= 0:
            data.ctrl[actuator_id] = float(target)
        applied += 1
    return applied


def apply_arm_pose(mujoco, model, data, pose=ARM_READY_QPOS, strict=True):
    """Apply and hold an arm posture by joint/actuator name."""
    if len(pose) != len(ARM_JOINT_NAMES):
        raise ValueError("arm pose must contain exactly 6 joint values")

    applied = 0
    for side in ("arm_l", "arm_r"):
        for joint_basename, target in zip(ARM_JOINT_NAMES, pose):
            joint_name = "%s/%s" % (side, joint_basename)
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + joint_name)
            if joint_id < 0:
                if strict:
                    raise RuntimeError("initial pose requires joint %s" % joint_name)
                continue
            data.qpos[model.jnt_qposadr[joint_id]] = float(target)
            if actuator_id >= 0:
                data.ctrl[actuator_id] = float(target)
            elif strict:
                raise RuntimeError(
                    "initial pose requires actuator act_%s" % joint_name)
            applied += 1
    return applied


def reset_to_ready(mujoco, model, data, pose=ARM_READY_QPOS, strict=True):
    """Reset the scene and apply the shared arm posture by name."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    else:
        mujoco.mj_resetData(model, data)
    applied = apply_arm_pose(mujoco, model, data, pose=pose, strict=strict)
    apply_hand_pose(mujoco, model, data)
    mujoco.mj_forward(model, data)
    return applied


def ready_keyframe_values(mujoco, model, pose=ARM_READY_QPOS):
    """Build qpos/ctrl arrays for a ready keyframe in a compiled model."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    apply_arm_pose(mujoco, model, data, pose=pose, strict=True)
    apply_hand_pose(mujoco, model, data)
    return data.qpos.copy(), data.ctrl.copy()
