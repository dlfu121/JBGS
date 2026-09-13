#!/usr/bin/env python3
"""Native KeyCollect ACT grasp probe.

Runs the ACT checkpoint inside its own MuJoCo scene (the same loop as
scripts/infer_mujoco.py) and reports the screwdriver body world Z on every
control step, so a grasp lift can be detected deterministically without any
image inspection.

Exits 0 when the screwdriver stays above REST_Z + MARGIN for SUSTAIN sim
seconds, otherwise exits 2 after MAX_STEPS control steps.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from lerobot_robot_mujoco import MuJoCoRobot  # noqa: E402

import infer_mujoco as im  # noqa: E402

MARGIN = 0.06
SUSTAIN_SIM_SEC = 0.30
MAX_STEPS = int(os.environ.get("NATIVE_PROBE_MAX_STEPS", "1200"))
CHECKPOINT = Path(os.environ.get(
    "CHECKPOINT", "outputs/train/act_rm65_dexhand/checkpoints/last"))
DATASET_ROOT = Path(os.environ.get(
    "DATASET_ROOT", "data/rm65_dexhand_merged"))


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, default=CHECKPOINT)
    ap.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--config", type=Path,
                    default=ROOT / "config" / "record_mujoco.yaml")
    args = ap.parse_args()

    im.install_act_depth_adapter()
    pretrained = im.find_checkpoint(args.checkpoint)
    print("checkpoint:", pretrained, flush=True)
    policy, pre, post = im_helpers(args, pretrained)
    robot_cfg = im.load_robot_config(args.config)
    robot_cfg.control_fps = 30
    robot_cfg.randomize_screwdrivers = True
    robot_cfg.show_viewer = False
    robot_cfg.show_camera_panel = False
    robot = MuJoCoRobot(robot_cfg)
    robot.connect()

    model = robot._sim._model
    data = robot._sim._data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "screwdriver_red")
    if body_id < 0:
        print("no screwdriver_red body", flush=True)
        robot.disconnect()
        return 2
    rest_z = float(data.xpos[body_id][2])
    print("screwdriver rest z = %.3f m (margin %.2f)" % (rest_z, MARGIN), flush=True)

    cameras = ["table_camera", "wrist_overhead_camera"]
    gripper_joints = list(robot_cfg.gripper_joint_names)
    period = 1.0 / robot_cfg.control_fps
    next_deadline = time.monotonic()
    lift_start = None
    z_max = rest_z
    step = 0
    last_report = time.monotonic()
    lifted = False
    try:
        while step < MAX_STEPS:
            obs = robot.get_observation(camera_names=cameras)
            policy_obs = im.make_observation(obs, cameras)
            with torch.inference_mode():
                action_tensor = policy.select_action(pre(policy_obs))
            action_vector = post(action_tensor)[0].numpy()
            robot.send_action(im.action_to_dict(action_vector, gripper_joints))

            z = float(data.xpos[body_id][2])
            z_max = max(z_max, z)
            if z - rest_z >= MARGIN:
                if lift_start is None:
                    lift_start = float(data.time)
                    print("screwdriver started to lift at sim t=%.2f z=%.3f"
                          % (lift_start, z), flush=True)
                elif float(data.time) - lift_start >= SUSTAIN_SIM_SEC:
                    print("LIFTED at sim t=%.2f z=%.3f (above rest %.3f for %.2f s)"
                          % (float(data.time), z, rest_z, SUSTAIN_SIM_SEC), flush=True)
                    lifted = True
                    break
            else:
                lift_start = None

            step += 1
            now = time.monotonic()
            if now - last_report >= 3.0:
                last_report = now
                print("step=%d sim_t=%.2f z=%.3f (rest %.3f)"
                      % (step, float(data.time), z, rest_z), flush=True)
            next_deadline += period
            remaining = next_deadline - now
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_deadline = now
    finally:
        robot.disconnect()
    print("finished: steps=%d max_z=%.3f lifted=%s" % (step, z_max, lifted),
          flush=True)
    return 0 if lifted else 2


def im_helpers(args, pretrained):
    ds_meta = im.LeRobotDatasetMetadata("local/rm65_dexhand_merged",
                                        root=args.dataset_root)
    policy_cfg = im.PreTrainedConfig.from_pretrained(pretrained)
    policy_cfg.pretrained_path = str(pretrained)
    policy_cfg.device = args.device
    policy = im.make_policy(policy_cfg, ds_meta=ds_meta)
    policy.eval()
    pre, post = im.make_pre_post_processors(
        policy_cfg=policy.config, pretrained_path=str(pretrained),
        preprocessor_overrides={"device_processor": {"device": str(args.device)}})
    print("policy ready", flush=True)
    return policy, pre, post


if __name__ == "__main__":
    sys.exit(main())
