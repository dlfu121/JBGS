#!/usr/bin/env python3
"""Native KeyCollect ACT retry-grasp evaluator.

Runs the ACT checkpoint inside its own MuJoCo scene.  Each attempt starts from
the home keyframe with a freshly randomized screwdriver; if the tool is not
lifted within ``--steps-per-attempt`` control steps the robot is reset
(reset_simulation + policy.reset) and the next attempt begins.  The screwdriver
body world Z is read directly from MuJoCo so success does not depend on any
image inspection.

Exit codes: 0 if at least one attempt lifts the screwdriver, otherwise 2.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from lerobot_robot_mujoco import MuJoCoRobot  # noqa: E402

import infer_mujoco as im  # noqa: E402

LIFT_MARGIN = 0.06
SUSTAIN_SIM_SEC = 0.30


def build_policy(args, pretrained):
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
    return policy, pre, post


def run_attempt(robot, policy, pre, post, cameras, gripper_joints,
                steps_per_attempt, attempt_index, records, body_id):
    model = robot._sim._model
    data = robot._sim._data
    rest_z = float(data.xpos[body_id][2])
    print("[attempt %d] rest z = %.3f m" % (attempt_index, rest_z), flush=True)
    lift_start = None
    max_z = rest_z
    period = 1.0 / robot.config.control_fps
    next_deadline = time.monotonic()
    try:
        for step in range(1, steps_per_attempt + 1):
            obs = robot.get_observation(camera_names=cameras)
            policy_obs = im.make_observation(obs, cameras)
            with torch.inference_mode():
                action_tensor = policy.select_action(pre(policy_obs))
            action_vector = post(action_tensor)[0].numpy()
            robot.send_action(im.action_to_dict(action_vector, gripper_joints))

            z = float(data.xpos[body_id][2])
            max_z = max(max_z, z)
            if z - rest_z >= LIFT_MARGIN:
                if lift_start is None:
                    lift_start = float(data.time)
                elif float(data.time) - lift_start >= SUSTAIN_SIM_SEC:
                    print("[attempt %d] LIFTED sim_t=%.2f z=%.3f"
                          % (attempt_index, float(data.time), z), flush=True)
                    records.append({"attempt": attempt_index, "lifted": True,
                                    "steps": step, "rest_z": rest_z,
                                    "max_z": z, "sim_time": float(data.time)})
                    return True
            else:
                lift_start = None

            now = time.monotonic()
            next_deadline += period
            remaining = next_deadline - now
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_deadline = now
    finally:
        pass
    records.append({"attempt": attempt_index, "lifted": False,
                    "steps": steps_per_attempt, "rest_z": rest_z,
                    "max_z": max_z})
    print("[attempt %d] not lifted after %d steps (max_z=%.3f)"
          % (attempt_index, steps_per_attempt, max_z), flush=True)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path,
                    default=ROOT / "outputs/train/act_rm65_dexhand/checkpoints/last")
    ap.add_argument("--dataset-root", type=Path,
                    default=ROOT / "data/rm65_dexhand_merged")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--config", type=Path,
                    default=ROOT / "config" / "record_mujoco.yaml")
    ap.add_argument("--max-attempts", type=int, default=3)
    ap.add_argument("--steps-per-attempt", type=int, default=400)
    ap.add_argument("--report", type=Path, default=Path("/tmp/native_retry.json"))
    args = ap.parse_args()

    im.install_act_depth_adapter()
    pretrained = im.find_checkpoint(args.checkpoint)
    print("checkpoint:", pretrained, flush=True)
    policy, pre, post = build_policy(args, pretrained)

    robot_cfg = im.load_robot_config(args.config)
    robot_cfg.control_fps = 30
    robot_cfg.randomize_screwdrivers = True
    robot_cfg.show_viewer = False
    robot_cfg.show_camera_panel = False
    robot = MuJoCoRobot(robot_cfg)
    robot.connect()
    body_id = mujoco.mj_name2id(
        robot._sim._model, mujoco.mjtObj.mjOBJ_BODY, "screwdriver_red")
    if body_id < 0:
        print("no screwdriver_red body", flush=True)
        robot.disconnect()
        return 2

    cameras = ["table_camera", "wrist_overhead_camera"]
    gripper_joints = list(robot_cfg.gripper_joint_names)
    records = []
    lifted = False
    try:
        for attempt in range(1, args.max_attempts + 1):
            if attempt > 1:
                policy.reset()
                robot.reset_simulation()
                print("--- reset to home + re-randomized screwdriver ---",
                      flush=True)
            if run_attempt(robot, policy, pre, post, cameras, gripper_joints,
                           args.steps_per_attempt, attempt, records, body_id):
                lifted = True
                break
    finally:
        robot.disconnect()

    summary = {"checkpoint": str(pretrained), "lifted": lifted,
               "records": records}
    try:
        args.report.write_text(json.dumps(summary, indent=2))
        print("report -> %s" % args.report, flush=True)
    except OSError:
        print(json.dumps(summary, indent=2), flush=True)
    return 0 if lifted else 2


if __name__ == "__main__":
    sys.exit(main())
