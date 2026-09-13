#!/usr/bin/env python3
"""Run a trained ACT checkpoint to control the RM65 + DexHand MuJoCo scene.

Reuses the same `MuJoCoRobot` plugin used for data collection, so observations
and actions match the training distribution exactly:

* `observation.state` = [6 arm pos, 6 arm vel, 20 finger pos, 7 ee_pose] (float32)
* RGB observations = float32 CHW in [0, 1]
* depth observations = float32 CHW in metres
* action = 26 delta commands (6 cartesian + 20 finger deltas)

Inference uses CUDA when requested and available. ACT caches an action chunk,
so it does not need to run a new network forward pass on every control step.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# MuJoCo chooses its OpenGL implementation at import time, so bootstrap these
# options before importing LeRobot or the local robot plugin.
_render_bootstrap = argparse.ArgumentParser(add_help=False)
_render_bootstrap.add_argument("--render-backend", choices=("egl", "glfw", "osmesa"))
_render_bootstrap.add_argument("--egl-device-id", type=int)
_render_args, _ = _render_bootstrap.parse_known_args()
if _render_args.render_backend is not None:
    os.environ["MUJOCO_GL"] = _render_args.render_backend
else:
    os.environ.setdefault("MUJOCO_GL", "egl")
if _render_args.egl_device_id is not None:
    os.environ["MUJOCO_EGL_DEVICE_ID"] = str(_render_args.egl_device_id)

import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors

from lerobot_robot_mujoco import MuJoCoRobot, MuJoCoRobotConfig
from lerobot.cameras import CameraConfig
from scripts.act_depth_adapter import install_act_depth_adapter


class _RenderCameraConfig(CameraConfig):
    """Minimal concrete camera config (MuJoCo only reads width/height/fps)."""

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
EE_POSE_NAMES = ("x", "y", "z", "qx", "qy", "qz", "qw")


def find_checkpoint(checkpoint: Path) -> Path:
    pretrained = Path(checkpoint) / "pretrained_model"
    if pretrained.exists():
        return pretrained
    steps = sorted(Path(checkpoint).glob("checkpoints/*/pretrained_model"))
    if not steps:
        raise FileNotFoundError(f"No checkpoint found under {checkpoint}")
    return steps[-1]


def load_robot_config(yaml_path: Path) -> MuJoCoRobotConfig:
    raw = yaml.safe_load(yaml_path.read_text())["robot"]
    raw["cameras"] = {
        name: _RenderCameraConfig(
            width=cfg.get("width"),
            height=cfg.get("height"),
            fps=cfg.get("fps"),
        )
        for name, cfg in raw["cameras"].items()
    }
    for drop in ("type", "id"):
        raw.pop(drop, None)
    return MuJoCoRobotConfig(**raw)


def build_state(obs: dict) -> np.ndarray:
    arm_pos = np.asarray([obs[f"{j}.pos"] for j in ARM_JOINTS], dtype=np.float32)
    arm_vel = np.asarray([obs[f"{j}.vel"] for j in ARM_JOINTS], dtype=np.float32)
    finger_pos = np.asarray(
        [obs[k] for k in obs if k.endswith(".pos") and k.startswith("r_f_")],
        dtype=np.float32,
    )
    ee_pose = np.asarray([obs[f"ee_pose.{n}"] for n in EE_POSE_NAMES], dtype=np.float32)
    return np.concatenate([arm_pos, arm_vel, finger_pos, ee_pose])


def img_to_float_chw(img: np.ndarray) -> torch.Tensor:
    array = np.ascontiguousarray(img)
    t = torch.from_numpy(array).float()
    if array.dtype == np.uint8:
        t = t / 255.0
    return t.permute(2, 0, 1).contiguous()


def make_observation(obs: dict, cameras: list[str]) -> dict:
    return {
        **{f"observation.images.{cam}": img_to_float_chw(obs[cam]) for cam in cameras},
        "observation.state": torch.from_numpy(build_state(obs)),
    }


def action_to_dict(action: np.ndarray, gripper_joints: list[str]) -> dict:
    delta = {
        "delta_x": float(action[0]),
        "delta_y": float(action[1]),
        "delta_z": float(action[2]),
        "delta_roll": float(action[3]),
        "delta_pitch": float(action[4]),
        "delta_yaw": float(action[5]),
    }
    for i, joint in enumerate(gripper_joints):
        delta[f"{joint}.delta"] = float(action[6 + i])
    return delta


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=ROOT / "outputs/train/act_rm65_dexhand/checkpoints/last",
        help="Checkpoint dir (containing pretrained_model/) or pretrained_model dir itself.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "record_mujoco.yaml",
        help="Robot config YAML (reuses the recording robot section).",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=ROOT / "data" / "rm65_dexhand_merged",
        help="Merged dataset root, used for policy feature shapes.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run before exiting (0 = until viewer close or Ctrl-C in headless mode).",
    )
    parser.add_argument(
        "--control-fps",
        type=int,
        default=None,
        help=(
            "Action execution rate. Defaults to the training dataset FPS; "
            "use an explicit value only when reproducing a known capture rate."
        ),
    )
    parser.add_argument(
        "--device",
        choices=("auto", "cuda", "cpu"),
        default="auto",
        help="Policy device; 'auto' selects CUDA when available.",
    )
    parser.add_argument(
        "--render-backend",
        choices=("egl", "glfw", "osmesa"),
        default=os.environ.get("MUJOCO_GL", "egl"),
        help="MuJoCo OpenGL backend: egl for headless GPU, glfw for a visible window, osmesa for CPU.",
    )
    parser.add_argument(
        "--egl-device-id",
        type=int,
        default=None,
        help="Optional EGL GPU index, set before MuJoCo is imported.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable the viewer and camera panel (recommended with EGL).",
    )
    parser.add_argument(
        "--require-gpu-rendering",
        action="store_true",
        help="Abort unless the actual OpenGL renderer is hardware accelerated.",
    )
    parser.add_argument(
        "--no-randomize",
        action="store_true",
        help="Keep screwdrivers at their authored pose instead of randomizing.",
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=None,
        help="Seed for screwdriver randomization.",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="If set, save the table camera frames as a video + PNG previews.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.control_fps is not None and args.control_fps <= 0:
        raise ValueError("--control-fps must be greater than zero.")
    if args.render_backend == "osmesa" and args.require_gpu_rendering:
        raise ValueError("OSMesa is a CPU renderer and cannot satisfy --require-gpu-rendering.")
    if args.render_backend == "egl" and not args.headless:
        raise ValueError("EGL inference must use --headless; use GLFW for a visible viewer.")
    install_act_depth_adapter()
    pretrained_dir = find_checkpoint(args.checkpoint)
    print(f"Using checkpoint: {pretrained_dir}")

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(
            "--device cuda was requested, but torch.cuda.is_available() is false."
        )
    device = (
        "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    )
    if device == "auto":
        device = "cpu"
        print("CUDA unavailable; running inference on CPU.")

    ds_meta = LeRobotDatasetMetadata(
        "local/rm65_dexhand_merged", root=args.dataset_root
    )
    policy_cfg = PreTrainedConfig.from_pretrained(pretrained_dir)
    policy_cfg.pretrained_path = str(pretrained_dir)
    policy_cfg.device = device
    policy = make_policy(policy_cfg, ds_meta=ds_meta)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=str(pretrained_dir),
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )
    print(f"Policy ready: {policy.config.type} on {device}")

    robot_cfg = load_robot_config(args.config)
    control_fps = args.control_fps if args.control_fps is not None else int(ds_meta.fps)
    if robot_cfg.control_fps != control_fps:
        source = "--control-fps" if args.control_fps is not None else "training metadata"
        print(
            f"Control rate override: config={robot_cfg.control_fps} Hz, "
            f"{source}={control_fps} Hz; using {control_fps} Hz."
        )
        robot_cfg.control_fps = control_fps
    if args.headless:
        robot_cfg.show_viewer = False
        robot_cfg.show_camera_panel = False
    if args.no_randomize:
        robot_cfg.randomize_screwdrivers = False
    robot = MuJoCoRobot(robot_cfg)
    robot.connect()
    cameras = [
        key.removeprefix("observation.images.")
        for key in policy.config.image_features
    ]
    missing_cameras = [camera for camera in cameras if camera not in robot.observation_features]
    if missing_cameras:
        raise ValueError(
            f"Checkpoint requires unavailable camera observations: {missing_cameras}"
        )
    if not cameras:
        raise ValueError("Checkpoint does not declare any camera observations.")

    render_camera_name = cameras[0].removesuffix("_depth")
    camera_cfg = robot_cfg.cameras[render_camera_name]
    robot.simulation.render_camera(
        render_camera_name,
        width=camera_cfg.width,
        height=camera_cfg.height,
    )
    gl_info = robot.simulation.get_opengl_info()
    print(
        "MuJoCo renderer: "
        f"backend={gl_info['backend']}, vendor={gl_info['vendor']}, "
        f"renderer={gl_info['renderer']}, hardware={gl_info['hardware_accelerated']}"
    )
    if args.require_gpu_rendering and not gl_info["hardware_accelerated"]:
        robot.disconnect()
        raise RuntimeError(
            "Hardware OpenGL rendering was required, but the active renderer is "
            f"{gl_info['renderer']!r}. Check the NVIDIA/EGL or GLX driver setup."
        )
    gripper_joints = list(robot_cfg.gripper_joint_names)
    period = 1.0 / max(1, robot_cfg.control_fps)
    print(f"Controlling {cameras} cameras at {robot_cfg.control_fps} Hz "
          f"({len(gripper_joints)} finger joints).")

    writer = None
    start = time.monotonic()
    next_deadline = start
    report_start = start
    report_step = 0
    overruns = 0
    step = 0
    try:
        while robot.is_connected:
            if robot_cfg.show_viewer and not robot.simulation.sync_viewer():
                break
            # Render only the observations consumed by this checkpoint. The
            # recording config may expose additional depth cameras that would
            # otherwise add a GPU render/readback on every control step.
            obs = robot.get_observation(camera_names=cameras)
            policy_obs = make_observation(obs, cameras)
            with torch.inference_mode():
                action_tensor = policy.select_action(preprocessor(policy_obs))
            action_vector = postprocessor(action_tensor)[0].numpy()
            robot.send_action(action_to_dict(action_vector, gripper_joints))

            if args.save_dir is not None:
                args.save_dir.mkdir(parents=True, exist_ok=True)
                frame = obs[cameras[0]]
                if writer is None:
                    import cv2
                    writer = cv2.VideoWriter(
                        str(args.save_dir / "infer.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        robot_cfg.control_fps,
                        (frame.shape[1], frame.shape[0]),
                    )
                writer.write(frame)

            step += 1
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break

            # Use an absolute deadline so observation, inference, physics, and
            # rendering time count toward the 1/fps period. Sleeping for a full
            # period here used to make the simulation run slower than real time.
            next_deadline += period
            now = time.monotonic()
            remaining = next_deadline - now
            if remaining > 0:
                time.sleep(remaining)
            else:
                overruns += 1
                # Do not burst through cached actions to catch up after a slow
                # ACT forward pass; resume periodic scheduling from now.
                next_deadline = now

            now = time.monotonic()
            report_elapsed = now - report_start
            if report_elapsed >= 5.0:
                report_steps = step - report_step
                print(
                    f"Control rate: {report_steps / report_elapsed:.1f} Hz "
                    f"(target {robot_cfg.control_fps} Hz, overruns {overruns})."
                )
                report_start = now
                report_step = step
                overruns = 0
    finally:
        if writer is not None:
            writer.release()
            print(f"Saved video: {args.save_dir / 'infer.mp4'}")
        robot.disconnect()

    elapsed = time.monotonic() - start
    actual_fps = step / elapsed if elapsed > 0 else 0.0
    print(
        f"Inference finished after {step} steps ({elapsed:.1f} s, "
        f"average {actual_fps:.1f} Hz)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
