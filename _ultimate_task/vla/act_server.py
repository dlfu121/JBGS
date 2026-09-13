#!/usr/bin/env python3
"""LeRobot ACT request/reply server for the worker_scene bridge.

The server runs in the Python 3.12 ``keycollect`` environment.  It receives a
length-prefixed pickle observation over a localhost TCP socket and returns one
26-D (Cartesian + hand delta) action.  It never imports ROS or MuJoCo from
worker_scene.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import torch
import pickle
import socket
import struct

from task_definition import ACT_CAMERAS

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--dataset-root", type=Path, required=True)
    ap.add_argument("--bind", default="127.0.0.1:5566")
    ap.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    args = ap.parse_args()
    pretrained = args.checkpoint / "checkpoints/040000/pretrained_model"
    if (args.checkpoint / "pretrained_model").exists():
        pretrained = args.checkpoint / "pretrained_model"
    if not pretrained.exists():
        raise FileNotFoundError(pretrained)
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    cfg = PreTrainedConfig.from_pretrained(pretrained)
    cfg.pretrained_path = str(pretrained)
    cfg.device = args.device
    meta = LeRobotDatasetMetadata("local/rm65_dexhand_merged", root=args.dataset_root)
    policy = make_policy(cfg, ds_meta=meta)
    policy.eval()
    pre, post = make_pre_post_processors(
        policy_cfg=policy.config, pretrained_path=str(pretrained),
        preprocessor_overrides={"device_processor": {"device": str(args.device)}})
    print(f"ACT server ready: {pretrained} on {args.device}", flush=True)
    host, port = args.bind.rsplit(":", 1)
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, int(port)))
    listener.listen(1)
    while True:
        sock, _ = listener.accept()
        try:
            _serve(policy, pre, post, sock)
        except (EOFError, ConnectionError, OSError):
            pass
        finally:
            try:
                sock.close()
            except OSError:
                pass


def _serve(policy, pre, post, sock):
    def recv_frame():
        header = sock.recv(8)
        if len(header) != 8:
            raise EOFError
        size = struct.unpack("!Q", header)[0]
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(min(1 << 20, size - len(data)))
            if not chunk:
                raise EOFError
            data.extend(chunk)
        return pickle.loads(data)

    def send_frame(value):
        data = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        sock.sendall(struct.pack("!Q", len(data)) + data)

    while True:
        payload = recv_frame()
        try:
            state = np.asarray(payload["state"], dtype=np.float32)
            if state.shape != (39,):
                raise ValueError(f"state shape {state.shape}, expected (39,)")
            obs = {"observation.state": torch.from_numpy(state)}
            for key in ACT_CAMERAS:
                frame = np.asarray(payload[key], dtype=np.uint8)
                if frame.shape != (480, 640, 3):
                    raise ValueError(f"{key} shape {frame.shape}")
                obs[f"observation.images.{key}"] = torch.from_numpy(
                    np.ascontiguousarray(frame).transpose(2, 0, 1)).float() / 255.0
            with torch.inference_mode():
                action = post(policy.select_action(pre(obs)))[0].cpu().numpy()
            action = np.asarray(action, dtype=np.float32).reshape(-1)
            if action.shape != (26,) or not np.all(np.isfinite(action)):
                raise ValueError(f"invalid action {action.shape}")
            # Keep the wire format independent of the NumPy version in the
            # Python 3.8 ROS process.  Pickling an ndarray from NumPy 2.x
            # references ``numpy._core``, which older NumPy releases do not
            # provide; plain Python floats are stable across environments.
            send_frame({"ok": True, "action": action.tolist()})
        except Exception as exc:
            send_frame({"ok": False, "error": f"{type(exc).__name__}: {exc}"})


if __name__ == "__main__":
    raise SystemExit(main())
