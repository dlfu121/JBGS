#!/usr/bin/env python3.8
"""ROS 2 observation client and ACT action publisher for worker_scene."""
import argparse
import json
import threading
import time
from pathlib import Path
import numpy as np
import pickle
import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String

from task_definition import TABLE_CAMERA, WRIST_CAMERA


class ActClient(Node):
    def __init__(self, endpoint, execute=False, dump_dir=None,
                 show_cameras=None):
        super().__init__("worker_scene_act_client")
        self.execute = execute
        self.endpoint = endpoint
        self.dump_dir = dump_dir
        self.show_cameras = execute if show_cameras is None else show_cameras
        self._camera_view_failed = False
        self._cv2 = None
        self.dumped = False
        self._state_written = False
        self._act_buffer = []
        self.state = None
        self.table = None
        self.wrist = None
        self.nav_arrived = False
        self.lift_done = False
        self._reported_start = False
        self.lock = threading.Lock()
        # ``str.removeprefix`` was added in Python 3.9; this client is
        # intentionally runnable in the ROS Foxy Python 3.8 environment.
        if endpoint.startswith("tcp://"):
            endpoint = endpoint[len("tcp://"):]
        host, port = endpoint.rsplit(":", 1)
        self.sock = socket.create_connection((host, int(port)), timeout=3.0)
        self.sock.settimeout(3.0)
        self.pub = self.create_publisher(Float64MultiArray,
                                          "/act/cartesian_action", 2)
        self.create_subscription(String, "/act/observation", self.on_state, 2)
        self.create_subscription(Image, "/act/table_image", self.on_table, 2)
        self.create_subscription(Image, "/act/wrist_image", self.on_wrist, 2)
        self.create_subscription(String, "/act/task_status", self.on_task_status, 2)
        self.create_subscription(String, "/nav_status", self.on_nav_status, 2)
        # Match KeyCollect's record/inference control_fps=24.
        # The ACT checkpoint was trained at 30 Hz and its actions are per-frame
        # increments, so request/apply them at 30 Hz to reproduce training speed.
        self.create_timer(1.0 / 30.0, self.step)

    def on_task_status(self, msg):
        try:
            status = json.loads(msg.data)
            if status.get("phase") == "LIFTED":
                with self.lock:
                    if not self.lift_done:
                        self.lift_done = True
                        self.get_logger().info(
                            "螺丝刀已脱离桌面，结束 ACT 推理并保持机械臂")
        except Exception as exc:
            self.get_logger().warning("bad ACT task status: %s" % exc)

    def on_nav_status(self, msg):
        # Do not let policy actions move the arm while the base is navigating.
        # The client may start before navigation and waits for the definitive
        # ARRIVED state published by nav_p2p.
        with self.lock:
            self.nav_arrived = (msg.data == "ARRIVED")

    def on_state(self, msg):
        try:
            data = json.loads(msg.data)
            # state order matches KeyCollect: arm pos, arm vel, hand pos, ee pose
            state = data["arm_pos"] + data["arm_vel"] + data["hand_pos"] + data["ee_pose"]
            if len(state) == 39:
                with self.lock:
                    self.state = np.asarray(state, dtype=np.float32)
                    if self.dump_dir is not None and not self._state_written:
                        self._state_written = True
                        self.dump_dir.mkdir(parents=True, exist_ok=True)
                        (self.dump_dir / "state_39.json").write_text(json.dumps(
                            {"state": [float(v) for v in state]}))
        except Exception as exc:
            self.get_logger().warning("bad ACT state: %s" % exc)

    def _dump_images_once(self):
        if self.dump_dir is None or self.dumped:
            return
        try:
            self.dump_dir.mkdir(parents=True, exist_ok=True)
            import cv2
            for name, frame in (("table", self.table), ("wrist", self.wrist)):
                cv2.imwrite(str(self.dump_dir / ("%s.jpg" % name)),
                            cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            self.dumped = True
            self.get_logger().info("已保存首帧观测到 %s" % self.dump_dir)
        except Exception as exc:
            self.get_logger().warning("dump images failed: %s" % exc)

    def image(self, msg):
        if msg.encoding != "rgb8" or msg.height != 480 or msg.width != 640:
            return None
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(480, 640, 3).copy()

    def on_table(self, msg):
        frame = self.image(msg)
        with self.lock:
            self.table = frame
        self._show_camera_views()

    def on_wrist(self, msg):
        frame = self.image(msg)
        with self.lock:
            self.wrist = frame
        self._show_camera_views()

    def _show_camera_views(self):
        if not self.show_cameras or self._camera_view_failed:
            return
        with self.lock:
            table = None if self.table is None else self.table.copy()
            wrist = None if self.wrist is None else self.wrist.copy()
        if table is None and wrist is None:
            return
        try:
            if self._cv2 is None:
                import cv2
                self._cv2 = cv2
                cv2.namedWindow("ACT table camera", cv2.WINDOW_NORMAL)
                cv2.namedWindow("ACT wrist camera", cv2.WINDOW_NORMAL)
            if table is not None:
                self._cv2.imshow(
                    "ACT table camera",
                    self._cv2.cvtColor(table, self._cv2.COLOR_RGB2BGR))
            if wrist is not None:
                self._cv2.imshow(
                    "ACT wrist camera",
                    self._cv2.cvtColor(wrist, self._cv2.COLOR_RGB2BGR))
            self._cv2.waitKey(1)
        except Exception as exc:
            self._camera_view_failed = True
            self.get_logger().warning("自动相机窗口打开失败: %s" % exc)

    def close_camera_views(self):
        if self._cv2 is not None:
            try:
                self._cv2.destroyWindow("ACT table camera")
                self._cv2.destroyWindow("ACT wrist camera")
            except Exception:
                pass

    def step(self):
        with self.lock:
            if (self.lift_done or not self.nav_arrived or self.state is None
                    or self.table is None or self.wrist is None):
                return
            payload = {"state": self.state.copy(),
                       TABLE_CAMERA: self.table.copy(),
                       WRIST_CAMERA: self.wrist.copy()}
            executing = self.execute and not self._reported_start
            self._dump_images_once()
        try:
            data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
            self.sock.sendall(struct.pack("!Q", len(data)) + data)
            header = self.sock.recv(8)
            if len(header) != 8:
                return
            size = struct.unpack("!Q", header)[0]
            data = bytearray()
            while len(data) < size:
                data.extend(self.sock.recv(min(1 << 20, size - len(data))))
            reply = pickle.loads(data)
            if not reply.get("ok"):
                self.get_logger().error(reply.get("error", "ACT failure"))
                return
            action = np.asarray(reply["action"], dtype=np.float64)
            if action.shape != (26,) or not np.all(np.isfinite(action)):
                self.get_logger().error("ACT reply requires 26 finite values")
                return
            if not self.execute:
                action[:] = 0.0
            msg = Float64MultiArray()
            msg.data = action.tolist()
            self.pub.publish(msg)
            if self.dump_dir is not None and self.execute:
                self._act_buffer.append(action[:6])
                if len(self._act_buffer) >= 100:
                    arr = np.asarray(self._act_buffer, dtype=np.float64)
                    stats = {
                        "n": int(len(self._act_buffer)),
                        "trans_norm_mean": float(np.linalg.norm(
                            arr[:, :3], axis=1).mean()),
                        "rot_norm_mean": float(np.linalg.norm(
                            arr[:, 3:6], axis=1).mean()),
                        "hand_max_abs": float(np.abs(arr).max()),
                        "sample": arr[-1].tolist(),
                    }
                    try:
                        self.dump_dir.mkdir(parents=True, exist_ok=True)
                        (self.dump_dir / "action_stats.json").write_text(
                            json.dumps(stats))
                    except Exception:
                        pass
                    del self._act_buffer[:]
            if executing:
                self._reported_start = True
                self.get_logger().info("启动 ACT 推理：机器人已就位于桌前并开始执行动作")
        except Exception as exc:
            self.get_logger().error("ACT request failed: %s" % exc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="127.0.0.1:5566",
                    help="ACT TCP endpoint HOST:PORT")
    ap.add_argument("--execute", action="store_true",
                    help="publish real actions; default is zero-action dry run")
    ap.add_argument("--dump-dir", default=None,
                    help="save first table/wrist frames, 39-D state and ACT "
                         "action statistics for debugging")
    ap.add_argument("--no-camera-view", action="store_true",
                    help="do not auto-open the two ACT camera windows")
    args = ap.parse_args()
    rclpy.init()
    node = ActClient(args.endpoint, args.execute,
                     dump_dir=(Path(args.dump_dir) if args.dump_dir else None),
                     show_cameras=(False if args.no_camera_view else None))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_camera_views()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
