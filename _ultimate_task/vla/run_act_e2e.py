#!/usr/bin/env python3.8
"""One-shot ARIAC VLA end-to-end run.

Launches (in this order):
  1. the randomized VLA MuJoCo scene + 3D bridge (headless EGL by default);
  2. the map->odom static TF and the saved-map navigator (nav_p2p --use-saved);
  3. the KeyCollect ACT TCP server (Python 3.12 ``keycollect`` env);
  4. the ROS ACT client in --execute mode.

Then publishes the table-approach navigation goal and waits for ``ARRIVED``.
The ACT client starts inference automatically once the robot is parked (it
gates on /nav_status == ARRIVED).  The runner then watches /act/task_status
for the ``LIFTED`` phase -- set by the bridge when the screwdriver stays at
least ACT_SCREWDRIVER_LIFT_MARGIN above its resting height for
ACT_SCREWDRIVER_SUSTAIN_SEC -- which freezes the arm and ends ACT inference.

Exit codes: 0 success (screwdriver lifted), 1 setup/navigation failure,
2 grasp timeout without a lift.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# Isolate this automated run from any interactive sim/nav stack that may be
# running on the default ROS domain.  Override with ROS_DOMAIN_ID when desired.
os.environ.setdefault("ROS_DOMAIN_ID", "71")

# Keep ROS 2 logs writable and isolated from the interactive VLA launcher.
_ROS_RUNTIME_DIR = Path("/tmp/ariac_act_e2e_ros")
(_ROS_RUNTIME_DIR / "log").mkdir(parents=True, exist_ok=True)
os.environ["ROS_HOME"] = str(_ROS_RUNTIME_DIR)
os.environ["ROS_LOG_DIR"] = str(_ROS_RUNTIME_DIR / "log")

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from task_definition import (
    MAP_ORIGIN_WORLD,
    TASK_PROFILE,
    yaw_degrees_to_quaternion_zw,
)

HERE = Path(__file__).resolve().parent
WORKER_SCENE = HERE.parents[1]
KEYCOLLECT = WORKER_SCENE / "KeyCollect"
SCENE_XML = WORKER_SCENE / "model" / "robot" / "ariac_lab_with_robot_3d.xml"
RANDOMIZED_XML = WORKER_SCENE / "model" / "robot" / "ariac_lab_with_robot_3d_vla.xml"
BRIDGE = WORKER_SCENE / "slam" / "bridge" / "bridge_ariac.py"
NAV = WORKER_SCENE / "nav_p2p.py"
SAVED_MAP = WORKER_SCENE / "maps" / "ariac" / "ariac_map_3d.pgm"
SAVED_MAP_YAML = WORKER_SCENE / "maps" / "ariac" / "ariac_map_3d.yaml"

CHECKPOINT = (KEYCOLLECT / "outputs" / "train" / "act_rm65_dexhand"
              / "checkpoints" / "last")
DATASET_ROOT = KEYCOLLECT / "data" / "rm65_dexhand_merged"
SERVER_PY = Path("/home/ee304/miniforge3/envs/keycollect/bin/python")

NAV_FAILURE_STATES = {"UNREACHABLE", "NO_MAP", "NO_POSE", "STUCK"}

sys.path.insert(0, str(WORKER_SCENE / "model" / "scenes"))
from randomize_ariac_grasp import (  # noqa: E402
    compute_robot_table_target,
    randomize_scene,
)


class ActE2eRunner(Node):
    def __init__(self, goal_world, nav_timeout, grasp_timeout, trace=None):
        super().__init__("act_e2e_runner")
        self.goal_world = tuple(float(v) for v in goal_world)
        self.nav_timeout = float(nav_timeout)
        self.grasp_timeout = float(grasp_timeout)
        self.trace = trace
        if trace is not None:
            with open(str(trace), "w") as handle:
                handle.write("monotonic,sim_time,phase,screwdriver_z,ee_x,ee_y,"
                             "ee_z,arm_pos,act_enabled\n")
        self.goal_pub = self.create_publisher(PoseStamped, "/nav_goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/nav_status", self._on_nav, 10)
        self.create_subscription(String, "/act/task_status", self._on_status, 10)
        self.nav_status = None
        self.nav_serial = 0
        self.phase = None
        self.last_status = {}

    def _on_nav(self, msg):
        self.nav_status = msg.data
        self.nav_serial += 1

    def _on_status(self, msg):
        try:
            status = json.loads(msg.data)
            phase = status.get("phase")
            if phase != self.phase:
                self.get_logger().info("ACT 阶段变化: %s -> %s"
                                       % (self.phase, phase))
                self.phase = phase
            self.last_status = status
            if self.trace is not None:
                ee = status.get("ee_xyz") or [None, None, None]
                with open(str(self.trace), "a") as handle:
                    handle.write("%.3f,%.3f,%s,%s,%s,%s,%s,%s,%s\n" % (
                        time.monotonic(), status.get("time"),
                        status.get("phase"), status.get("screwdriver_z"),
                        ee[0], ee[1], ee[2], status.get("arm_pos"),
                        status.get("act_enabled")))
        except Exception as exc:
            self.get_logger().warning("bad task status: %s" % exc)

    def _publish_stop(self):
        self.stop_pub.publish(Twist())

    def _wait_subscriber(self, timeout_sec=30.0):
        deadline = time.monotonic() + timeout_sec
        while (rclpy.ok() and self.goal_pub.get_subscription_count() == 0
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.goal_pub.get_subscription_count() > 0

    def _navigate(self):
        if not self._wait_subscriber():
            self.get_logger().error("/nav_goal 无订阅者，nav_p2p 未就绪")
            return False
        world_x, world_y, yaw_deg = self.goal_world
        map_x = world_x - MAP_ORIGIN_WORLD[0]
        map_y = world_y - MAP_ORIGIN_WORLD[1]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = map_x
        msg.pose.position.y = map_y
        quat_z, quat_w = yaw_degrees_to_quaternion_zw(yaw_deg)
        msg.pose.orientation.z = quat_z
        msg.pose.orientation.w = quat_w
        baseline = self.nav_serial
        self.goal_pub.publish(msg)
        self.get_logger().info(
            "前往桌前: world=(%.3f, %.3f) map=(%.3f, %.3f) yaw=%+.1f deg"
            % (world_x, world_y, map_x, map_y, yaw_deg))
        deadline = time.monotonic() + self.nav_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.nav_serial <= baseline:
                continue
            if self.nav_status == "ARRIVED":
                for _ in range(int(0.8 / 0.05)):
                    self._publish_stop()
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().info("机器人已到达桌前并停车，等待 ACT 推理")
                return True
            if self.nav_status in NAV_FAILURE_STATES:
                self.get_logger().error("桌前导航失败: %s" % self.nav_status)
                return False
        self.get_logger().error("桌前导航超时（%.1f s）" % self.nav_timeout)
        return False

    def _wait_lift(self):
        deadline = time.monotonic() + self.grasp_timeout
        last_report = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.phase == "LIFTED":
                # Brief hold so the freeze state is observable on the topics.
                hold_end = time.monotonic() + 1.0
                while rclpy.ok() and time.monotonic() < hold_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                z = self.last_status.get("screwdriver_z")
                self.get_logger().info(
                    "成功：螺丝刀已脱离桌面 (screwdriver_z=%.3f m)，ACT 已结束并冻结机械臂"
                    % (float(z) if z is not None else float("nan")))
                return True
            if time.monotonic() - last_report >= 5.0:
                last_report = time.monotonic()
                z = self.last_status.get("screwdriver_z")
                rest = self.last_status.get("screwdriver_rest_z")
                self.get_logger().info(
                    "等待离桌中 phase=%s screwdriver_z=%s (rest=%s) ..."
                    % (self.phase, z, rest))
        self.get_logger().error(
            "ACT 抓取超时（%.1f s），最后一次 phase=%s"
            % (self.grasp_timeout, self.phase))
        return False


def _compat_meshes_ok():
    try:
        subprocess.check_call(
            [sys.executable, "model/scenes/build_ariac_compat_meshes.py",
             "--check"],
            cwd=str(WORKER_SCENE), stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        return True
    except subprocess.CalledProcessError:
        return False


def _kill_leftovers():
    """Terminate leftover worker-scene processes on this run's ROS domain only.

    Scanning each process environment prevents an interactive sim/nav stack on
    another ROS domain (e.g. the user's own viewers on domain 0) from being
    killed by an automated E2E run.
    """
    domain = os.environ.get("ROS_DOMAIN_ID", "71").encode()
    matched = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        env_path = "/proc/%s/environ" % entry
        try:
            with open(env_path, "rb") as handle:
                env = handle.read()
        except OSError:
            continue
        if domain not in env:
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as handle:
                cmdline = handle.read().decode(errors="replace")
        except OSError:
            continue
        if any(key in cmdline for key in (
                "slam/bridge/bridge_ariac.py", "bridge_ariac.py",
                "bridge_warehouse.py", "nav_p2p.py",
                "act_server.py", "act_bridge_client.py")):
            matched.append(int(entry))
    for pid in matched:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(2.0)


def _spawn_processes(args, randomized_xml):
    env = os.environ.copy()
    env["SLAM_SCENE_NAME"] = "ariac"
    env["ARIAC_TASK_PROFILE"] = TASK_PROFILE
    env["MUJOCO_SCENE_XML"] = str(randomized_xml)
    env["MUJOCO_GL"] = "glfw" if args.view else "egl"
    children = []
    try:
        bridge_args = [sys.executable, str(BRIDGE)]
        if args.view:
            bridge_args.append("--view")
        children.append(subprocess.Popen(
            bridge_args, cwd=str(WORKER_SCENE), env=env))
        time.sleep(5.0)
        if children[0].poll() is not None:
            raise RuntimeError("ARIAC MuJoCo bridge 启动失败")

        children.append(subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "map", "odom",
             "--ros-args", "-p", "use_sim_time:=true"],
            cwd=str(WORKER_SCENE), env=env))
        time.sleep(1.0)
        if children[1].poll() is not None:
            raise RuntimeError("map->odom 静态 TF 启动失败")

        children.append(subprocess.Popen(
            [sys.executable, str(NAV), "--use-saved", "--scene", "ariac"],
            cwd=str(WORKER_SCENE), env=env))
        time.sleep(3.0)
        if children[2].poll() is not None:
            raise RuntimeError("保存地图导航节点启动失败")

        server_log = _ROS_RUNTIME_DIR / "act_server.log"
        with open(str(server_log), "wb") as logf:
            children.append(subprocess.Popen(
                [str(SERVER_PY), str(HERE / "act_server.py"),
                 "--checkpoint", str(args.checkpoint.resolve()),
                 "--dataset-root", str(args.dataset_root.resolve()),
                 "--device", args.device],
                cwd=str(HERE), env=env, stdout=logf, stderr=logf))
        _wait_server_ready(children[3], server_log)

        client_log = _ROS_RUNTIME_DIR / "act_client.log"
        with open(str(client_log), "wb") as logf:
            client_args = [sys.executable, str(HERE / "act_bridge_client.py"),
                           "--endpoint", args.endpoint, "--execute"]
            if args.dump_dir is not None:
                client_args += ["--dump-dir", str(args.dump_dir.resolve())]
            children.append(subprocess.Popen(
                client_args, cwd=str(HERE), env=env, stdout=logf, stderr=logf))
        return children
    except Exception:
        _stop_processes(children)
        raise


def _wait_server_ready(process, log_path, timeout_sec=120.0):
    """Wait until the ACT server prints its ready line.

    A raw TCP connect cannot be used as a readiness probe: the server accepts a
    single connection and the probe would consume it and kill the process.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("ACT server 退出，见 %s" % log_path)
        try:
            text = Path(log_path).read_text(errors="replace")
        except OSError:
            text = ""
        if "ACT server ready" in text:
            return
        time.sleep(1.0)
    raise RuntimeError("ACT server 未在 %.0f 秒内就绪（见 %s）"
                       % (timeout_sec, log_path))


def _stop_processes(children):
    for process in reversed(children):
        if process.poll() is None:
            process.terminate()
    for process in reversed(children):
        if process.poll() is None:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()


def _print_log_tail(path, limit=25):
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        print("    (无日志 %s)" % path)
        return
    lines = path.read_text(errors="replace").splitlines()
    for line in lines[-limit:]:
        print("    | " + line)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path, default=CHECKPOINT,
                    help="ACT checkpoint dir（含 pretrained_model/）")
    ap.add_argument("--dataset-root", type=Path, default=DATASET_ROOT,
                    help="LeRobot dataset root (meta/)")
    ap.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    ap.add_argument("--endpoint", default="127.0.0.1:5566")
    ap.add_argument("--seed", type=int, default=None,
                    help="screwdriver 随机种子（默认系统随机）")
    ap.add_argument("--nav-timeout", type=float, default=240.0)
    ap.add_argument("--grasp-timeout", type=float, default=150.0)
    ap.add_argument("--view", action="store_true",
                    help="打开 MuJoCo 查看器（否则 EGL 无头渲染）")
    ap.add_argument("--trace-out", type=Path, default=None,
                    help="写入 /act/task_status 遥测 CSV（调试用）")
    ap.add_argument("--dump-dir", type=Path, default=None,
                    help="ACT client 保存首帧图像/39维状态/动作统计")
    args = ap.parse_args()
    if not args.checkpoint.exists():
        ap.error("找不到 checkpoint: %s" % args.checkpoint)
    if not (args.dataset_root / "meta").exists():
        ap.error("找不到 dataset meta: %s" % args.dataset_root)
    for path in (SAVED_MAP, SAVED_MAP_YAML):
        if not path.is_file():
            ap.error("缺少保存地图: %s" % path)

    # Refresh the composed base scene exactly like run_nav_saved.sh does.
    if not _compat_meshes_ok():
        subprocess.check_call(
            [sys.executable, "model/scenes/build_ariac_compat_meshes.py"],
            cwd=str(WORKER_SCENE))
    subprocess.check_call(
        [sys.executable, "model/robot/gen_ariac_robot.py", "--start-z", "-0.08"],
        cwd=str(WORKER_SCENE))

    sample = randomize_scene(SCENE_XML, RANDOMIZED_XML, args.seed)
    print("螺丝刀随机采样: seed=%d world=(%.3f, %.3f, %.3f) tool_yaw=%+.2f deg"
          % (int(sample["seed"]), sample["world_x"], sample["world_y"],
             sample["world_z"], math.degrees(sample["screwdriver_yaw"])))

    target = compute_robot_table_target()
    goal_world = (target["dog_x"], target["dog_y"],
                  math.degrees(target["dog_yaw"]))
    print("机器人桌前目标: world=(%.3f, %.3f) yaw=%+.1f deg" % goal_world)
    print("螺丝刀静止参考高度 %.3f m（桌面 %.3f m）"
          % (sample["world_z"], target["tabletop_z"]))

    _kill_leftovers()
    children = []
    node = None
    success = False
    try:
        children = _spawn_processes(args, RANDOMIZED_XML)
        rclpy.init()
        node = ActE2eRunner(goal_world, args.nav_timeout, args.grasp_timeout,
                            trace=args.trace_out)
        if node._navigate():
            success = node._wait_lift()
    except KeyboardInterrupt:
        print("用户中断")
    except Exception as exc:
        print("E2E 运行异常: %s" % exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_processes(children)
        print("---- ACT server 日志尾部 ----")
        _print_log_tail(_ROS_RUNTIME_DIR / "act_server.log")
        print("---- ACT client 日志尾部 ----")
        _print_log_tail(_ROS_RUNTIME_DIR / "act_client.log")
    if success:
        print("\n结果: PASS - 螺丝刀已脱离桌面，ACT 闭环控制链完整跑通")
        return 0
    print("\n结果: FAIL")
    return 2


if __name__ == "__main__":
    sys.exit(main())
