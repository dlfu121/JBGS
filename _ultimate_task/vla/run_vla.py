#!/usr/bin/env python3.8
"""Launch the ARIAC VLA scene and navigate the robot to the table.

This is the VLA goal-publishing step used after
``./slam/navigation/run_nav_saved.sh --scene ariac --view``.  The shared launcher owns
MuJoCo, the saved map, the map->odom TF, and navigation; this script only
publishes the calculated table-approach goal.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

# Keep ROS 2 logs in a writable per-task directory.  Some graphical lab
# images mount $HOME read-only; rclpy would otherwise abort before the bridge
# has a chance to open its MuJoCo window ("Failed to acquire logging mutex").
_ROS_RUNTIME_DIR = Path("/tmp/ariac_vla_ros")
(_ROS_RUNTIME_DIR / "log").mkdir(parents=True, exist_ok=True)
os.environ["ROS_HOME"] = str(_ROS_RUNTIME_DIR)
os.environ["ROS_LOG_DIR"] = str(_ROS_RUNTIME_DIR / "log")

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from task_definition import (MAP_ORIGIN_WORLD, TASK_PROFILE,
                             yaw_degrees_to_quaternion_zw)


HERE = Path(__file__).resolve().parent
WORKER_SCENE = HERE.parents[1]
SCENE_XML = WORKER_SCENE / "model" / "robot" / "ariac_lab_with_robot_3d.xml"
RANDOMIZED_XML = WORKER_SCENE / "model" / "robot" / "ariac_lab_with_robot_3d_vla.xml"
BRIDGE = WORKER_SCENE / "slam" / "bridge" / "bridge_ariac.py"
NAV = WORKER_SCENE / "slam" / "navigation" / "nav_p2p.py"
SAVED_MAP = WORKER_SCENE / "maps" / "ariac" / "ariac_map_3d.pgm"
SAVED_MAP_YAML = WORKER_SCENE / "maps" / "ariac" / "ariac_map_3d.yaml"
ACTIVE_STATES = {"PLANNING", "FOLLOWING", "DYNAMIC_AVOID", "ALIGNING"}
FAILURE_STATES = {"UNREACHABLE", "NO_MAP", "NO_POSE", "STUCK"}
ARRIVAL_HOLD_SEC = 0.8

# After the grasp demo the robot waits POST_GRASP_HOLD_SEC, then drives to a
# standoff point in front of the worker and turns to face him.  The worker
# position is given in the RViz/map frame used by nav_p2p.
DEMO_DONE_PHASE = "DEMO_DONE"
DEMO_ATTACHED_PHASE = "DEMO_ATTACHED"
POST_GRASP_HOLD_SEC = 10.0
WORKER_MAP_XY = (9.6, -1.8)
WORKER_WORLD = (WORKER_MAP_XY[0] + MAP_ORIGIN_WORLD[0],
                WORKER_MAP_XY[1] + MAP_ORIGIN_WORLD[1])
WORKER_STANDOFF_M = 1.5

# Import the implementation next to ariac_lab.xml without requiring the VLA
# directory itself to be installed as a Python package.
SCENE_SCRIPTS = WORKER_SCENE / "model" / "scenes"
sys.path.insert(0, str(SCENE_SCRIPTS))
from randomize_ariac_grasp import (  # noqa: E402
    compute_robot_table_target,
    randomize_scene,
)


class TableApproachRunner(Node):
    """Publish one navigation goal and wait for an actual ARRIVED state."""

    def __init__(self, goal_world, timeout_sec):
        super().__init__("ariac_vla_table_runner")
        self.goal_world = tuple(float(value) for value in goal_world)
        self.timeout_sec = float(timeout_sec)
        self.goal_pub = self.create_publisher(PoseStamped, "/nav_goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.demo_pub = self.create_publisher(String, "/act/demo_grasp", 10)
        self.create_subscription(String, "/nav_status", self._on_status, 10)
        self.create_subscription(String, "/act/task_status", self._on_task_status, 10)
        self.status = None
        self.status_serial = 0
        self.demo_done = False
        self.attach_time = None

    def _publish_stop(self):
        self.stop_pub.publish(Twist())

    def trigger_demo(self):
        """Ask the bridge to run the fist/teleport/return demo."""
        deadline = time.monotonic() + 10.0
        while (rclpy.ok() and self.demo_pub.get_subscription_count() == 0
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.demo_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                "/act/demo_grasp 没有订阅者；请确认 bridge 以 --task vla 启动")
        msg = String()
        msg.data = "start"
        self.demo_pub.publish(msg)
        self.get_logger().info("已触发握拳吸附演示 (/act/demo_grasp=start)")

    def _on_status(self, msg):
        self.status = msg.data
        self.status_serial += 1
        self.get_logger().info("导航状态: %s" % self.status)

    def _on_task_status(self, msg):
        try:
            phase = json.loads(msg.data).get("phase")
        except (ValueError, TypeError):
            return
        if phase == DEMO_ATTACHED_PHASE and self.attach_time is None:
            self.attach_time = time.monotonic()
            self.get_logger().info("螺丝刀已吸附到机械手，开始计时")
        if phase == DEMO_DONE_PHASE and not self.demo_done:
            self.demo_done = True
            self.get_logger().info("抓取演示已完成 (phase=%s)" % phase)

    def wait_demo_done(self, timeout_sec=180.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self.demo_done and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.demo_done:
            self.get_logger().error("等待抓取演示完成超时（%.1f 秒）" % timeout_sec)
        return self.demo_done

    def worker_goal(self):
        """Return a standoff goal in front of the worker, facing him."""
        tx, ty = self.goal_world[0], self.goal_world[1]
        wx, wy = WORKER_WORLD
        dx, dy = wx - tx, wy - ty
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return wx, wy, 0.0
        ux, uy = dx / length, dy / length
        goal_x = wx - WORKER_STANDOFF_M * ux
        goal_y = wy - WORKER_STANDOFF_M * uy
        yaw_deg = math.degrees(math.atan2(uy, ux))
        return goal_x, goal_y, yaw_deg

    def _navigate(self, goal_world, timeout_sec, label):
        # RViz and the saved-map navigator can take several seconds to start
        # on a graphical machine; tolerate launching this terminal slightly
        # before nav_p2p has created its subscription.
        discovery_deadline = time.monotonic() + 30.0
        while (rclpy.ok() and self.goal_pub.get_subscription_count() == 0
               and time.monotonic() < discovery_deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().error("/nav_goal 没有订阅者，请确认 slam/navigation/nav_p2p.py 已启动")
            return False

        world_x, world_y, yaw_deg = goal_world
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

        baseline = self.status_serial
        self.goal_pub.publish(msg)
        self.get_logger().info(
            "前往%s目标: world=(%.3f, %.3f), map=(%.3f, %.3f), yaw=%.1f deg"
            % (label, world_x, world_y, map_x, map_y, yaw_deg))

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.status_serial <= baseline:
                continue
            if self.status == "ARRIVED":
                hold_deadline = time.monotonic() + ARRIVAL_HOLD_SEC
                while rclpy.ok() and time.monotonic() < hold_deadline:
                    self._publish_stop()
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().info("机器人已锁定%s目标并停车" % label)
                return True
            if self.status in FAILURE_STATES:
                self.get_logger().error("%s导航失败: %s" % (label, self.status))
                return False
        self.get_logger().error("%s导航超时（%.1f 秒）" % (label, timeout_sec))
        return False

    def execute(self):
        return self._navigate(self.goal_world, self.timeout_sec, "桌前")

    def execute_worker(self):
        return self._navigate(self.worker_goal(), self.timeout_sec, "工人")


def _start_processes(args, randomized_xml):
    """Start bridge and navigator, returning child processes for cleanup."""
    if args.external:
        return []
    env = os.environ.copy()
    env["SLAM_SCENE_NAME"] = "ariac"
    env["ARIAC_TASK_PROFILE"] = TASK_PROFILE
    env["MUJOCO_SCENE_XML"] = str(randomized_xml)
    if args.view:
        env["MUJOCO_GL"] = "glfw"

    bridge_args = [sys.executable, str(BRIDGE)]
    if args.view:
        bridge_args.append("--view")
    if args.dynamic_person:
        bridge_args.append("--dynamic-person")
    children = []
    try:
        children.append(
            subprocess.Popen(bridge_args, cwd=str(WORKER_SCENE), env=env))
        time.sleep(5.0)
        if children[0].poll() is not None:
            raise RuntimeError("ARIAC MuJoCo bridge 启动失败")

        # nav_p2p uses map coordinates while bridge publishes odom.  The
        # saved-map launch path (slam/navigation/run_nav_saved.sh) installs this same
        # identity transform; without it the navigator cannot obtain a pose
        # in the saved map and reports NO_POSE/NO_MAP.
        static_tf = subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "map", "odom",
             "--ros-args", "-p", "use_sim_time:=true"],
            cwd=str(WORKER_SCENE), env=env)
        children.append(static_tf)
        time.sleep(1.0)
        if static_tf.poll() is not None:
            raise RuntimeError("map->odom 静态 TF 启动失败")

        nav = subprocess.Popen(
            [sys.executable, str(NAV), "--use-saved", "--scene", "ariac"],
            cwd=str(WORKER_SCENE), env=env)
        children.append(nav)
        time.sleep(3.0)
        if nav.poll() is not None:
            raise RuntimeError("保存地图导航节点启动失败")
        return children
    except Exception:
        _stop_processes(children)
        raise


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=None,
                        help="兼容 --start-stack 的螺丝刀种子；标准流程请设置 VLA_SEED")
    parser.add_argument("--goal-timeout", type=float, default=180.0,
                        help="桌前导航超时时间（秒）")
    parser.add_argument("--start-stack", action="store_true",
                        help="兼容旧方式：由本脚本启动 bridge/nav（不推荐）")
    parser.add_argument("--no-view", dest="view", action="store_false",
                        help=argparse.SUPPRESS)
    parser.set_defaults(view=True)
    parser.add_argument("--dynamic-person", action="store_true",
                        help="仅兼容旧方式；推荐把该参数传给 run_nav_saved.sh")
    parser.add_argument("--external", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--exit-on-arrival", action="store_true",
                        help="到达桌前后退出；默认继续等待，Ctrl-C 退出")
    args = parser.parse_args()
    if args.goal_timeout <= 0.0:
        parser.error("--goal-timeout 必须为正数")
    missing_maps = [str(path) for path in (SAVED_MAP, SAVED_MAP_YAML)
                    if not path.is_file()]
    if missing_maps:
        parser.error(
            "找不到 ARIAC 保存地图，请先准备以下文件: %s" %
            ", ".join(missing_maps))

    # run_nav_saved.sh randomizes and loads the scene before this publisher is
    # started.  Do not rewrite the XML here: doing so would not affect the
    # already-running MuJoCo process.  --start-stack retains the old behavior.
    if args.start_stack:
        sample = randomize_scene(SCENE_XML, RANDOMIZED_XML, args.seed)
    else:
        sample = {"world_x": float("nan"), "world_y": float("nan"),
                  "world_z": float("nan"), "theta": float("nan"),
                  "screwdriver_yaw": 0.0}
    target = compute_robot_table_target()
    goal_world = (target["dog_x"], target["dog_y"],
                  math.degrees(target["dog_yaw"]))
    if args.start_stack:
        print("随机螺丝刀: world=(%.3f, %.3f, %.3f), 位置扇形角度=%+.2f deg, 工具yaw=%+.2f deg"
              % (sample["world_x"], sample["world_y"], sample["world_z"],
                 math.degrees(sample["theta"]),
                 math.degrees(sample["screwdriver_yaw"])))
    else:
        print("螺丝刀场景由 run_nav_saved.sh 启动时随机生成并加载")
    print("机器人桌前目标: world=(%.3f, %.3f), yaw=%+.1f deg"
          % goal_world)

    children = []
    node = None
    success = False
    try:
        children = _start_processes(args, RANDOMIZED_XML) if args.start_stack else []
        rclpy.init()
        node = TableApproachRunner(goal_world, args.goal_timeout)
        success = node.execute()
        if success:
            node.trigger_demo()
            success = node.wait_demo_done()
        if success:
            node.get_logger().info(
                "抓取演示完成，等待吸附后 %.0f 秒再离开桌面"
                % POST_GRASP_HOLD_SEC)
            if node.attach_time is None:
                hold_end = time.monotonic() + POST_GRASP_HOLD_SEC
            else:
                hold_end = node.attach_time + POST_GRASP_HOLD_SEC
            while rclpy.ok() and time.monotonic() < hold_end:
                node._publish_stop()
                rclpy.spin_once(node, timeout_sec=0.1)
            success = node.execute_worker()
        if success and not args.exit_on_arrival:
            node.get_logger().info("已面对工人；Ctrl-C 退出目标发布器")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        success = False
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_processes(children)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
