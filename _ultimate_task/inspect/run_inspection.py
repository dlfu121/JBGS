#!/usr/bin/env python3.8
"""Execute the ARIAC cabinet -> tank -> hydrant inspection route."""

import argparse
import json
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float64MultiArray, Int64, String

from task_definition import (CAMERA as INSPECTION_CAMERA,
                             yaw_degrees_to_quaternion_zw)


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROUTE = os.path.join(HERE, "inspection_route.json")
ACTIVE_STATES = {"PLANNING", "FOLLOWING", "DYNAMIC_AVOID", "ALIGNING"}
FAILURE_STATES = {"UNREACHABLE", "NO_MAP", "NO_POSE", "STUCK"}
CAPTURE_TIMEOUT_SEC = 15.0
# The preview renderer can substantially reduce wall-clock callback frequency.
# Allow the arm to complete its smooth trajectory even on a slow workstation.
ARM_ARRIVAL_TIMEOUT_SEC = 60.0
ARM_TARGET_MATCH_TOL = 1e-4
PREVIEW_WARMUP_SEC = 0.5
# Keep the base stopped briefly after ARRIVED so a late callback from the
# dynamic safety layer cannot roll it away from the inspection pose.
ARRIVAL_HOLD_SEC = 0.8


class InspectionRunner(Node):
    def __init__(self, route, navigation_only=False, dynamic_person=False,
                 pedestrian_seed=0):
        super().__init__("ariac_inspection_runner")
        self.route = route
        self.navigation_only = navigation_only
        self.dynamic_person = bool(dynamic_person)
        self.pedestrian_seed = int(pedestrian_seed)
        self.status = None
        self.status_serial = 0
        self.dynamic_avoid_seen = False
        self.capture_results = {}
        self.arm_status = None
        self.hold_still = False
        self.goal_pub = self.create_publisher(PoseStamped, "/nav_goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, "/inspection/left_arm_pose", 10)
        self.preview_pub = self.create_publisher(
            String, "/inspection/camera_preview", 10)
        self.capture_pub = self.create_publisher(
            String, "/inspection/capture", 10)
        person_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.person_pub = self.create_publisher(
            Bool, "/inspection/dynamic_person", person_qos)
        self.person_seed_pub = self.create_publisher(
            Int64, "/inspection/dynamic_person_seed", person_qos)
        self.create_subscription(String, "/nav_status", self._on_status, 10)
        self.create_subscription(
            String, "/inspection/capture_done", self._on_capture_done, 10)
        self.create_subscription(
            String, "/inspection/left_arm_status", self._on_arm_status, 10)
        self._publish_dynamic_person()

    def _publish_dynamic_person(self):
        seed = Int64()
        seed.data = self.pedestrian_seed
        self.person_seed_pub.publish(seed)
        msg = Bool()
        msg.data = self.dynamic_person
        self.person_pub.publish(msg)

    def _check_dynamic_person_consumer(self):
        if not self.dynamic_person:
            return True
        deadline = time.monotonic() + 3.0
        while (rclpy.ok()
               and (self.person_pub.get_subscription_count() == 0
                    or self.person_seed_pub.get_subscription_count() == 0)
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if (self.person_pub.get_subscription_count() == 0
                or self.person_seed_pub.get_subscription_count() == 0):
            self.get_logger().error(
                "动态行人控制没有订阅者；请使用当前 bridge_ariac.py 启动仿真")
            return False
        self._publish_dynamic_person()
        self.get_logger().info(
            "已开启动态行人测试：储罐前及长通道的 2 个独立行人，"
            "机械狗接近后沿固定轨迹持续行走且不让行；"
            "第一条巡检路线还放置了一个未标定箱子，"
            "机械狗根据左右通行空间主动规划绕行")
        return True

    def _on_status(self, msg):
        self.status = msg.data
        self.status_serial += 1
        if self.status == "DYNAMIC_AVOID" and not self.dynamic_avoid_seen:
            self.dynamic_avoid_seen = True
            self.get_logger().info(
                "已触发动态避障：检测到行人或陌生障碍，"
                "正在规划空旷一侧的局部绕行路线")
        self.get_logger().info("导航状态: %s" % self.status)

    def _on_capture_done(self, msg):
        try:
            result = json.loads(msg.data)
            self.capture_results[str(result["stop_id"])] = result
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning("忽略无效拍摄回执: %s" % exc)

    def _on_arm_status(self, msg):
        try:
            status = json.loads(msg.data)
            if status.get("state") == "arrived" and len(status["target"]) == 6:
                self.arm_status = status
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning("忽略无效机械臂状态: %s" % exc)

    def _publish_stop(self):
        if self.hold_still:
            self.stop_pub.publish(Twist())

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_stop()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _check_action_consumers(self):
        if self.navigation_only:
            return True
        discovery_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            if (self.arm_pub.get_subscription_count() > 0
                    and self.capture_pub.get_subscription_count() > 0
                    and self.preview_pub.get_subscription_count() > 0
                    and self.count_publishers(
                        "/inspection/left_arm_status") > 0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            "机械臂、到位反馈或拍摄接口尚未接入；请确认 ARIAC bridge 已启动，"
            "且 slam/navigation/nav_p2p.py 已启动（推荐先运行 ./slam/navigation/run_nav_saved.sh --scene ariac --view），"
            "或先使用 --navigation-only")
        return False

    def _check_navigation_consumers(self):
        """Verify that nav_p2p and the live lidar bridge are really online.

        nav_p2p deliberately publishes zero velocity when its point-cloud
        watchdog expires.  Checking only ``/nav_goal`` therefore gives a
        false sense that navigation is ready while the robot remains still.
        Do both checks before publishing the first goal and report the exact
        missing endpoint to the user.
        """
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            goal_ready = self.goal_pub.get_subscription_count() > 0
            cloud_ready = self.count_publishers("/pointcloud") > 0
            if goal_ready and cloud_ready:
                self.get_logger().info(
                    "导航接口就绪：/nav_goal 已连接，/pointcloud bridge 在线")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().error(
                "/nav_goal 没有订阅者：请先启动 ./slam/navigation/run_nav_saved.sh "
                "--scene ariac（该脚本会启动 slam/navigation/nav_p2p.py）")
        if self.count_publishers("/pointcloud") == 0:
            self.get_logger().error(
                "/pointcloud 没有发布者：bridge 未启动或使用了 --no-lidar；"
                "nav_p2p 会因此持续停车")
        return False

    def _publish_goal(self, stop, constrain_yaw=True):
        discovery_deadline = time.monotonic() + 3.0
        while (rclpy.ok() and self.goal_pub.get_subscription_count() == 0
               and time.monotonic() < discovery_deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().error("/nav_goal 没有订阅者，请先启动 slam/navigation/nav_p2p.py")
            return False

        x, y, yaw_deg = stop["pose_map"]
        # Small lateral correction at the cabinet keeps the left wrist clear
        # of the cabinet side rail while preserving the calibrated pose used
        # by the route geometry checks.
        offset = stop.get("nav_offset_map", (0.0, 0.0))
        x += float(offset[0]); y += float(offset[1])
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        if constrain_yaw:
            quat_z, quat_w = yaw_degrees_to_quaternion_zw(yaw_deg)
            msg.pose.orientation.z = quat_z
            msg.pose.orientation.w = quat_w
        else:
            # A zero-norm quaternion means “position only” to nav_p2p.  This
            # is the final safety fallback when localization keeps oscillating
            # around the requested heading: parking is more important than
            # endlessly rotating in place at an otherwise reached stop.
            msg.pose.orientation.w = 0.0
        self.goal_pub.publish(msg)
        return True

    def _navigate(self, stop, timeout_sec):
        """Follow one static route goal and positively lock the arrival.

        nav_p2p owns global static-map planning and invokes dynamic avoidance
        only as a transient safety layer.  Do not resend the goal while it is
        active: receiving a replacement goal clears the current path.  An
        earlier version also required seeing an ACTIVE state before ARRIVED;
        that races when the robot starts inside the goal tolerance or when
        intermediate states are published between executor spins.
        """
        self.hold_still = False
        baseline = self.status_serial
        if not self._publish_goal(stop):
            return False
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.status_serial <= baseline:
                continue
            # Accept the first ARRIVED transition belonging to this goal.
            # PLANNING/FOLLOWING (and even DYNAMIC_AVOID) may be skipped.
            if self.status == "ARRIVED":
                self.hold_still = True
                hold_deadline = time.monotonic() + ARRIVAL_HOLD_SEC
                while rclpy.ok() and time.monotonic() < hold_deadline:
                    self._publish_stop()
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().info(
                    "%s 已锁定目标点并停车（静态路径完成，动态避障已释放）"
                    % stop["id"])
                return True
            if self.status == "ALIGNING":
                # Keep the yaw-constrained goal active.  nav_p2p publishes
                # ALIGNING while rotating in place and ARRIVED only after the
                # requested heading is reached; replacing the goal with a
                # zero-norm quaternion here would cancel the rotation.
                self.get_logger().info(
                    "%s 已到达位置，正在旋转至目标朝向" % stop["id"])
            if self.status in FAILURE_STATES:
                self.get_logger().error(
                    "%s 导航失败: %s" % (stop["id"], self.status))
                return False
        self.get_logger().error("%s 导航超时" % stop["id"])
        return False

    @staticmethod
    def _same_arm_target(actual, expected):
        return (len(actual) == len(expected)
                and all(abs(float(a) - float(b)) <= ARM_TARGET_MATCH_TOL
                        for a, b in zip(actual, expected)))

    def _move_arm_and_wait(self, target, label, timeout_sec):
        target = [float(value) for value in target]
        self.arm_status = None
        arm = Float64MultiArray()
        arm.data = target
        self.arm_pub.publish(arm)
        self.get_logger().info("左臂开始移动到%s；机器狗保持静止" % label)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_stop()
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.arm_status
            if (status is not None
                    and self._same_arm_target(status.get("target", ()), target)):
                self.get_logger().info(
                    "左臂已实际到达%s，最大关节误差 %.4f rad"
                    % (label, float(status.get("max_error", 0.0))))
                return True
        self.get_logger().error("左臂移动到%s超时，禁止拍摄" % label)
        return False

    def _set_preview(self, active):
        command = String()
        command.data = "open" if active else "close"
        self.preview_pub.publish(command)

    def _inspect(self, stop):
        if self.navigation_only:
            self._spin_for(stop["dwell_sec"])
            return True

        self._set_preview(True)
        # Let the preview renderer create its context and display a first frame
        # before commanding the arm. This makes the small window visibly stay
        # open throughout the lift instead of appearing only at capture time.
        self._spin_for(PREVIEW_WARMUP_SEC)
        if not self._move_arm_and_wait(
                stop["left_arm_pose"], "%s 拍摄姿态" % stop["id"],
                ARM_ARRIVAL_TIMEOUT_SEC):
            return False

        dwell_sec = max(3.0, float(stop.get("dwell_sec", 0.0)),
                        float(stop.get("arm_settle_sec", 0.0)))
        # Allow one extra optical settle interval after the bridge reports the
        # final joint sample.  This prevents a stale cabinet preview frame
        # from being mistaken for the actual capture moment.
        self._spin_for(0.6)
        self.get_logger().info(
            "%s 已到位；机器狗与机械臂保持静止 %.1f 秒后拍摄"
            % (stop["id"], dwell_sec))
        self._spin_for(dwell_sec)

        self.capture_results.pop(stop["id"], None)
        trigger = String()
        trigger.data = json.dumps({
            "stop_id": stop["id"],
            "target": stop["target"],
            "camera": self.route["camera"],
            "gauge_world": stop["gauge_world"],
        }, separators=(",", ":"))
        self.capture_pub.publish(trigger)
        self.get_logger().info("已触发 %s 拍摄，等待保存回执" % stop["id"])
        deadline = time.monotonic() + CAPTURE_TIMEOUT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_stop()
            rclpy.spin_once(self, timeout_sec=0.1)
            result = self.capture_results.get(stop["id"])
            if result is None:
                continue
            if not result.get("success"):
                self.get_logger().error(
                    "%s 拍摄失败: %s"
                    % (stop["id"], result.get("error", "unknown error")))
                return False
            self.get_logger().info(
                "%s 拍摄完成: %s" % (stop["id"], result.get("path", "")))
            return True
        self.get_logger().error("%s 等待拍摄回执超时" % stop["id"])
        return False

    def _return_arm_to_travel_pose(self):
        if self.navigation_only:
            return True
        returned = self._move_arm_and_wait(
            self.route["travel_arm_pose"], "统一行走姿态",
            ARM_ARRIVAL_TIMEOUT_SEC)
        self._set_preview(False)
        if returned:
            self.get_logger().info("左臂已归位，相机预览已关闭")
        return returned

    def _return_to_start(self, timeout_sec):
        """完成最后一个巡检点后返回路线配置的出发位姿。"""
        pose = self.route.get("return_pose_map", (0.0, 0.0, 0.0))
        start = {"id": "return_to_start", "pose_map": pose}
        if not self._navigate(start, timeout_sec):
            self.get_logger().error("返回出发点失败")
            return False
        self.get_logger().info(
            "已返回出发点: map=(%.3f, %.3f), yaw=%.1f deg"
            % tuple(float(value) for value in pose))
        return True

    def execute(self, timeout_sec):
        self._publish_dynamic_person()
        if not self._check_dynamic_person_consumer():
            return False
        if not self._check_navigation_consumers():
            return False
        if not self._check_action_consumers():
            return False
        self.get_logger().info(
            "开始 ARIAC 巡检，共 %d 个点" % len(self.route["stops"]))
        for index, stop in enumerate(self.route["stops"], 1):
            self.get_logger().info(
                "[%d/%d] 前往 %s: map=(%.3f, %.3f), yaw=%.1f deg"
                % ((index, len(self.route["stops"]), stop["id"])
                   + tuple(stop["pose_map"])))
            if not self._navigate(stop, timeout_sec):
                return False
            if not self._inspect(stop):
                self._return_arm_to_travel_pose()
                return False
            if not self._return_arm_to_travel_pose():
                return False
        if self.route.get("return_to_start", False):
            if not self._return_to_start(timeout_sec):
                return False
        if self.dynamic_person:
            if self.dynamic_avoid_seen:
                self.get_logger().info("全部巡检点执行完成；动态避障演示已实际触发")
            else:
                self.get_logger().warning(
                    "全部巡检点执行完成，但未观察到 DYNAMIC_AVOID；"
                    "请确认 /pointcloud 在线且行人模型已加载")
        else:
            self.get_logger().info("全部巡检点执行完成")
        return True

    def stop_and_close(self):
        """Leave the base stopped and never strand the preview after exit."""
        self.hold_still = True
        for _ in range(3):
            self._publish_stop()
            rclpy.spin_once(self, timeout_sec=0.03)
        if not self.navigation_only:
            self._set_preview(False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", default=DEFAULT_ROUTE)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    parser.add_argument(
        "--navigation-only", action="store_true",
        help="只验证路线和停留，不发布机械臂/拍摄动作")
    person_group = parser.add_mutually_exclusive_group()
    person_group.add_argument(
        "--dynamic-person", action="store_true",
        help=("开启 2 个动态行人（默认开启）；接近后独立行走且不让行，"
              "在储罐前和长通道演示主动绕行"))
    person_group.add_argument(
        "--no-dynamic-person", action="store_true",
        help="关闭动态行人，仅测试静态障碍导航")
    parser.add_argument(
        "--pedestrian-seed", type=int,
        help="兼容旧启动脚本的参数；当前行人使用固定轨迹，不产生随机运动")
    args = parser.parse_args()
    with open(args.route, encoding="utf-8") as stream:
        route = json.load(stream)
    route.setdefault("camera", INSPECTION_CAMERA)
    if route.get("scene") != "ariac" or not route.get("stops"):
        parser.error("路线必须是包含 stops 的 ARIAC 配置")
    if len(route.get("travel_arm_pose", ())) != 6:
        parser.error("travel_arm_pose 必须有 6 个关节值")
    if float(route.get("travel_arm_settle_sec", 0.0)) <= 0.0:
        parser.error("travel_arm_settle_sec 必须为正数")
    if route.get("return_to_start", False):
        return_pose = route.get("return_pose_map", (0.0, 0.0, 0.0))
        if len(return_pose) != 3:
            parser.error("return_pose_map 必须有 3 个数 (x, y, yaw_deg)")
    for index, stop in enumerate(route["stops"], 1):
        missing = {"id", "target", "gauge_world", "pose_map", "left_arm_pose",
                   "arm_settle_sec", "dwell_sec"} - set(stop)
        if missing:
            parser.error("第 %d 个点缺少字段: %s" %
                         (index, ", ".join(sorted(missing))))
        if len(stop["gauge_world"]) != 3 or len(stop["pose_map"]) != 3:
            parser.error("%s 的 gauge_world/pose_map 必须各有 3 个数" % stop["id"])
        if len(stop["left_arm_pose"]) != 6:
            parser.error("%s 的 left_arm_pose 必须有 6 个关节值" % stop["id"])
        if float(stop["dwell_sec"]) < 3.0:
            parser.error("%s 到位后必须至少静止 3 秒再拍摄" % stop["id"])

    rclpy.init()
    # Pedestrian paths are deterministic.  Keep accepting --pedestrian-seed so
    # older launch files remain valid, but never generate a per-run random
    # value or use it to alter motion.
    pedestrian_seed = (args.pedestrian_seed
                       if args.pedestrian_seed is not None else 0)
    node = InspectionRunner(
        route, args.navigation_only,
        # Inspection is also the dynamic-avoidance demonstration.  Keep it
        # enabled by default so a normal run does not silently become a
        # static-only navigation test; --no-dynamic-person remains the escape
        # hatch for regression runs that need a quiet scene.
        dynamic_person=bool(not args.no_dynamic_person),
        pedestrian_seed=pedestrian_seed)
    try:
        success = node.execute(args.goal_timeout)
    except KeyboardInterrupt:
        success = False
    finally:
        if rclpy.ok():
            node.stop_and_close()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
