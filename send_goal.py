#!/usr/bin/env python3.8
"""给机器狗发一个目标点，让它自己规划并走过去。

坐标系说明:
  --frame map   地图系（nav_p2p 规划用的坐标系，默认）
  --frame world 世界系（自动做 world -> map 换算；ARIAC 起点 world(4, 4.6)=map 原点）

用法:
  python3.8 send_goal.py 3.0 2.0                 # map 系 (3.0, 2.0)
  python3.8 send_goal.py --world -4 1            # 世界系 (-4, 1) = map (4.5, 1)
  python3.8 send_goal.py --frame map 2.0 -1.5 --wait   # 发完并等待到达/状态
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

MAP_ORIGINS = {
    "ariac": (4.0, 4.6),
    "lab": (-8.5, 0.0),
    "warehouse": (-9.0, -9.0),
}


class GoalSender(Node):
    def __init__(self, x, y, frame, wait, scene, yaw_deg):
        super().__init__("goal_sender")
        self._wait = wait
        self.pub = self.create_publisher(PoseStamped, "nav_goal", 10)
        self._status = None
        if wait:
            self.sub = self.create_subscription(String, "nav_status", self._on_status, 10)

        gx, gy = x, y
        if frame == "world":
            origin = MAP_ORIGINS[scene]
            gx = x - origin[0]
            gy = y - origin[1]
        self.goal = (gx, gy)
        self.yaw = None if yaw_deg is None else math.radians(yaw_deg)
        self.get_logger().info(
            "目标 -> map 系 (%.2f, %.2f)  (原输入 %s 系 (%.2f, %.2f))"
            % (gx, gy, frame, x, y))

    def _on_status(self, msg):
        self._status = msg.data

    def send(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.goal
        if self.yaw is not None:
            msg.pose.orientation.z = math.sin(self.yaw / 2.0)
            msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pub.publish(msg)
        self.get_logger().info("已发布 /nav_goal, 看 RViz 里的绿色 /nav_path")

    def spin_wait(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._status in ("ARRIVED", "UNREACHABLE", "NO_MAP", "STUCK"):
                self.get_logger().info("状态 -> %s" % self._status)
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--frame", choices=("map", "world"), default="map")
    ap.add_argument("--scene", choices=("ariac", "lab", "warehouse"),
                    default="ariac")
    ap.add_argument("--wait", action="store_true", help="发完等待到达/失败状态")
    ap.add_argument("--yaw-deg", type=float,
                    help="到点后的最终朝向（map 系角度）；省略则不约束")
    args = ap.parse_args()

    rclpy.init()
    node = GoalSender(args.x, args.y, args.frame, args.wait, args.scene,
                      args.yaw_deg)
    node.send()
    if args.wait:
        node.spin_wait()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
