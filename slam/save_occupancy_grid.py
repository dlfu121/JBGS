#!/usr/bin/env python3.8
"""Save one transient-local OccupancyGrid as a Nav2 PGM/YAML map."""
import argparse
import math
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


class MapWriter(Node):
    def __init__(self, topic, output):
        super().__init__("reliable_map_writer")
        self.output = output
        self.saved = False
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            raise RuntimeError("invalid OccupancyGrid dimensions")

        pgm_path = self.output + ".pgm"
        yaml_path = self.output + ".yaml"
        with open(pgm_path, "wb") as stream:
            stream.write(("P5\n# CREATOR: reliable ROS2 map writer\n"
                          "%d %d\n255\n" % (width, height)).encode("ascii"))
            pixels = bytearray(width * height)
            for image_y in range(height):
                map_y = height - image_y - 1
                for x in range(width):
                    value = msg.data[map_y * width + x]
                    pixels[image_y * width + x] = (
                        0 if value >= 65 else 254 if 0 <= value <= 25 else 205)
            stream.write(pixels)

        q = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with open(yaml_path, "w", encoding="ascii") as stream:
            stream.write(
                "image: %s\n"
                "mode: trinary\n"
                "resolution: %.9g\n"
                "origin: [%.9g, %.9g, %.9g]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.25\n"
                % (os.path.basename(pgm_path), msg.info.resolution,
                   msg.info.origin.position.x, msg.info.origin.position.y, yaw)
            )
        print("saved width=%d height=%d resolution=%.4f cells=%d"
              % (width, height, msg.info.resolution, len(msg.data)), flush=True)
        self.saved = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = MapWriter(args.topic, args.output)
    deadline = time.monotonic() + args.timeout
    try:
        while not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.saved:
        raise SystemExit("timed out waiting for transient-local %s" % args.topic)


if __name__ == "__main__":
    main()
