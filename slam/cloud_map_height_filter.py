#!/usr/bin/env python3
"""Remove floor points from the accumulated cloud used by RViz MapCloud."""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField


_Z_FORMATS = {
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def filter_cloud_data(msg, min_height):
    """Return complete point records whose finite z value is above the floor."""
    if msg.point_step <= 0:
        raise ValueError("point_step must be positive")

    z_field = next((field for field in msg.fields if field.name == "z"), None)
    if z_field is None or z_field.datatype not in _Z_FORMATS:
        raise ValueError("z field must be FLOAT32 or FLOAT64")

    endian = ">" if msg.is_bigendian else "<"
    z_struct = struct.Struct(endian + _Z_FORMATS[z_field.datatype])
    if z_field.offset + z_struct.size > msg.point_step:
        raise ValueError("z field extends past point_step")

    row_step = msg.row_step or msg.point_step * msg.width
    if msg.height and msg.width:
        required = (msg.height - 1) * row_step + msg.width * msg.point_step
        if required > len(msg.data):
            raise ValueError("cloud data is shorter than its dimensions")

    output = bytearray()
    for row in range(msg.height):
        row_start = row * row_step
        for column in range(msg.width):
            point_start = row_start + column * msg.point_step
            z = z_struct.unpack_from(msg.data, point_start + z_field.offset)[0]
            if math.isfinite(z) and min_height <= z:
                output.extend(msg.data[point_start:point_start + msg.point_step])
    return bytes(output)


class CloudMapHeightFilter(Node):
    def __init__(self):
        super().__init__("cloud_map_height_filter")
        self.min_height = float(
            self.declare_parameter("min_height", 0.18).value)
        input_topic = str(
            self.declare_parameter("input_topic", "/cloud_map").value)
        output_topic = str(
            self.declare_parameter("output_topic", "/cloud_map_visual").value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.on_cloud, qos)
        self.message_count = 0
        self.get_logger().info(
            "MapCloud filter: %s -> %s, keeping z >= %.2fm"
            % (input_topic, output_topic, self.min_height))

    def on_cloud(self, msg):
        try:
            data = filter_cloud_data(msg, self.min_height)
        except ValueError as error:
            self.get_logger().error("Cannot filter cloud_map: %s" % error)
            return

        filtered = PointCloud2()
        filtered.header = msg.header
        filtered.height = 1
        filtered.width = len(data) // msg.point_step
        filtered.fields = msg.fields
        filtered.is_bigendian = msg.is_bigendian
        filtered.point_step = msg.point_step
        filtered.row_step = len(data)
        filtered.data = data
        filtered.is_dense = msg.is_dense
        self.publisher.publish(filtered)

        self.message_count += 1
        if self.message_count == 1 or self.message_count % 30 == 0:
            input_points = msg.width * msg.height
            self.get_logger().info(
                "MapCloud points: kept %d/%d, removed %d floor points"
                % (filtered.width, input_points, input_points - filtered.width))


def main():
    rclpy.init()
    node = CloudMapHeightFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
