#!/usr/bin/env python3.8
"""无 ROS 单测：PLY 障碍点投影到三态 PGM。"""

import os
import struct
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "..", "..", "..", "..", "slam"))

from project_cloud_to_pgm import (  # noqa: E402
    project_cloud,
    read_pgm,
    read_ply_xyz,
    write_pgm,
)


def check_projection_and_flip():
    image = np.full((4, 5), 254, dtype=np.uint8)
    # Two returns land in grid (1, 1), one isolated return lands in (3, 2).
    points = np.array([
        [0.51, 0.51, 0.20],
        [0.52, 0.52, 0.25],
        [1.51, 1.01, 0.20],
        [4.0, 1.0, 0.20],  # outside the 5x4 map
        [0.5, 0.5, 0.05],  # ground, excluded
    ], dtype=np.float32)
    result, cells, valid = project_cloud(
        image, points, resolution=0.5, origin=(0.0, 0.0),
        min_height=0.12, max_height=2.0, min_points=2, inflate_cells=0)

    assert valid == 4  # includes the height-valid point outside the map
    assert cells == 1
    # map gy=1 is PGM row height-1-gy = 2
    assert result[2, 1] == 0
    assert result[1, 3] == 254
    assert result[0, 0] == 254


def check_dilation():
    image = np.full((5, 5), 254, dtype=np.uint8)
    points = np.array([[1.1, 1.1, 0.5], [1.1, 1.1, 0.6]], dtype=np.float32)
    result, cells, _ = project_cloud(
        image, points, resolution=1.0, origin=(0.0, 0.0),
        min_height=0.12, max_height=2.0, min_points=2, inflate_cells=1)

    assert cells == 5  # disk radius 1: center plus four neighbours
    assert int((result == 0).sum()) == 5


def check_pgm_roundtrip():
    image = np.array([[0, 205, 254], [254, 0, 205]], dtype=np.uint8)
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "map.pgm")
        write_pgm(path, image)
        np.testing.assert_array_equal(read_pgm(path), image)


def check_binary_ply():
    header = (
        b"ply\n"
        b"format binary_little_endian 1.0\n"
        b"element vertex 2\n"
        b"property float x\n"
        b"property float y\n"
        b"property float z\n"
        b"property uchar intensity\n"
        b"end_header\n"
    )
    payload = struct.pack("<fffBfffB", 1.0, 2.0, 3.0, 7,
                          -1.0, -2.0, -3.0, 8)
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "cloud.ply")
        with open(path, "wb") as stream:
            stream.write(header + payload)
        points = read_ply_xyz(path)
    np.testing.assert_allclose(points, [[1, 2, 3], [-1, -2, -3]])


def main():
    check_projection_and_flip()
    check_dilation()
    check_pgm_roundtrip()
    check_binary_ply()
    print("PASS projection=ok flip=ok dilation=ok pgm=ok binary_ply=ok")


if __name__ == "__main__":
    main()
