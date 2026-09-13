#!/usr/bin/env python3
"""Add confirmed obstacle returns from a PLY cloud to a trinary PGM map.

RTAB-Map's 3D occupancy projection can leave thin or repeatedly observed
surfaces sparse after probabilistic ray tracing.  The exported global scan
cloud is a better source for the final obstacle silhouette: points above the
ground are projected into the existing map grid and only those cells are
changed to occupied.  Existing free/unknown cells are otherwise preserved.
"""

import argparse
import os
import tempfile
from pathlib import Path

import numpy as np


PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def _read_yaml_map(path):
    values = {}
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    try:
        resolution = float(values["resolution"])
        origin_text = values["origin"].strip().strip("[]")
        origin = tuple(float(part.strip()) for part in origin_text.split(",")[:2])
    except (KeyError, ValueError) as exc:
        raise ValueError("地图 YAML 缺少有效的 resolution/origin") from exc
    if resolution <= 0.0 or len(origin) != 2:
        raise ValueError("地图 YAML 的 resolution/origin 无效")
    return resolution, origin


def _pgm_token(stream):
    """Read one ASCII PGM header token, including comments."""
    while True:
        byte = stream.read(1)
        if not byte:
            raise ValueError("PGM 头部不完整")
        if byte in b" \t\r\n":
            continue
        if byte == b"#":
            stream.readline()
            continue
        break

    token = bytearray(byte)
    while True:
        byte = stream.read(1)
        if not byte or byte in b" \t\r\n":
            break
        if byte == b"#":
            stream.readline()
            break
        token.extend(byte)
    return bytes(token)


def read_pgm(path):
    with open(path, "rb") as stream:
        if _pgm_token(stream) != b"P5":
            raise ValueError("只支持二进制 P5 PGM")
        width = int(_pgm_token(stream))
        height = int(_pgm_token(stream))
        max_value = int(_pgm_token(stream))
        if max_value > 255:
            raise ValueError("只支持 maxval <= 255 的 PGM")
        payload = stream.read(width * height)
    if len(payload) != width * height:
        raise ValueError("PGM 像素数据长度不符")
    return np.frombuffer(payload, dtype=np.uint8).copy().reshape(height, width)


def write_pgm(path, image):
    image = np.asarray(image, dtype=np.uint8)
    if image.ndim != 2:
        raise ValueError("PGM 图像必须是二维数组")

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=".%s." % target.name,
                                     suffix=".tmp", dir=str(target.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            header = "P5\n%d %d\n255\n" % (image.shape[1], image.shape[0])
            stream.write(header.encode("ascii"))
            stream.write(np.ascontiguousarray(image).tobytes())
        os.replace(temp_name, target)
    except Exception:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise


def _read_ply_header(stream):
    format_name = None
    elements = []
    current = None
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError("PLY 头部不完整")
        line = raw.decode("ascii", errors="strict").strip()
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "format":
            format_name = fields[1]
        elif fields[0] == "element":
            current = {"name": fields[1], "count": int(fields[2]),
                       "properties": []}
            elements.append(current)
        elif fields[0] == "property" and current is not None:
            if fields[1] == "list":
                current["properties"].append(("list", fields[2], fields[3],
                                               fields[4]))
            else:
                current["properties"].append((fields[1], fields[2]))
        elif fields[0] == "end_header":
            break
    return format_name, elements


def read_ply_xyz(path):
    with open(path, "rb") as stream:
        format_name, elements = _read_ply_header(stream)
        vertex = next((item for item in elements if item["name"] == "vertex"),
                      None)
        if vertex is None:
            raise ValueError("PLY 中没有 vertex 元素")
        names = [prop[1] for prop in vertex["properties"]
                 if prop[0] != "list"]
        if any(prop[0] == "list" for prop in vertex["properties"]):
            raise ValueError("不支持带 list vertex 属性的 PLY")
        try:
            xyz_indices = [names.index(name) for name in ("x", "y", "z")]
        except ValueError as exc:
            raise ValueError("PLY vertex 缺少 x/y/z 属性") from exc

        if format_name == "ascii":
            rows = []
            for _ in range(vertex["count"]):
                parts = stream.readline().split()
                if len(parts) < len(names):
                    raise ValueError("ASCII PLY vertex 数据不完整")
                rows.append([float(parts[index]) for index in xyz_indices])
            return np.asarray(rows, dtype=np.float32)

        if format_name != "binary_little_endian":
            raise ValueError("只支持 ASCII 或 binary_little_endian PLY")

        dtype_fields = []
        for prop in vertex["properties"]:
            ptype, name = prop
            dtype_code = PLY_TYPES.get(ptype)
            if dtype_code is None:
                raise ValueError("未知 PLY 属性类型: %s" % ptype)
            dtype_fields.append((name, "<" + dtype_code))
        data = np.fromfile(stream, dtype=np.dtype(dtype_fields),
                           count=vertex["count"])
        if data.shape[0] != vertex["count"]:
            raise ValueError("二进制 PLY vertex 数据不完整")
        return np.column_stack([data["x"], data["y"], data["z"]]).astype(
            np.float32, copy=False)


def _disk_offsets(radius):
    offsets = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                offsets.append((dy, dx))
    return offsets


def dilate(mask, radius):
    if radius <= 0:
        return mask
    height, width = mask.shape
    result = np.zeros_like(mask, dtype=bool)
    for dy, dx in _disk_offsets(radius):
        src_y0 = max(0, -dy)
        src_y1 = min(height, height - dy)
        src_x0 = max(0, -dx)
        src_x1 = min(width, width - dx)
        dst_y0 = max(0, dy)
        dst_y1 = min(height, height + dy)
        dst_x0 = max(0, dx)
        dst_x1 = min(width, width + dx)
        result[dst_y0:dst_y1, dst_x0:dst_x1] |= mask[src_y0:src_y1,
                                                      src_x0:src_x1]
    return result


def project_cloud(image, points, resolution, origin, min_height, max_height,
                  min_points, inflate_cells):
    if max_height <= min_height:
        raise ValueError("max_height 必须大于 min_height")
    if min_points < 1:
        raise ValueError("min_points 必须至少为 1")
    if inflate_cells < 0:
        raise ValueError("inflate_cells 不能为负数")

    height, width = image.shape
    valid = points[np.isfinite(points).all(axis=1)]
    valid = valid[(valid[:, 2] > min_height) & (valid[:, 2] <= max_height)]
    if valid.size == 0:
        return image.copy(), 0, 0

    grid = np.floor((valid[:, :2] - np.asarray(origin)) / resolution)
    grid = grid.astype(np.int64)
    inside = ((grid[:, 0] >= 0) & (grid[:, 0] < width) &
              (grid[:, 1] >= 0) & (grid[:, 1] < height))
    grid = grid[inside]
    if grid.size == 0:
        return image.copy(), 0, 0

    cell_ids = grid[:, 0] + width * grid[:, 1]
    counts = np.bincount(cell_ids, minlength=width * height)
    projected = (counts.reshape(height, width) >= min_points)
    projected = dilate(projected, inflate_cells)

    # PGM rows run top-to-bottom while map coordinates run bottom-to-top.
    projected_image = np.flipud(projected)
    result = image.copy()
    result[projected_image] = 0
    return result, int(projected_image.sum()), int(valid.shape[0])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pgm", required=True)
    parser.add_argument("--yaml", required=True)
    parser.add_argument("--ply", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--min-height", type=float, default=0.12)
    parser.add_argument("--max-height", type=float, default=2.0)
    parser.add_argument("--min-points", type=int, default=2)
    parser.add_argument("--inflate-cells", type=int, default=1)
    args = parser.parse_args(argv)

    image = read_pgm(args.pgm)
    resolution, origin = _read_yaml_map(args.yaml)
    points = read_ply_xyz(args.ply)
    result, occupied_cells, valid_points = project_cloud(
        image, points, resolution, origin, args.min_height, args.max_height,
        args.min_points, args.inflate_cells)
    before = int(np.count_nonzero(image == 0))
    after = int(np.count_nonzero(result == 0))
    if occupied_cells == 0:
        raise ValueError("没有可投影到 PGM 范围内的障碍点，拒绝覆盖地图")
    write_pgm(args.output, result)
    print("云点投影: valid_points=%d, projected_cells=%d, "
          "black=%d->%d, output=%s" %
          (valid_points, occupied_cells, before, after, args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
