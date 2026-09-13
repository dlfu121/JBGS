#!/usr/bin/env python3
"""Build MuJoCo 3.2.3-compatible copies of zero-thickness ARIAC meshes."""

import argparse
import shutil
from pathlib import Path


HERE = Path(__file__).resolve().parent
MESH_DIR = HERE.parent / "assets" / "ariac" / "meshes"
OUTPUT_DIR = MESH_DIR / "compat"
PLANAR_MESHES = (
    "mesh_000_floor_floor_visual_floor_visual_01.obj",
    "mesh_000_floor_floor_visual_floor_visual_02.obj",
    "mesh_000_floor_floor_visual_floor_visual_03.obj",
    "mesh_000_floor_floor_visual_floor_visual_04.obj",
    "mesh_000_floor_floor_visual_floor_visual_05.obj",
    "mesh_000_floor_floor_visual_floor_visual_06.obj",
    "mesh_004_voltage_testing_stand_stand_visual_visual_01.obj",
    "mesh_009_assembly_table_stand_visual_visual_00.obj",
)
MARKER = "# MuJoCo 3.2.3 compatibility tetrahedron"


def read_vertices(path):
    vertices = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append(tuple(float(value) for value in line.split()[1:4]))
    if not vertices:
        raise ValueError("mesh has no vertices: %s" % path)
    return vertices


def build_mesh(source, output):
    vertices = read_vertices(source)
    minimum = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    extent = max(maximum[axis] - minimum[axis] for axis in range(3))
    side = max(0.01, extent * 1e-4)
    base = len(vertices) + 1

    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(str(source), str(output))
    with output.open("a", encoding="ascii") as stream:
        stream.write("\n%s\n" % MARKER)
        stream.write("v %.9g %.9g %.9g\n" % tuple(minimum))
        stream.write("v %.9g %.9g %.9g\n" %
                     (minimum[0] + side, minimum[1], minimum[2]))
        stream.write("v %.9g %.9g %.9g\n" %
                     (minimum[0], minimum[1] + side, minimum[2]))
        stream.write("v %.9g %.9g %.9g\n" %
                     (minimum[0], minimum[1], minimum[2] + side))
        stream.write("f %d %d %d\n" % (base, base + 2, base + 1))
        stream.write("f %d %d %d\n" % (base, base + 1, base + 3))
        stream.write("f %d %d %d\n" % (base, base + 3, base + 2))
        stream.write("f %d %d %d\n" % (base + 1, base + 2, base + 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="only verify that generated meshes are current")
    args = parser.parse_args()

    stale = []
    for name in PLANAR_MESHES:
        source = MESH_DIR / name
        output = OUTPUT_DIR / name
        if not source.is_file():
            raise SystemExit("missing source mesh: %s" % source)
        if (not output.is_file() or
                output.stat().st_mtime_ns < source.stat().st_mtime_ns or
                MARKER not in output.read_text(encoding="utf-8", errors="replace")):
            stale.append((source, output))

    if args.check:
        if stale:
            raise SystemExit("compatibility meshes are missing or stale; run %s" %
                             Path(__file__).relative_to(HERE.parents[1]))
        print("ARIAC compatibility meshes: OK (%d files)" % len(PLANAR_MESHES))
        return

    for source, output in stale:
        build_mesh(source, output)
        print("generated %s" % output.relative_to(HERE.parents[1]))
    print("ARIAC compatibility meshes ready: %d" % len(PLANAR_MESHES))


if __name__ == "__main__":
    main()
