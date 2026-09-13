#!/usr/bin/env python3
"""Inject the mobile robot and 3D lidar into the ARIAC lab scene."""

import argparse
import os
import re
from pathlib import Path

# Model compilation does not create a rendering context. Avoid an inherited
# headless EGL setting that is unsupported by this workstation.
os.environ["MUJOCO_GL"] = "glfw"
import mujoco

try:
    from .initial_pose import ready_keyframe_values
except ImportError:
    from initial_pose import ready_keyframe_values

HERE = Path(__file__).resolve().parent
ROBOT_SOURCE = HERE / "robot_template_3d_py38.xml"
SCENE_SOURCE = HERE.parent / "scenes" / "ariac_lab.xml"
DEFAULT_OUTPUT = HERE / "ariac_lab_with_robot_3d.xml"
# Keep the collision body clearly above the ARIAC floor.  Its half-height is
# 0.647 m and its local centre is z=0.772 m, so z=-0.11 leaves only 15 mm of
# clearance; tiny solver/contact tolerances can then pin the slide joints.
# The visual wheel mesh remains close enough to the floor at this safer offset.
DEFAULT_START_Z = -0.08
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


def section(text, tag):
    match = re.search(r"(?s)  <%s(?:\s[^>]*)?>.*?</%s>" % (tag, tag), text)
    if not match:
        raise ValueError("missing <%s> section" % tag)
    return match.group(0)


def build(start_x, start_y, start_z=DEFAULT_START_Z):
    robot = ROBOT_SOURCE.read_text(encoding="utf-8")
    scene = SCENE_SOURCE.read_text(encoding="utf-8")

    # ariac_lab.xml is stored in model/scenes, while the composed model is
    # written to model/robot.  Relocate the scene-local mesh paths so they
    # continue to resolve from the generated file's directory.
    scene = scene.replace('file="assets/meshes/',
                          'file="../assets/ariac/meshes/')
    scene = scene.replace(' inertia="shell"', '')
    for mesh_name in PLANAR_MESHES:
        scene = scene.replace(
            'file="../assets/ariac/meshes/%s"' % mesh_name,
            'file="../assets/ariac/meshes/compat/%s"' % mesh_name)

    defaults = re.findall(
        r"(?ms)^    <default class=\"(?:arm|hand)_[lr]/main\">.*?^    </default>",
        section(robot, "default"))
    default_block = "  <default>\n%s\n  </default>\n\n" % "\n".join(defaults)
    scene = scene.replace("  <asset>\n", default_block + "  <asset>\n", 1)

    asset_lines = []
    for line in section(robot, "asset").splitlines()[1:-1]:
        if ('mesh name="dog_' in line or 'mesh name="arm_' in line or
                'mesh name="hand_' in line or 'material name="dog_mat"' in line):
            line = re.sub(r'file="[^"]*/model/assets/', 'file="../assets/', line)
            asset_lines.append(line)
    scene = scene.replace("  </asset>", "\n".join(asset_lines) + "\n  </asset>", 1)

    worldbody = section(robot, "worldbody")
    dog_start = worldbody.index('    <body name="dog_base"')
    dog = worldbody[dog_start:worldbody.rfind("  </worldbody>")].rstrip()
    dog = re.sub(r'(<body name="dog_base"\s+pos=")[^"]*"',
                 r'\g<1>%.6g %.6g %.6g"' %
                 (start_x, start_y, start_z), dog, count=1)
    # KeyCollect's training table_camera is a fixed world camera at
    # (-0.62, 0, 0.80), fovy=100, xyaxes
    # (-0.136637,-0.990621,0, 0.134110,-0.018498,0.990794).  Relative to its
    # base_link (-0.7,0,0.6) that is (+0.080, 0, +0.200).  The ARIAC right arm
    # is mounted 180 deg about its base, so mirror that pose across the base
    # (x -> -x, negate the xyaxes x/y) and keep it on fixed arm_r/base_link.
    table_camera = re.search(r'\s*<camera name="table_camera"[^>]*/>', scene)
    if table_camera:
        camera_line = ('      <camera name="table_camera" pos="-0.080 0.000 0.200" '
                       'xyaxes="0.136637 0.990621 0.000000 '
                       '-0.134110 0.018498 0.990794" fovy="100" />')
        scene = scene[:table_camera.start()] + scene[table_camera.end():]
        arm1 = re.search(r'(<body name="arm_r/base_link"[^>]*>\s*\n)', dog)
        if not arm1:
            raise RuntimeError("robot template missing arm_r/base_link")
        dog = dog[:arm1.end()] + camera_line + "\n" + dog[arm1.end():]
    scene = scene.replace("  </worldbody>", dog + "\n  </worldbody>", 1)

    tail = section(robot, "actuator") + "\n\n" + section(robot, "sensor")
    scene = scene.replace("</mujoco>", tail + "\n</mujoco>", 1)
    return scene


def add_ready_keyframe(output):
    """Add a ready keyframe using the composed model's actual qpos layout."""
    model = mujoco.MjModel.from_xml_path(str(output))
    qpos, ctrl = ready_keyframe_values(mujoco, model)

    def values_text(values):
        return " ".join("%.12g" % float(value) for value in values)

    block = (
        "  <keyframe>\n"
        "    <key name=\"ready\" qpos=\"%s\" ctrl=\"%s\"/>\n"
        "  </keyframe>\n" % (values_text(qpos), values_text(ctrl)))
    text = output.read_text(encoding="utf-8")
    if "<keyframe" in text:
        raise RuntimeError("ARIAC source unexpectedly already contains a keyframe")
    output.write_text(text.replace("</mujoco>", block + "</mujoco>", 1),
                      encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-x", type=float, default=4.0)
    parser.add_argument("--start-y", type=float, default=4.6)
    parser.add_argument("--start-z", type=float, default=DEFAULT_START_Z,
                        help="机器人根节点相对地面的高度偏移（米）")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output.write_text(
        build(args.start_x, args.start_y, args.start_z), encoding="utf-8")
    add_ready_keyframe(args.output)
    mujoco.MjModel.from_xml_path(str(args.output))
    print("generated %s (start: %.3f, %.3f, %.3f)" %
          (args.output, args.start_x, args.start_y, args.start_z))


if __name__ == "__main__":
    main()
