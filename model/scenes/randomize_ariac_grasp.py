#!/usr/bin/env python3
"""Randomize the ARIAC screwdriver in the right-arm grasp sector.

The input ranges are expressed in the robot frame, where ``forward`` is
world +Y because the mobile robot is configured to face +Y.  A sample is
first drawn from the requested rectangle and then rotated by an angle in
[-25, 25] degrees.  This is the rotated-rectangle (sector) construction,
and not an axis-aligned random box in world coordinates.

The script edits only the opening tag of ``screwdriver_on_assembly_table``
and therefore preserves the hand-authored meshes, comments, and materials
in the source scene.  It can be run on either ariac_lab.xml or the composed
ariac_lab_with_robot_3d.xml.
"""

from __future__ import annotations

import argparse
import math
import random
import re
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from _ultimate_task.vla.task_definition import (  # noqa: E402
    DOG_BASE_Z,
    RIGHT_ARM_BASE_LOCAL_XY,
    RIGHT_ARM_BASE_Z_LOCAL,
    ROBOT_MODEL_REFERENCE_YAW,
    TABLE_APPROACH_FORWARD,
    TABLE_CENTER_WORLD,
    VISIBLE_ROBOT_YAW_DEG,
)


# World geometry (metres).  These values are deliberately kept in one place
# so that the sampler and the robot/table target calculation cannot drift.
TABLE_CENTER = TABLE_CENTER_WORLD
# The table body sits on the floor, so this is the tabletop height in world
# (and local) coordinates: 1.34 m = right-arm base 1.24 m + 0.10 m, matching
# the KeyCollect arm/table relative height.
TABLETOP_Z = 1.340
# KeyCollect tabletop footprint (0.8 m lateral x 1.0 m forward), expressed as
# local bounds around the table centre.
TABLE_X_BOUNDS = (-0.40, 0.40)
TABLE_Y_BOUNDS = (-0.50, 0.50)

# Right-arm baseline and requested comfortable grasp region.  The lateral band
# is shifted 0.05 m towards the robot's left (-X) and kept at +/-0.15 m so the
# 0.36 m screwdriver, which lies laterally, stays inside the 0.8 m-wide
# KeyCollect tabletop.
LATERAL_CENTRE_SHIFT_M = -0.100
# Keep the band in the original [0.52, 0.62] m forward span so the screwdriver
# sits in front of the palm camera (as in KeyCollect) instead of directly under
# it, where the dexterous hand would block the wrist view.  The baseline sits
# TABLE_APPROACH_FORWARD (0.70 m) behind the table centre.
FORWARD_CENTRE_SHIFT_M = 0.000
FORWARD_RANGE = (0.520 + FORWARD_CENTRE_SHIFT_M,
                 0.620 + FORWARD_CENTRE_SHIFT_M)   # user's Delta X, metres
LATERAL_RANGE = (-0.150 + LATERAL_CENTRE_SHIFT_M,
                 0.150 + LATERAL_CENTRE_SHIFT_M)   # user's Delta Y, metres
DELTA_Z = 0.118                     # user's Delta Z, metres
SECTOR_HALF_ANGLE = math.radians(25.0)
# Keep the screwdriver's long axis close to world +X.  The sector angle above
# controls the position region; the tool itself gets a small independent yaw
# variation so it is not generated at exactly the same orientation every time.
SCREWDRIVER_YAW_HALF_ANGLE = math.radians(10.0)

# The screwdriver mesh spans z=+/-0.018 m around its body origin.  Keeping
# its lowest point 1 mm above the 0.86 m tabletop avoids z-fighting while the
# grasp reference remains exactly at TABLETOP_Z.
SCREWDRIVER_BODY_Z = TABLETOP_Z + 0.019
# Use the measured OBJ extents rather than a 0.24 m bounding circle.  The
# latter unnecessarily reserves 24 cm in Y even though the screwdriver is
# only about 42 mm wide.
SCREWDRIVER_X_BOUNDS = (-0.120, 0.240)
SCREWDRIVER_Y_HALF_WIDTH = 0.021

# arm_r/base_link in dog_base coordinates, and the fixed mobile-base height
# used by gen_ariac_robot.py.  The arm installation height is intentionally
# not confused with the grasp baseline height (TABLETOP_Z - DELTA_Z).
RIGHT_ARM_BASE_LOCAL = RIGHT_ARM_BASE_LOCAL_XY
ROBOT_YAW = math.radians(VISIBLE_ROBOT_YAW_DEG)
NOMINAL_FORWARD = TABLE_APPROACH_FORWARD


def rotate_robot_frame(forward: float, lateral: float, theta: float) -> Tuple[float, float]:
    """Map robot-frame forward/lateral offsets into world dx, dy."""
    # With +Y as forward and +X as lateral, a positive theta turns towards
    # world +X.  This is the usual 2-D rotation in the robot's local basis.
    world_dx = forward * math.sin(theta) + lateral * math.cos(theta)
    world_dy = forward * math.cos(theta) - lateral * math.sin(theta)
    return world_dx, world_dy


def _fits_tabletop(local_x: float, local_y: float,
                   screwdriver_yaw: float) -> bool:
    """Return True when the yawed tool stays inside the trimmed tabletop."""
    c, s = math.cos(screwdriver_yaw), math.sin(screwdriver_yaw)
    tool_x_min = local_x + min(x * c - y * s for x in SCREWDRIVER_X_BOUNDS
                               for y in (-SCREWDRIVER_Y_HALF_WIDTH,
                                         SCREWDRIVER_Y_HALF_WIDTH))
    tool_x_max = local_x + max(x * c - y * s for x in SCREWDRIVER_X_BOUNDS
                               for y in (-SCREWDRIVER_Y_HALF_WIDTH,
                                         SCREWDRIVER_Y_HALF_WIDTH))
    tool_y_min = local_y + min(x * s + y * c for x in SCREWDRIVER_X_BOUNDS
                               for y in (-SCREWDRIVER_Y_HALF_WIDTH,
                                         SCREWDRIVER_Y_HALF_WIDTH))
    tool_y_max = local_y + max(x * s + y * c for x in SCREWDRIVER_X_BOUNDS
                               for y in (-SCREWDRIVER_Y_HALF_WIDTH,
                                         SCREWDRIVER_Y_HALF_WIDTH))
    return (tool_x_min >= TABLE_X_BOUNDS[0] and
            tool_x_max <= TABLE_X_BOUNDS[1] and
            tool_y_min >= TABLE_Y_BOUNDS[0] and
            tool_y_max <= TABLE_Y_BOUNDS[1])


def sample_grasp(rng: random.Random) -> Dict[str, float]:
    """Draw one screwdriver reference pose and return world/local values.

    A shifted band can extend past the trimmed tabletop, so positions that
    would place any part of the tool off the table are redrawn rather than
    aborted.  This keeps the whole sampled region usable and never raises.
    """
    for _ in range(500):
        forward = rng.uniform(*FORWARD_RANGE)
        lateral = rng.uniform(*LATERAL_RANGE)
        theta = rng.uniform(-SECTOR_HALF_ANGLE, SECTOR_HALF_ANGLE)
        dx, dy = rotate_robot_frame(forward, lateral, theta)
        # The offsets are defined from the right-hand baseline, not from the
        # tabletop centre.  The baseline sits NOMINAL_FORWARD metres behind the
        # table centre along world -Y; the sampled forward/lateral rectangle is
        # then rotated inside the robot's +/-25 degree sector and added to it.
        baseline_x = TABLE_CENTER[0]
        baseline_y = TABLE_CENTER[1] - NOMINAL_FORWARD
        world_x = baseline_x + dx
        world_y = baseline_y + dy

        # The baseline is TABLETOP_Z - DELTA_Z; the screwdriver grasp reference
        # is TABLETOP_Z.  The mesh origin is 19 mm above the table to account
        # for its 18 mm radius, so the object itself rests on the tabletop.
        local_x = world_x - TABLE_CENTER[0]
        local_y = world_y - TABLE_CENTER[1]
        # A position near a tabletop edge may only admit part of the yaw range;
        # first try several tool-angle perturbations before redrawing the
        # position.
        for _ in range(25):
            # Flip the tool 180 deg about the vertical axis (position
            # unchanged) so the handle and tip face the opposite way.
            screwdriver_yaw = (rng.uniform(-SCREWDRIVER_YAW_HALF_ANGLE,
                                           SCREWDRIVER_YAW_HALF_ANGLE)
                               + math.pi)
            if _fits_tabletop(local_x, local_y, screwdriver_yaw):
                return {
                    "forward": forward,
                    "lateral": lateral,
                    "theta": theta,
                    "screwdriver_yaw": screwdriver_yaw,
                    "world_x": world_x,
                    "world_y": world_y,
                    "world_z": SCREWDRIVER_BODY_Z,
                    "local_x": local_x,
                    "local_y": local_y,
                }
    raise RuntimeError("sampled screwdriver would exceed the trimmed tabletop")


def compute_robot_table_target(
    table_center: Tuple[float, float] = TABLE_CENTER,
    nominal_forward: float = NOMINAL_FORWARD,
) -> Dict[str, float]:
    """Compute the right-arm baseline and mobile-base target in world frame.

    ``nominal_forward`` puts the baseline behind the table along -Y.  MuJoCo
    rotates the arm-base offset by the XML's fixed 180-degree model reference
    plus the navigation yaw.  At the requested +90-degree visible heading,
    this yields (+0.175, +0.100) m in world XY, so the target dog pose is the
    baseline target minus that offset.
    """
    baseline_x = table_center[0]
    baseline_y = table_center[1] - nominal_forward
    model_yaw = ROBOT_MODEL_REFERENCE_YAW + ROBOT_YAW
    c, s = math.cos(model_yaw), math.sin(model_yaw)
    arm_offset_x = RIGHT_ARM_BASE_LOCAL[0] * c - RIGHT_ARM_BASE_LOCAL[1] * s
    arm_offset_y = RIGHT_ARM_BASE_LOCAL[0] * s + RIGHT_ARM_BASE_LOCAL[1] * c
    return {
        "tabletop_z": TABLETOP_Z,
        "baseline_x": baseline_x,
        "baseline_y": baseline_y,
        "baseline_z": TABLETOP_Z - DELTA_Z,
        # At the approach pose, the rotated arm-base offset added to the
        # returned dog target lands exactly on the requested baseline.
        "arm_base_x": baseline_x,
        "arm_base_y": baseline_y,
        "arm_base_z": DOG_BASE_Z + RIGHT_ARM_BASE_Z_LOCAL,
        "dog_x": baseline_x - arm_offset_x,
        "dog_y": baseline_y - arm_offset_y,
        "dog_z": DOG_BASE_Z,
        "dog_yaw": ROBOT_YAW,
    }


def _replace_screwdriver_pose(text: str, sample: Dict[str, float]) -> str:
    """Replace the screwdriver body pose while retaining XML formatting."""
    pattern = re.compile(
        r'(?P<prefix><body\s+name="screwdriver_on_assembly_table"\s+pos=")[^"]+'
        r'(?P<middle>"\s+)(?P<rotation>(?:quat|euler))="[^"]+"')
    # Flip the tool 180 deg about the vertical axis (position unchanged) so the
    # handle and tip face the opposite way.
    replacement = (
        r'\g<prefix>%.9g %.9g %.9g\g<middle>euler="0 0 %.9g"' %
        (sample["local_x"], sample["local_y"], sample["world_z"],
         sample["screwdriver_yaw"]))
    text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise ValueError("could not find screwdriver_on_assembly_table body")
    return text


def randomize_scene(input_path: Path, output_path: Path,
                    seed: Optional[int]) -> Dict[str, float]:
    """Write a randomized scene and return its sampled pose plus targets."""
    if seed is None:
        seed = random.SystemRandom().randrange(0, 2**32)
    rng = random.Random(seed)
    sample = sample_grasp(rng)
    text = input_path.read_text(encoding="utf-8")
    output_path.write_text(_replace_screwdriver_pose(text, sample), encoding="utf-8")
    sample["seed"] = float(seed)
    sample.update(compute_robot_table_target())
    return sample


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path,
        default=Path(__file__).with_name("ariac_lab.xml"),
        help="source XML (scene-only or composed robot scene)")
    parser.add_argument(
        "--output", type=Path, default=None,
        help="output XML; defaults to <input stem>_randomized.xml")
    parser.add_argument("--seed", type=int, default=None,
                        help="reproducible random seed")
    args = parser.parse_args()
    output = args.output or args.input.with_name(args.input.stem + "_randomized.xml")
    sample = randomize_scene(args.input, output, args.seed)
    print("wrote %s" % output)
    print("seed: %d" % sample["seed"])
    print("screwdriver world: (%.4f, %.4f, %.4f) m, position-sector theta=%+.3f deg, "
          "tool yaw=%+.3f deg" % (
        sample["world_x"], sample["world_y"], sample["world_z"],
        math.degrees(sample["theta"]), math.degrees(sample["screwdriver_yaw"])))
    print("sample offsets: forward=%.4f m lateral=%+.4f m" % (
        sample["forward"], sample["lateral"]))
    print("tabletop_z: %.3f m" % sample["tabletop_z"])
    print("right baseline target: (%.4f, %.4f, %.4f) m" % (
        sample["baseline_x"], sample["baseline_y"], sample["baseline_z"]))
    print("dog table approach: (%.4f, %.4f, %.4f), yaw=%+.3f deg" % (
        sample["dog_x"], sample["dog_y"], sample["dog_z"],
        math.degrees(sample["dog_yaw"])))


if __name__ == "__main__":
    main()
