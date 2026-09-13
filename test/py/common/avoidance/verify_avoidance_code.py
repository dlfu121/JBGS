#!/usr/bin/env python3.8
"""Fast behavioral checks for local avoidance without ROS or MuJoCo."""
import math
import os
import py_compile
import sys

import numpy as np


ROOT = os.path.dirname(os.path.abspath(__file__))
BRIDGE_DIR = os.path.join(ROOT, "..", "..", "..", "..", "slam", "bridge")
sys.path.insert(0, BRIDGE_DIR)

from local_avoidance import (  # noqa: E402
    AvoidanceConfig,
    LocalAvoidance,
    ScanClearance,
)


COMPILE_TARGETS = [
    os.path.join(BRIDGE_DIR, "local_avoidance.py"),
    os.path.join(BRIDGE_DIR, "explore_planner.py"),
    os.path.join(BRIDGE_DIR, "bridge_warehouse.py"),
    os.path.join(ROOT, "..", "..", "..", "..", "slam", "frontier_explorer.py"),
    os.path.join(ROOT, "..", "..", "warehouse", "avoidance", "test_avoidance_simple.py"),
    os.path.join(ROOT, "..", "navigation", "test_explore_planner.py"),
]


def config():
    return AvoidanceConfig(
        range_min=0.12,
        range_max=8.0,
        emergency_dist=0.75,
        slow_dist=1.50,
        front_half_angle_deg=42.0,
        max_turn_rate=0.60,
    )


def check_compile():
    for path in COMPILE_TARGETS:
        py_compile.compile(path, doraise=True)


def check_horizontal_clearance():
    azimuths = np.radians([-90.0, -30.0, 0.0, 30.0, 90.0])
    vertical = math.radians(-60.0)
    directions = np.zeros((1, len(azimuths), 3), dtype=np.float64)
    directions[0, :, 0] = math.cos(vertical) * np.cos(azimuths)
    directions[0, :, 1] = math.cos(vertical) * np.sin(azimuths)
    directions[0, :, 2] = math.sin(vertical)

    ranges = np.full((1, len(azimuths)), -1.0, dtype=np.float64)
    ranges[0, 2] = 0.90
    scan = LocalAvoidance(config()).analyze(
        ranges, directions, lidar_z=0.95)

    expected = 0.90 * math.cos(math.radians(60.0))
    assert abs(scan.front - expected) < 1e-6, scan
    assert scan.front < config().emergency_dist, scan
    return scan.front


def check_recovery_state_machine():
    avoider = LocalAvoidance(config())
    blocked = ScanClearance(
        front=0.50, rear=2.0, left_score=2.0, right_score=1.0,
        left_clearance=2.0, right_clearance=0.60)
    side_blocked = ScanClearance(
        front=1.30, rear=2.0, left_score=2.0, right_score=1.0,
        left_clearance=2.0, right_clearance=0.60)
    side_clear = ScanClearance(
        front=1.30, rear=2.0, left_score=2.0, right_score=1.0,
        left_clearance=2.0, right_clearance=2.0)

    backup = avoider.recovery_command(0.0, blocked)
    assert avoider.phase == LocalAvoidance.BACKUP
    assert backup == (-config().backup_speed, 0.0, 0.0)

    turn = avoider.recovery_command(0.9, blocked)
    assert avoider.phase == LocalAvoidance.TURN
    assert turn[2] > 0.0

    clearing = avoider.recovery_command(2.0, side_blocked)
    assert avoider.phase == LocalAvoidance.CLEAR
    assert clearing[0] > 0.0

    # Elapsed time alone must not release recovery while the original
    # obstacle is still alongside the robot.
    still_clearing = avoider.recovery_command(4.5, side_blocked)
    assert still_clearing[0] > 0.0
    assert avoider.phase == LocalAvoidance.CLEAR

    avoider.recovery_command(4.6, side_clear)
    finished = avoider.recovery_command(5.0, side_clear)
    assert finished is None
    assert avoider.phase == LocalAvoidance.IDLE


def check_corridor_and_direction_lock():
    cfg = config()
    avoider = LocalAvoidance(cfg)
    angles = np.radians([-40.0, 0.0, 40.0])
    directions = np.zeros((1, len(angles), 3), dtype=np.float64)
    directions[0, :, 0] = np.cos(angles)
    directions[0, :, 1] = np.sin(angles)
    ranges = np.array([[1.20, -1.0, 1.20]])
    corridor = avoider.analyze(ranges, directions, lidar_z=0.95)
    assert abs(corridor.front - 1.20) < 1e-6, corridor
    assert corridor.path_front == cfg.range_max, corridor
    assert avoider.recovery_command(0.0, corridor) is None
    assert avoider.caution_command(0.0, corridor, 0.0, 0.5) is None

    left_open = ScanClearance(1.0, 2.0, 2.0, 1.0)
    right_open = ScanClearance(1.0, 2.0, 1.0, 2.0)
    first = avoider.caution_command(1.0, left_open, 0.0, 0.5)
    second = avoider.caution_command(5.0, right_open, 0.0, 0.5)
    assert first[2] > 0.0 and second[2] > 0.0
    avoider.caution_command(6.0, ScanClearance(2.0, 2.0, 2.0, 2.0),
                            0.0, 0.5)
    third = avoider.caution_command(7.0, right_open, 0.0, 0.5)
    assert third[2] < 0.0


def main():
    check_compile()
    horizontal = check_horizontal_clearance()
    check_recovery_state_machine()
    check_corridor_and_direction_lock()
    print(
        "PASS compiled=%d horizontal_clearance=%.3fm "
        "recovery=backup->turn->clear->idle"
        % (len(COMPILE_TARGETS), horizontal)
    )


if __name__ == "__main__":
    main()
