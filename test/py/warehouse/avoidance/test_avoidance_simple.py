#!/usr/bin/env python3.8
"""Headless MuJoCo regression test for patrol avoidance and tabletop scans."""
import math
import os
import sys

import mujoco
import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "slam", "bridge"))

from local_avoidance import (  # noqa: E402
    AvoidanceConfig,
    LocalAvoidance,
    ScanClearance,
)


XML = os.path.join(HERE, "..", "..", "..", "model", "robot", "warehouse_with_robot_3d.xml")
VERTICAL_ANGLES_DEG = np.concatenate([
    np.linspace(-60.0, -20.0, 12, endpoint=False),
    np.linspace(-20.0, 5.0, 42, endpoint=False),
    np.linspace(5.0, 20.0, 10),
])
NUM_H_RAYS = 360
RANGE_MIN = 0.12
RANGE_MAX = 8.0
CONTROL_HZ = 50.0
SCAN_HZ = 10.0

WAYPOINTS = [
    (-8.5, -8.5), (-8.5, -4.0), (-8.5, 0.0), (-8.5, 4.0),
    (-8.5, 8.5), (-4.0, 8.5), (0.0, 8.5), (4.0, 8.5),
    (5.5, 5.5), (2.2, 5.0), (-3.5, 5.0), (-7.5, 5.0),
    (-8.2, 4.0), (-8.2, 1.8), (-7.5, 1.8), (-3.5, 1.8),
    (1.0, 1.8), (1.0, -3.0), (-3.5, -3.0), (-7.5, -3.0),
    (-8.5, -4.0), (-8.5, -8.5), (-4.0, -8.5), (0.0, -8.5),
    (4.0, -8.5), (8.5, -8.5), (10.2, -4.5), (10.2, 0.0),
    (10.2, 5.5),
]
PATROL_V = 0.50
PATROL_W = 0.60
PATROL_TOL = 0.50
STUCK_WINDOW = 9.0
PROGRESS_DIST = 0.30
MAX_RECOVERY_ATTEMPTS = 3


def direction_table():
    out = np.zeros((len(VERTICAL_ANGLES_DEG), NUM_H_RAYS, 3), dtype=np.float32)
    for layer, angle_deg in enumerate(VERTICAL_ANGLES_DEG):
        phi = math.radians(float(angle_deg))
        cp, sp = math.cos(phi), math.sin(phi)
        for az in range(NUM_H_RAYS):
            theta = -math.pi + 2.0 * math.pi * az / NUM_H_RAYS
            out[layer, az] = (cp * math.cos(theta), cp * math.sin(theta), sp)
    return out


class WarehouseHarness:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.model.opt.disableflags = (
            int(self.model.opt.disableflags)
            | int(mujoco.mjtDisableBit.mjDSBL_SENSOR))

        self.dirs = direction_table()
        self.dirs_flat = self.dirs.reshape(-1, 3).astype(np.float64)
        self.total_rays = len(self.dirs_flat)
        self.ray_geomid = np.zeros(self.total_rays, dtype=np.int32)
        self.ray_dist = np.zeros(self.total_rays, dtype=np.float64)
        self.ranges = np.full(self.dirs.shape[:2], -1.0)

        m = self.model
        self.dog_body = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self.lidar_site = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        self.dog_home = self.data.xpos[self.dog_body][:2].copy()
        self.lidar_z = float(self.data.site_xpos[self.lidar_site][2])
        self.act = {
            name: mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + name)
            for name in ("base_x", "base_y", "base_yaw")
        }
        self.qadr = {
            name: m.jnt_qposadr[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in ("base_x", "base_y", "base_yaw")
        }

        self.avoider = LocalAvoidance(AvoidanceConfig(
            range_min=RANGE_MIN,
            range_max=RANGE_MAX,
            emergency_dist=0.75,
            slow_dist=1.50,
            front_half_angle_deg=42.0,
            max_turn_rate=PATROL_W,
        ))
        self.wp = 1
        self.progress_wp = self.wp
        self.progress_best = float("inf")
        self.progress_t = self.data.time
        self.recovery_attempts = 0
        self.skipped_waypoints = 0
        self.recovery_entries = 0
        self.active_started = None
        self.max_recovery_duration = 0.0
        self.obstacle_contact_ticks = 0
        self.scan_count = 0

        self.robot_bodies = self._descendant_bodies(self.dog_body)
        self.robot_geom_ids = np.asarray([
            geom for geom in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom]) in self.robot_bodies
        ], dtype=np.int32)
        self.scan()

    def _descendant_bodies(self, root):
        result = set()
        for body in range(self.model.nbody):
            current = body
            while current > 0 and current != root:
                current = int(self.model.body_parentid[current])
            if current == root:
                result.add(body)
        return result

    def pose(self):
        return np.array([
            self.dog_home[0] + float(self.data.qpos[self.qadr["base_x"]]),
            self.dog_home[1] + float(self.data.qpos[self.qadr["base_y"]]),
            float(self.data.qpos[self.qadr["base_yaw"]]),
        ])

    def set_pose(self, x, y, yaw):
        self.data.qpos[self.qadr["base_x"]] = x - self.dog_home[0]
        self.data.qpos[self.qadr["base_y"]] = y - self.dog_home[1]
        self.data.qpos[self.qadr["base_yaw"]] = yaw
        mujoco.mj_forward(self.model, self.data)

    def scan(self):
        yaw = self.pose()[2]
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(self.dirs_flat)
        world[:, 0] = c * self.dirs_flat[:, 0] - s * self.dirs_flat[:, 1]
        world[:, 1] = s * self.dirs_flat[:, 0] + c * self.dirs_flat[:, 1]
        world[:, 2] = self.dirs_flat[:, 2]
        mujoco.mj_multiRay(
            self.model, self.data,
            self.data.site_xpos[self.lidar_site], world.ravel(),
            None, 1, self.dog_body,
            self.ray_geomid, self.ray_dist,
            self.total_rays, RANGE_MAX)
        self_hit = np.isin(self.ray_geomid, self.robot_geom_ids)
        self.ray_geomid[self_hit] = -1
        self.ray_dist[self_hit] = -1.0
        raw = self.ray_dist.reshape(self.ranges.shape)
        np.copyto(self.ranges, raw)
        self.ranges[(raw < 0.0) | (raw > RANGE_MAX)] = -1.0
        self.scan_count += 1

    def clearance(self):
        return self.avoider.analyze(self.ranges, self.dirs, self.lidar_z)

    def reset_progress(self, distance=float("inf")):
        self.progress_wp = self.wp
        self.progress_best = distance
        self.progress_t = self.data.time
        self.recovery_attempts = 0

    def patrol_command(self):
        if self.wp >= len(WAYPOINTS):
            return (0.0, 0.0, 0.0)

        pose = self.pose()
        tx, ty = WAYPOINTS[self.wp]
        dx, dy = tx - pose[0], ty - pose[1]
        distance = math.hypot(dx, dy)
        if distance < PATROL_TOL:
            self.wp += 1
            self.avoider.reset()
            self.reset_progress()
            return (0.0, 0.0, 0.0)

        goal_yaw = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(goal_yaw - pose[2]), math.cos(goal_yaw - pose[2]))
        normal_turn = max(-PATROL_W, min(PATROL_W, 1.8 * error))
        now = self.data.time
        scan = self.clearance()

        if self.progress_wp != self.wp:
            self.reset_progress(distance)
        elif distance < self.progress_best - PROGRESS_DIST:
            self.progress_best = distance
            self.progress_t = now
            self.recovery_attempts = 0

        force = False
        if not self.avoider.active and now - self.progress_t >= STUCK_WINDOW:
            if self.recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                self.wp += 1
                self.skipped_waypoints += 1
                self.avoider.reset()
                self.reset_progress()
                return (0.0, 0.0, 0.0)
            self.recovery_attempts += 1
            self.progress_t = now
            force = True

        was_active = self.avoider.active
        recovery = self.avoider.recovery_command(
            now, scan, preferred=1.0 if error >= 0.0 else -1.0,
            force=force)
        if not was_active and self.avoider.active:
            self.recovery_entries += 1
            self.active_started = now
        if was_active and not self.avoider.active and self.active_started is not None:
            self.max_recovery_duration = max(
                self.max_recovery_duration, now - self.active_started)
            self.active_started = None
        if recovery is not None:
            return recovery

        caution = self.avoider.caution_command(
            now, scan, goal_error=error, cruise_speed=PATROL_V)
        if caution is not None:
            return caution
        if abs(error) > 0.35:
            return (0.0, 0.0, normal_turn)
        return (PATROL_V * max(0.25, 1.0 - abs(error)), 0.0, normal_turn)

    def _count_obstacle_contacts(self):
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body1 = int(self.model.geom_bodyid[contact.geom1])
            body2 = int(self.model.geom_bodyid[contact.geom2])
            robot1 = body1 in self.robot_bodies
            robot2 = body2 in self.robot_bodies
            if robot1 == robot2:
                continue
            other_body = body2 if robot1 else body1
            other_name = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, other_body) or "")
            if other_name != "floor":
                self.obstacle_contact_ticks += 1
                return

    def step(self, tick):
        vxb, vyb, wz = self.patrol_command()
        yaw = self.pose()[2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.data.ctrl[self.act["base_x"]] = c * vxb - s * vyb
        self.data.ctrl[self.act["base_y"]] = s * vxb + c * vyb
        self.data.ctrl[self.act["base_yaw"]] = wz
        substeps = max(1, int(round((1.0 / CONTROL_HZ) / self.model.opt.timestep)))
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)
            self._count_obstacle_contacts()
        if (tick + 1) % int(CONTROL_HZ / SCAN_HZ) == 0:
            self.scan()


def test_horizontal_clearance():
    dirs = direction_table()
    ranges = np.full(dirs.shape[:2], -1.0)
    layer = int(np.argmin(np.abs(VERTICAL_ANGLES_DEG + 60.0)))
    front = NUM_H_RAYS // 2
    ranges[layer, front] = 0.90
    avoider = LocalAvoidance(AvoidanceConfig(
        range_min=RANGE_MIN, range_max=RANGE_MAX,
        emergency_dist=0.75, slow_dist=1.50,
        front_half_angle_deg=42.0, max_turn_rate=0.60))
    scan = avoider.analyze(ranges, dirs, lidar_z=0.95)
    assert scan.front < 0.50, scan
    assert scan.front < avoider.config.emergency_dist, scan


def test_recovery_state_machine():
    avoider = LocalAvoidance(AvoidanceConfig(
        range_min=RANGE_MIN, range_max=RANGE_MAX,
        emergency_dist=0.75, slow_dist=1.50,
        front_half_angle_deg=42.0, max_turn_rate=0.60))
    blocked = ScanClearance(0.50, 2.0, 2.0, 1.0, 2.0, 0.60)
    side_blocked = ScanClearance(1.30, 2.0, 2.0, 1.0, 2.0, 0.60)
    side_clear = ScanClearance(1.30, 2.0, 2.0, 1.0, 2.0, 2.0)
    assert avoider.recovery_command(0.0, blocked) == (-0.16, 0.0, 0.0)
    assert avoider.recovery_command(0.9, blocked)[2] > 0.0
    assert avoider.recovery_command(2.0, side_blocked)[0] > 0.0
    assert avoider.recovery_command(4.5, side_blocked)[0] > 0.0
    avoider.recovery_command(4.6, side_clear)
    assert avoider.recovery_command(5.0, side_clear) is None
    assert not avoider.active


def test_tabletop_density(harness):
    harness.set_pose(6.5, 8.5, 0.0)
    harness.scan()
    work_table = mujoco.mj_name2id(
        harness.model, mujoco.mjtObj.mjOBJ_BODY, "work_table")
    tabletop = next(
        geom for geom in range(harness.model.ngeom)
        if int(harness.model.geom_bodyid[geom]) == work_table
        and abs(float(harness.model.geom_pos[geom][2]) - 0.77) < 1e-6)
    hits = int(np.count_nonzero(harness.ray_geomid == tabletop))
    assert hits >= 250, "tabletop has only %d direct lidar hits" % hits
    return hits


def test_full_patrol():
    harness = WarehouseHarness()
    max_sim_time = 480.0
    max_ticks = int(max_sim_time * CONTROL_HZ)
    for tick in range(max_ticks):
        harness.step(tick)
        if harness.wp >= len(WAYPOINTS):
            break

    assert harness.wp >= len(WAYPOINTS), (
        "patrol timed out at waypoint %d, pose=%s, phase=%s"
        % (harness.wp, np.round(harness.pose(), 3), harness.avoider.phase))
    assert harness.skipped_waypoints == 0, (
        "patrol skipped %d waypoints" % harness.skipped_waypoints)
    assert harness.obstacle_contact_ticks == 0, (
        "robot contacted a non-floor obstacle on %d physics steps"
        % harness.obstacle_contact_ticks)
    assert harness.max_recovery_duration < 8.0, (
        "a recovery stayed active for %.2fs" % harness.max_recovery_duration)
    return harness


def main():
    if not os.path.isfile(XML):
        raise SystemExit("missing model: " + XML)
    test_horizontal_clearance()
    test_recovery_state_machine()
    patrol = test_full_patrol()
    table_hits = test_tabletop_density(patrol)
    print(
        "PASS sim_time=%.2fs waypoints=%d recoveries=%d max_recovery=%.2fs "
        "obstacle_contacts=%d tabletop_hits=%d scans=%d"
        % (patrol.data.time, patrol.wp, patrol.recovery_entries,
           patrol.max_recovery_duration, patrol.obstacle_contact_ticks,
           table_hits, patrol.scan_count))


if __name__ == "__main__":
    main()
