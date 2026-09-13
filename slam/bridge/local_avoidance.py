#!/usr/bin/env python3
"""Local scan analysis and recovery state machine shared by the 3D bridges.

The lidar reports slant range. Navigation must use horizontal clearance: for a
low obstacle, most of the slant range is the vertical drop from the lidar and
is not usable stopping distance.
"""
from dataclasses import dataclass
import math

import numpy as np


def _angle_diff(angle, center=0.0):
    return np.arctan2(np.sin(angle - center), np.cos(angle - center))


def build_range_rays(points, num_rays=360, range_max=8.0,
                     z_min=-0.85, z_max=0.50):
    """PointCloud2 点(lidar 局部系, 前方=x+) -> 每方位角的最小水平净距。

    points: (N, 3) 或 (N, 4) float 数组（忽略第 4 列）。
    无命中扇区取 range_max。返回 shape (num_rays,) 的 float64 数组。
    """
    pts = np.asarray(points, dtype=np.float64)
    rays = np.full(num_rays, range_max, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
        return rays
    z = pts[:, 2]
    pts = pts[(z > z_min) & (z < z_max)]
    if len(pts) == 0:
        return rays

    dist = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
    angle = np.arctan2(pts[:, 1], pts[:, 0])  # [-pi, pi]
    idx = np.clip(((angle + np.pi) / (2 * np.pi) * num_rays).astype(int),
                  0, num_rays - 1)
    np.minimum.at(rays, idx, dist)
    return rays


@dataclass(frozen=True)
class AvoidanceConfig:
    range_min: float
    range_max: float
    emergency_dist: float
    slow_dist: float
    front_half_angle_deg: float
    max_turn_rate: float
    # dog_collision lateral half-width is 0.313m.  The extra margin keeps the
    # swept body clear without treating parallel corridor walls as obstacles.
    front_corridor_half_width: float = 0.42
    body_z_min: float = 0.07
    body_z_max: float = 1.25
    release_margin: float = 0.30
    backup_time: float = 0.80
    backup_speed: float = 0.16
    backup_stop_dist: float = 0.50
    turn_min_time: float = 1.00
    turn_max_time: float = 3.20
    clear_min_time: float = 0.50
    side_clear_hold: float = 0.30
    clear_speed: float = 0.22
    clear_turn_rate: float = 0.12


@dataclass(frozen=True)
class ScanClearance:
    # front is the broad monitoring fan; path_front is the swept body corridor.
    front: float
    rear: float
    left_score: float
    right_score: float
    left_clearance: float = float("inf")
    right_clearance: float = float("inf")
    path_front: float = float("inf")


class LocalAvoidance:
    """Stateful obstacle avoidance with direction locking and hysteresis."""

    IDLE = "idle"
    BACKUP = "backup"
    TURN = "turn"
    CLEAR = "clear"

    def __init__(self, config):
        self.config = config
        self.phase = self.IDLE
        self.turn_dir = 1.0  # positive yaw and y are left in base_link
        self._phase_start = 0.0
        self._caution_dir = 1.0
        self._caution_active = False
        self._side_clear_since = None
        self.forced = False

    @property
    def active(self):
        return self.phase != self.IDLE

    def reset(self):
        self.phase = self.IDLE
        self._phase_start = 0.0
        self._caution_active = False
        self._side_clear_since = None
        self.forced = False

    def analyze(self, ranges, directions, lidar_z):
        """Convert a layered range image to footprint-relevant clearances."""
        cfg = self.config
        ranges = np.asarray(ranges)
        directions = np.asarray(directions)
        if ranges.shape != directions.shape[:2]:
            raise ValueError("ranges and directions have incompatible shapes")

        hit_z = lidar_z + ranges * directions[:, :, 2]
        horizontal = ranges * np.linalg.norm(directions[:, :, :2], axis=2)
        valid = ((ranges > cfg.range_min) & (ranges < cfg.range_max)
                 & (hit_z > cfg.body_z_min) & (hit_z < cfg.body_z_max))

        per_ray = np.where(valid, horizontal, np.inf).min(axis=0)
        per_ray[~np.isfinite(per_ray)] = cfg.range_max
        angles = np.arctan2(directions[0, :, 1], directions[0, :, 0])

        def sector_min(center, half_width):
            mask = np.abs(_angle_diff(angles, center)) <= half_width
            return float(per_ray[mask].min()) if mask.any() else cfg.range_max

        # Keep a broad fan for monitoring approaching dynamic obstacles, but
        # use the swept-width corridor for stop/recovery decisions.  A wide
        # cone alone confuses parallel shelves with a blocked path.
        hit_x = ranges * directions[:, :, 0]
        hit_y = ranges * directions[:, :, 1]
        front_half = math.radians(cfg.front_half_angle_deg)
        front = sector_min(0.0, front_half)
        in_front_angle = np.abs(_angle_diff(
            np.arctan2(hit_y, hit_x), 0.0)) <= front_half
        front_mask = (valid & (hit_x > 0.0) & in_front_angle
                      & (np.abs(hit_y) <= cfg.front_corridor_half_width))
        front_values = np.where(front_mask, horizontal, np.inf)
        path_front = float(front_values.min())
        if not math.isfinite(path_front):
            path_front = cfg.range_max
        rear = sector_min(math.pi, math.radians(35.0))

        # Score the whole side rather than only the closest ray. This keeps a
        # thin leg from outweighing a genuinely open route around the object.
        side_inner = math.radians(12.0)
        side_outer = math.radians(105.0)

        def side_score(sign):
            signed = angles * sign
            mask = (signed >= side_inner) & (signed <= side_outer)
            values = np.minimum(per_ray[mask], cfg.slow_dist * 1.5)
            if not len(values):
                return cfg.range_max
            return float(0.65 * np.percentile(values, 30)
                         + 0.35 * values.mean())

        def side_clearance(sign):
            signed = angles * sign
            mask = ((signed >= math.radians(45.0))
                    & (signed <= math.radians(125.0)))
            return (float(per_ray[mask].min())
                    if mask.any() else cfg.range_max)

        return ScanClearance(
            front=front,
            rear=rear,
            left_score=side_score(1.0),
            right_score=side_score(-1.0),
            left_clearance=side_clearance(1.0),
            right_clearance=side_clearance(-1.0),
            path_front=path_front,
        )

    def choose_turn(self, scan, preferred=0.0):
        delta = scan.left_score - scan.right_score
        if abs(delta) > 0.08:
            # Scores are clearance/free-space scores (larger is safer), so
            # turn toward the side with the larger score.
            return 1.0 if delta > 0.0 else -1.0
        if preferred:
            return 1.0 if preferred > 0.0 else -1.0
        # A repeated symmetric failure tries the other side next time.
        return -self.turn_dir

    def begin(self, now, scan, preferred=0.0, forced=False):
        self.turn_dir = self.choose_turn(scan, preferred)
        self.forced = forced
        self._phase_start = now
        self._side_clear_since = None
        self.phase = (self.BACKUP if scan.rear > self.config.backup_stop_dist
                      else self.TURN)

    def _obstacle_side_clearance(self, scan):
        # Turning left moves the original obstacle onto the robot's right,
        # and vice versa.  Do not turn back toward the route until that side
        # has actually cleared.
        return (scan.right_clearance if self.turn_dir > 0.0
                else scan.left_clearance)

    def _blocking_front(self, scan):
        # Hand-built ScanClearance values from older callers/tests have no
        # path_front.  Preserve their former behavior via the fan value.
        return (scan.path_front if math.isfinite(scan.path_front)
                else scan.front)

    def recovery_command(self, now, scan, preferred=0.0, force=False):
        """Return a recovery command, or None when normal control may resume."""
        cfg = self.config
        if not self.active:
            if (not force
                    and self._blocking_front(scan) >= cfg.emergency_dist):
                return None
            self.begin(now, scan, preferred=preferred, forced=force)

        # A transition may make the next phase immediately actionable, so a
        # small bounded loop avoids inserting artificial zero-command ticks.
        for _ in range(3):
            elapsed = now - self._phase_start

            if self.phase == self.BACKUP:
                if (elapsed < cfg.backup_time
                        and scan.rear > cfg.backup_stop_dist):
                    return (-cfg.backup_speed, 0.0, 0.0)
                self.phase = self.TURN
                self._phase_start = now
                self._side_clear_since = None
                continue

            if self.phase == self.TURN:
                elapsed = now - self._phase_start
                release_dist = cfg.emergency_dist + cfg.release_margin
                must_turn = elapsed < cfg.turn_min_time
                still_blocked = (self._blocking_front(scan) < release_dist
                                 and elapsed < cfg.turn_max_time)
                if must_turn or still_blocked:
                    return (0.0, 0.0, self.turn_dir * cfg.max_turn_rate)
                self.phase = self.CLEAR
                self._phase_start = now
                self._side_clear_since = None
                continue

            if self.phase == self.CLEAR:
                # Do not back up again if a new obstacle appears while clearing;
                # keep the locked direction and rotate until that route opens.
                if self._blocking_front(scan) < cfg.emergency_dist:
                    self.phase = self.TURN
                    self._phase_start = now
                    self._side_clear_since = None
                    return (0.0, 0.0, self.turn_dir * cfg.max_turn_rate)

                side_release = cfg.emergency_dist + cfg.release_margin
                side_clear = (self._obstacle_side_clearance(scan)
                              >= side_release)
                if side_clear:
                    if self._side_clear_since is None:
                        self._side_clear_since = now
                    stable = now - self._side_clear_since
                    if (elapsed >= cfg.clear_min_time
                            and stable >= cfg.side_clear_hold):
                        self.reset()
                        return None
                else:
                    self._side_clear_since = None

                return (cfg.clear_speed, 0.0,
                        self.turn_dir * cfg.clear_turn_rate)

        return (0.0, 0.0, self.turn_dir * cfg.max_turn_rate)

    def caution_command(self, now, scan, goal_error, cruise_speed):
        """Return a locked preventive steering command inside the slow zone."""
        cfg = self.config
        path_front = self._blocking_front(scan)
        # The broad fan remains available for monitoring and turn-side
        # selection.  A single scan cannot distinguish a parallel static wall
        # from a crossing person, so motion commands are gated by actual
        # intrusion into the swept body corridor.
        if path_front >= cfg.slow_dist:
            self._caution_active = False
            return None

        if not self._caution_active:
            preferred = 1.0 if goal_error >= 0.0 else -1.0
            self._caution_dir = self.choose_turn(scan, preferred)
            self._caution_active = True

        span = max(cfg.slow_dist - cfg.emergency_dist, 1e-6)
        clearance_ratio = np.clip(
            (path_front - cfg.emergency_dist) / span, 0.0, 1.0)
        speed = cruise_speed * float(clearance_ratio)
        turn = (0.35 + 0.25 * (1.0 - float(clearance_ratio)))
        turn = min(cfg.max_turn_rate, turn)

        # A goal correction may strengthen avoidance, but must never cancel it.
        goal_turn = np.clip(1.8 * goal_error,
                            -cfg.max_turn_rate, cfg.max_turn_rate)
        if goal_turn * self._caution_dir > 0.0:
            turn = max(turn, abs(float(goal_turn)))
        return (speed, 0.0, self._caution_dir * turn)
