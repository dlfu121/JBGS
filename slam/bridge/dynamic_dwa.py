#!/usr/bin/env python3
"""Conditional holonomic DWA helpers, independent of ROS message types."""

from dataclasses import dataclass
import heapq
import math

import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_segment_distances(points, start, end):
    """Return the distance from each 2D point to a finite segment."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty(0, dtype=np.float64)
    start = np.asarray(start, dtype=np.float64)
    delta = np.asarray(end, dtype=np.float64) - start
    denom = float(np.dot(delta, delta))
    if denom < 1e-12:
        return np.linalg.norm(points - start, axis=1)
    ratio = np.clip(((points - start) @ delta) / denom, 0.0, 1.0)
    projection = start + ratio[:, None] * delta
    return np.linalg.norm(points - projection, axis=1)


def path_corridor_clearance(points, pose_xy, path, path_index,
                            forward_distance=3.0,
                            corridor_half_width=None):
    """Minimum obstacle distance to the upcoming part of a sparse path.

    ``corridor_half_width`` optionally limits the trigger to the swept body
    corridor.  This matters in shelf aisles: a parallel shelf can be close to
    the path centreline while still being safely outside the robot footprint.
    The default ``None`` preserves the original centreline-distance behavior
    for callers that use this helper as a generic geometric query.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0 or not path:
        return float("inf")
    index = max(0, min(int(path_index), len(path) - 1))
    start = np.asarray(pose_xy, dtype=np.float64)
    remaining = max(0.0, float(forward_distance))
    best = float("inf")
    while remaining > 1e-9 and index < len(path):
        waypoint = np.asarray(path[index], dtype=np.float64)
        length = float(np.linalg.norm(waypoint - start))
        if length > remaining and length > 1e-12:
            end = start + (waypoint - start) * (remaining / length)
            remaining = 0.0
        else:
            end = waypoint
            remaining -= length
            index += 1
        segment_points = points
        if corridor_half_width is not None and len(points):
            delta = end - start
            denom = float(np.dot(delta, delta))
            if denom < 1e-12:
                lateral = np.linalg.norm(points - start, axis=1)
            else:
                ratio = np.clip(((points - start) @ delta) / denom, 0.0, 1.0)
                projection = start + ratio[:, None] * delta
                lateral = np.linalg.norm(points - projection, axis=1)
            segment_points = points[lateral <= float(corridor_half_width)]
        distances = point_segment_distances(segment_points, start, end)
        if distances.size:
            best = min(best, float(distances.min()))
        start = end
    return best


def path_lookahead(pose_xy, path, path_index, distance=1.2):
    """Interpolate a local target along a sparse global path."""
    if not path:
        return None
    index = max(0, min(int(path_index), len(path) - 1))
    start = np.asarray(pose_xy, dtype=np.float64)
    remaining = max(0.0, float(distance))
    while index < len(path):
        waypoint = np.asarray(path[index], dtype=np.float64)
        length = float(np.linalg.norm(waypoint - start))
        if length >= remaining and length > 1e-12:
            target = start + (waypoint - start) * (remaining / length)
            return float(target[0]), float(target[1])
        remaining -= length
        start = waypoint
        index += 1
    return tuple(path[-1])


def detour_path_index(pose_xy, path, path_index, static_clearance,
                      search_distance=5.0):
    """Rejoin forward segments after bypassing waypoints during avoidance.

    Normal corner following requires reaching every waypoint. Applying that
    rule to a sidestep keeps the target behind the robot and makes DWA turn
    back toward the person. Only skip to a nearby segment with a clear static
    connection; never jump across a wall or a distant fold in the route.
    """
    index = max(0, min(int(path_index), len(path) - 1))
    best_index, best_distance = index, float('inf')
    position = np.asarray(pose_xy)
    travelled = 0.0
    for end_index in range(max(1, index), len(path)):
        start, end = np.asarray(path[end_index - 1]), np.asarray(path[end_index])
        delta = end - start
        length = float(np.linalg.norm(delta))
        if end_index > max(1, index):
            travelled += length
        if travelled > search_distance:
            break
        ratio = np.clip(np.dot(position - start, delta) / max(length ** 2, 1e-12), 0., 1.)
        projection = start + ratio * delta
        distance = float(np.linalg.norm(position - projection))
        if distance >= best_distance:
            continue
        connection = np.linspace(position, projection, max(2, int(distance / .05) + 2))
        if all(static_clearance(*point) >= 0.0 for point in connection):
            best_index, best_distance = max(index, end_index), distance
    return best_index


class LocalDetour:
    """Find a short route around live obstacles before scoring DWA velocities.

    A goal on the far side of a person is a local minimum for constant-velocity
    DWA: staying on the global line can score better than beginning a detour.
    This bounded grid search compares complete left/right passages. Previous
    route affinity prevents noisy scans from repeatedly switching sides.
    """

    def __init__(self, clearance=0.47, resolution=0.15, radius=4.2):
        self.clearance = clearance
        self.resolution = resolution
        self.radius = radius
        self.path = []

    def reset(self):
        self.path = []

    def plan(self, pose_xy, goal, obstacles, static_clearance):
        resolution = self.resolution
        half = int(math.ceil(self.radius / resolution))
        size = 2 * half + 1
        origin = np.asarray(pose_xy) - half * resolution
        yy, xx = np.mgrid[:size, :size]
        positions = origin + np.column_stack((xx.ravel(), yy.ravel())) * resolution
        static = np.asarray([static_clearance(x, y) for x, y in positions])
        points = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
        distances = np.full(len(positions), np.inf)
        for start in range(0, len(positions), 128):
            if len(points):
                delta = positions[start:start + 128, None, :] - points[None, :, :]
                distances[start:start + 128] = np.sqrt(
                    np.sum(delta * delta, axis=2).min(axis=1))
        # Half a cell diagonal reserves room between grid centres; DWA still
        # checks its actual rollout against the original hard clearance.
        blocked = ((static < 0.0) | (distances < self.clearance + resolution * 0.71))
        costs = 1.0 + 0.30 / np.maximum(distances, 0.1)
        costs += 0.12 / np.maximum(static, 0.1)
        if self.path:
            old = np.asarray(self.path)
            affinity = np.linalg.norm(positions[:, None, :] - old[None, :, :], axis=2).min(axis=1)
            costs += 0.65 * np.minimum(affinity, 1.5)
        start_id = half * size + half
        # Never carve a start out of an actual obstacle. The discretisation
        # reserve alone may be relaxed at the measured robot position.
        if static[start_id] < 0.0 or distances[start_id] < self.clearance:
            self.reset()
            return []
        blocked[start_id] = False
        goal_distance = np.linalg.norm(positions - np.asarray(goal), axis=1)
        # A moving person may occupy the exact global lookahead. Permit a
        # nearby endpoint, but never select the robot's own waiting position.
        endpoint_radius = min(0.50, max(resolution * 0.75,
                                       goal_distance[start_id] * 0.15))
        endpoints = (goal_distance <= endpoint_radius) & ~blocked
        if not endpoints.any():
            self.reset()
            return []
        best = np.full(len(positions), np.inf)
        best[start_id] = 0.0
        parents = {}
        queue = [(float(goal_distance[start_id]), 0.0, start_id)]
        finish = None
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > best[current]:
                continue
            if endpoints[current]:
                finish = current
                break
            cy, cx = divmod(current, size)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < size and 0 <= ny < size):
                    continue
                nxt = ny * size + nx
                if blocked[nxt]:
                    continue
                if dx and dy and (blocked[cy * size + nx] or blocked[ny * size + cx]):
                    continue
                # Also sample the edge midpoint on the native static map.
                midpoint = (positions[current] + positions[nxt]) * 0.5
                if static_clearance(*midpoint) < 0.0:
                    continue
                candidate = cost + resolution * math.hypot(dx, dy) * (costs[current] + costs[nxt]) * 0.5
                if candidate < best[nxt]:
                    best[nxt] = candidate
                    parents[nxt] = current
                    heapq.heappush(queue, (candidate + max(0.0, goal_distance[nxt] - endpoint_radius), candidate, nxt))
        if finish is None:
            self.reset()
            return []
        route = [finish]
        while route[-1] != start_id:
            route.append(parents[route[-1]])
        self.path = [tuple(positions[index]) for index in reversed(route)]
        return self.path


@dataclass(frozen=True)
class DWAConfig:
    prediction_time: float = 1.8
    time_step: float = 0.12
    max_forward: float = 0.45
    max_reverse: float = 0.10
    max_lateral: float = 0.35
    max_yaw_rate: float = 0.90
    dynamic_clearance: float = 0.47
    path_weight: float = 2.2
    goal_weight: float = 3.0
    yaw_weight: float = 0.35
    obstacle_weight: float = 0.55
    smooth_weight: float = 0.45
    speed_weight: float = 0.75


class HolonomicDWA:
    """Sample constant body-frame velocities over a short local horizon."""

    def __init__(self, config=None):
        self.config = config or DWAConfig()

    def _samples(self):
        cfg = self.config
        vx = np.asarray([-cfg.max_reverse, 0.0, 0.12, 0.26,
                         cfg.max_forward])
        vy = np.linspace(-cfg.max_lateral, cfg.max_lateral, 7)
        wz = np.linspace(-cfg.max_yaw_rate, cfg.max_yaw_rate, 7)
        for x in vx:
            for y in vy:
                for yaw_rate in wz:
                    yield float(x), float(y), float(yaw_rate)

    def _simulate(self, pose, command):
        cfg = self.config
        steps = max(1, int(math.ceil(cfg.prediction_time / cfg.time_step)))
        result = np.empty((steps, 3), dtype=np.float64)
        x, y, yaw = pose
        vx, vy, wz = command
        for index in range(steps):
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * vx - s * vy) * cfg.time_step
            y += (s * vx + c * vy) * cfg.time_step
            yaw = wrap_angle(yaw + wz * cfg.time_step)
            result[index] = (x, y, yaw)
        return result

    @staticmethod
    def _path_distance(point, path, path_index):
        if not path:
            return 0.0
        first = max(0, int(path_index) - 1)
        last = min(len(path) - 1, int(path_index) + 16)
        if first >= last:
            return math.hypot(point[0] - path[last][0],
                              point[1] - path[last][1])
        segments = np.asarray(path[first:last + 1], dtype=np.float64)
        delta = np.diff(segments, axis=0)
        ratio = np.clip(np.sum((point - segments[:-1]) * delta, axis=1)
                        / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0.0, 1.0)
        return float(np.linalg.norm(
            point - (segments[:-1] + ratio[:, None] * delta), axis=1).min())

    def plan(self, pose, local_goal, path, path_index, dynamic_points,
             static_clearance, previous_command=(0.0, 0.0, 0.0)):
        """Return the safest scored command, or None if even stopping is unsafe.

        ``static_clearance(x, y)`` returns metric clearance on the already
        inflated static map, and a negative value outside its traversable area.
        Dynamic points are obstacle surface returns in map coordinates.
        """
        cfg = self.config
        obstacles = np.asarray(dynamic_points, dtype=np.float64).reshape(-1, 2)
        previous = np.asarray(previous_command, dtype=np.float64)
        goal_heading = math.atan2(local_goal[1] - pose[1],
                                  local_goal[0] - pose[0])
        best = None
        best_score = float("inf")
        for command in self._samples():
            trajectory = self._simulate(pose, command)
            static_values = np.asarray(
                [static_clearance(x, y) for x, y in trajectory[:, :2]],
                dtype=np.float64)
            if np.any(static_values < 0.0):
                continue

            dynamic_min = float("inf")
            if obstacles.size:
                delta = trajectory[:, None, :2] - obstacles[None, :, :]
                dynamic_min = float(np.sqrt(np.sum(delta * delta, axis=2)).min())
                if dynamic_min < cfg.dynamic_clearance:
                    continue

            end = trajectory[-1]
            goal_cost = math.hypot(end[0] - local_goal[0],
                                   end[1] - local_goal[1])
            path_cost = self._path_distance(end[:2], path, path_index)
            yaw_cost = abs(wrap_angle(goal_heading - end[2]))
            obstacle_cost = 0.0
            if math.isfinite(dynamic_min):
                obstacle_cost = 1.0 / max(dynamic_min, 0.05)
            smooth_cost = float(np.linalg.norm(
                np.asarray(command, dtype=np.float64) - previous))
            progress_speed = math.hypot(command[0], command[1])
            score = (cfg.goal_weight * goal_cost
                     + cfg.path_weight * path_cost
                     + cfg.yaw_weight * yaw_cost
                     + cfg.obstacle_weight * obstacle_cost
                     + cfg.smooth_weight * smooth_cost
                     - cfg.speed_weight * progress_speed)
            if score < best_score:
                best_score = score
                best = command
        return best
