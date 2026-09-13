#!/usr/bin/env python3
"""ROS 无关的全局路径规划：膨胀代价栅格 + Dijkstra 可达性 + 路径回溯。

供 frontier_explorer.py 使用，也便于无 ROS 单测。只依赖 numpy/scipy。
"""
import heapq
import math

import numpy as np
from scipy import ndimage

SQRT2 = math.sqrt(2.0)
DIRS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))


class ExplorePlanner:
    """占用栅格 -> 膨胀代价 -> Dijkstra 可达距离 -> 路径回溯。"""

    def __init__(self, inflate=0.45, waypoint_spacing=0.4,
                 max_goal_dist=25.0, near_goal_snap=10,
                 blacklist_radius=2.0):
        self.inflate = inflate
        self.waypoint_spacing = waypoint_spacing
        self.max_goal_dist = max_goal_dist
        self.near_goal_snap = near_goal_snap
        self.blacklist_radius = blacklist_radius

        self.cost = None          # (h,w) float：0 可走 / 1 不可走
        self.resolution = 1.0
        self.origin = (0.0, 0.0)
        self.shape = (0, 0)       # (w, h)
        self.wavefront = None     # (h,w) 可达距离，inf=不可达
        self._parent = {}

    # ---------------- 代价栅格 ----------------

    def update_map(self, data, resolution, origin):
        """data: (h,w) int8 OccupancyGrid 数据（-1 unknown, 0~100 概率）。

        free 格走 EDT 按 inflate 膨胀；unknown 一律视为不可走——
        frontier 本身是 free 边界格，可达即可，不需要穿过 unknown。
        """
        free = (data >= 0) & (data < 50)
        dist = ndimage.distance_transform_edt(free)
        r_infl = self.inflate / resolution
        cost = np.where(dist < r_infl, 1.0, 0.0)
        cost[0, :] = 1.0
        cost[-1, :] = 1.0
        cost[:, 0] = 1.0
        cost[:, -1] = 1.0
        self.cost = cost
        self.resolution = resolution
        self.origin = origin
        h, w = cost.shape
        self.shape = (w, h)
        self.wavefront = None

    # ---------------- 坐标 ----------------

    def world_to_grid(self, x, y):
        ox, oy = self.origin
        r = self.resolution
        return int(math.floor((x - ox) / r)), int(math.floor((y - oy) / r))

    def grid_to_world(self, i, j):
        ox, oy = self.origin
        r = self.resolution
        return ox + (i + 0.5) * r, oy + (j + 0.5) * r

    # ---------------- Dijkstra ----------------

    def compute_dijkstra(self, start_world):
        """从起点做 8-连通 Dijkstra，得到全图可达距离。返回是否成功。"""
        if self.cost is None:
            return False
        w, h = self.shape
        sx, sy = self.world_to_grid(start_world[0], start_world[1])
        if not (0 <= sx < w and 0 <= sy < h):
            return False

        dist = np.full((h, w), np.inf, dtype=np.float64)
        dist[sy, sx] = 0.0
        self._parent = {}
        openq = [(0.0, sx, sy)]
        while openq:
            d, cx, cy = heapq.heappop(openq)
            if d > dist[cy, cx]:
                continue
            for dx, dy, c in DIRS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if self.cost[ny, nx] > 0:
                    continue
                nd = d + c * self.resolution  # 距离统一为米
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    self._parent[(nx, ny)] = (cx, cy)
                    heapq.heappush(openq, (nd, nx, ny))
        self.wavefront = dist
        return True

    # ---------------- 目标吸附与可达性 ----------------

    def _snap_to_free(self, gx, gy):
        """目标格被膨胀占用时，就近找代价栅格里可走的格。"""
        w, h = self.shape
        for r in range(1, self.near_goal_snap + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if (0 <= nx < w and 0 <= ny < h
                            and self.cost[ny, nx] == 0.0):
                        return nx, ny
        return None, None

    def reachable_dist(self, goal_world):
        """目标的可达距离（米），不可达返回 inf。"""
        if self.wavefront is None:
            return math.inf
        w, h = self.shape
        gx, gy = self.world_to_grid(goal_world[0], goal_world[1])
        if not (0 <= gx < w and 0 <= gy < h):
            return math.inf
        if self.cost[gy, gx] > 0.0:
            gx, gy = self._snap_to_free(gx, gy)
            if gx is None:
                return math.inf
        return float(self.wavefront[gy, gx])

    def plan_goal(self, goal_world):
        """生成到目标的世界系 waypoints，不可达返回 None。"""
        if self.wavefront is None:
            return None
        w, h = self.shape
        gx, gy = self.world_to_grid(goal_world[0], goal_world[1])
        if not (0 <= gx < w and 0 <= gy < h):
            return None
        if self.cost[gy, gx] > 0.0:
            gx, gy = self._snap_to_free(gx, gy)
            if gx is None:
                return None
        if not math.isfinite(self.wavefront[gy, gx]):
            return None

        cells = []
        cur = (gx, gy)
        while True:
            cells.append(cur)
            if cur in self._parent:
                cur = self._parent[cur]
            else:
                break
        cells.reverse()

        pts = [self.grid_to_world(i, j) for i, j in cells]

        out = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= self.waypoint_spacing:
                out.append(p)
        if out[-1] != pts[-1]:
            out.append(pts[-1])
        return out

    # ---------------- 目标选择 ----------------

    def select_frontier(self, frontiers, goal_bias=0.6, blacklist=()):
        """按（大小 + 距离）打分选最佳可达 frontier。

        frontiers: [(x, y, size), ...]（世界系）
        blacklist: 拉黑的目标列表 [(x, y), ...]
        返回 (x, y, 可达距离) 或 None。
        """
        if not frontiers or self.wavefront is None:
            return None
        best = None
        best_score = -1e18
        max_size = max(f[2] for f in frontiers)
        for fx, fy, size in frontiers:
            if any(math.hypot(fx - bx, fy - by) < self.blacklist_radius
                   for bx, by in blacklist):
                continue
            dist = self.reachable_dist((fx, fy))
            if not math.isfinite(dist) or dist > self.max_goal_dist:
                continue  # 不可达：墙另一侧 / 孤立区域
            # Frontier size is an information-gain proxy, but raw linear
            # normalization lets one long shelf edge dominate every aisle
            # entrance.  Log compression preserves the preference for larger
            # frontiers without turning the explorer into an outside perimeter
            # follower.
            size_score = (math.log1p(size) /
                          max(math.log1p(max_size), 1e-9))
            dist_score = max(0.0, 1.0 - dist / self.max_goal_dist)
            score = (goal_bias * size_score
                     + (1 - goal_bias) * dist_score)
            if score > best_score:
                best_score = score
                best = (fx, fy, dist)
        return best
