#!/usr/bin/env python3.8
"""无 ROS 单测：ExplorePlanner 全局规划 + build_range_rays + LocalAvoidance。

运行:
    python3.8 test/py/common/navigation/test_explore_planner.py
"""
import math
import os
import py_compile
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
BRIDGE_DIR = os.path.join(ROOT, "..", "..", "..", "..", "slam", "bridge")
sys.path.insert(0, BRIDGE_DIR)

from explore_planner import ExplorePlanner  # noqa: E402
from frontier_policy import (  # noqa: E402
    filter_visited_frontiers,
    goal_has_frontier,
    prune_visits,
)
from local_avoidance import (  # noqa: E402
    AvoidanceConfig,
    LocalAvoidance,
    build_range_rays,
)

COMPILE_TARGETS = [
    os.path.join(BRIDGE_DIR, "explore_planner.py"),
    os.path.join(BRIDGE_DIR, "frontier_policy.py"),
    os.path.join(BRIDGE_DIR, "local_avoidance.py"),
    os.path.join(BRIDGE_DIR, "bridge_warehouse.py"),
    os.path.join(ROOT, "..", "..", "..", "..", "slam", "frontier_explorer.py"),
    os.path.join(ROOT, "test_explore_planner.py"),
]

# 合成地图：100x100，res=0.1，原点 (0,0)，即 10m x 10m
RES = 0.1
SIZE = 100


def make_map(wall_cols=(50, 51), wall_rows=None):
    """wall_rows: (r0, r1) 区间（含），None 表示整列墙。返回 (h,w) int8。"""
    occ = np.full((SIZE, SIZE), -1, dtype=np.int8)  # 默认 unknown
    occ[:, :] = 0  # 全部标记为 free
    r0, r1 = wall_rows or (0, SIZE - 1)
    for c in wall_cols:
        occ[r0:r1 + 1, c] = 100  # occupied
    return occ


def check_compile():
    for path in COMPILE_TARGETS:
        py_compile.compile(path, doraise=True)


def check_inflation():
    """膨胀：障碍附近的 free 格变不可走，远处的 free 格保持可走。"""
    occ = make_map(wall_rows=(0, SIZE - 1))  # 整列墙
    planner = ExplorePlanner(inflate=0.45)
    planner.update_map(occ, RES, (0.0, 0.0))
    cost = planner.cost
    assert cost.shape == (SIZE, SIZE)
    # 墙列(50/51)本身及其 4 格内都被占用
    assert cost[50, 50] > 0 and cost[50, 49] > 0
    assert cost[50, 46] > 0   # 距墙 0.4m < 膨胀 0.45
    # 远处可走
    assert cost[50, 20] == 0
    # 地图边界强制不可走
    assert cost[0, 0] > 0 and cost[SIZE - 1, SIZE - 1] > 0
    return cost


def check_unreachable_across_wall():
    """整面墙：墙另一侧的目标不可达，select_frontier 会跳过它。"""
    occ = make_map(wall_rows=(0, SIZE - 1))
    planner = ExplorePlanner(inflate=0.45)
    planner.update_map(occ, RES, (0.0, 0.0))
    assert planner.compute_dijkstra((2.0, 5.0))

    assert math.isinf(planner.reachable_dist((8.0, 5.0)))  # 墙另一侧
    assert planner.plan_goal((8.0, 5.0)) is None

    # 同侧目标可达
    d = planner.reachable_dist((2.0, 2.0))
    assert math.isfinite(d) and d > 0.0

    # select_frontier 只返回可达的候选
    chosen = planner.select_frontier(
        [(8.0, 5.0, 100), (2.0, 2.0, 10)], goal_bias=0.6, blacklist=())
    assert chosen is not None and chosen[0] < 4.0, chosen
    return chosen


def check_path_goes_around_wall():
    """带缺口的墙：路径存在、全程在可走格内、且穿过缺口到达另一侧。"""
    # 墙列 50/51 在 rows 0..39 和 50..99 占据，rows 40..49 为缺口
    occ = make_map()
    occ[0:40, 50] = 100
    occ[50:100, 50] = 100
    occ[0:40, 51] = 100
    occ[50:100, 51] = 100
    occ[40:50, 50] = 0
    occ[40:50, 51] = 0

    planner = ExplorePlanner(inflate=0.45)
    planner.update_map(occ, RES, (0.0, 0.0))
    assert planner.compute_dijkstra((2.0, 5.0))

    path = planner.plan_goal((8.0, 5.0))
    assert path is not None, "缺口应该存在可达路径"

    # 每个 waypoint 都落在可走格
    cost = planner.cost
    for x, y in path:
        gx, gy = planner.world_to_grid(x, y)
        assert cost[gy, gx] == 0.0, "路径穿过了不可走格"

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    assert min(xs) <= 2.5 and max(xs) >= 7.5, "路径应跨越墙两侧"
    assert min(ys) >= 4.0 and max(ys) <= 6.0, "路径应穿过缺口(y 4~5)附近"
    return path


def check_select_frontier_blacklist():
    """拉黑：被拉黑的目标不会被再次选中。"""
    occ = make_map(wall_rows=(0, SIZE - 1))
    planner = ExplorePlanner(inflate=0.45)
    planner.update_map(occ, RES, (0.0, 0.0))
    assert planner.compute_dijkstra((2.0, 5.0))

    frontiers = [(2.0, 2.0, 30), (2.0, 8.0, 5)]
    chosen = planner.select_frontier(
        frontiers, goal_bias=0.6, blacklist=[(2.0, 2.0)])
    assert chosen is not None and chosen[1] == 8.0, chosen
    # 全部拉黑 -> 无候选
    chosen = planner.select_frontier(
        frontiers, goal_bias=0.6,
        blacklist=[(2.0, 2.0), (2.0, 8.0)])
    assert chosen is None
    return chosen


def check_frontier_goal_revalidation():
    """新地图清除目标附近 unknown 后，旧 frontier 应立即失效。"""
    occ = np.full((40, 40), -1, dtype=np.int8)
    occ[10:30, 10:30] = 0
    goal = (1.05, 2.05)  # 已知区域左边界的 free cell 中心
    assert goal_has_frontier(
        occ, RES, (0.0, 0.0), goal, radius=0.3)

    occ[:, :] = 0
    assert not goal_has_frontier(
        occ, RES, (0.0, 0.0), goal, radius=0.3)


def check_visited_frontier_cooldown():
    """访问冷却只抑制邻近目标，并在 TTL 后释放。"""
    frontiers = [(1.0, 1.0, 20), (1.8, 1.0, 10), (5.0, 5.0, 8)]
    visits = [(1.0, 1.0, 10.0)]
    active = prune_visits(visits, now=20.0, ttl=30.0)
    eligible = filter_visited_frontiers(frontiers, active, radius=2.0)
    assert eligible == [(5.0, 5.0, 8)], eligible

    expired = prune_visits(visits, now=40.0, ttl=30.0)
    assert expired == []
    assert filter_visited_frontiers(frontiers, expired, radius=2.0) == frontiers


def check_rays_and_clearance():
    """点云 -> 每方位角净距 -> LocalAvoidance 前方净距与急停。"""
    config = AvoidanceConfig(
        range_min=0.12, range_max=8.0, emergency_dist=0.75,
        slow_dist=1.50, front_half_angle_deg=42.0, max_turn_rate=1.0)
    avoider = LocalAvoidance(config)

    points = np.array([[0.5, 0.0, 0.1],   # 正前方 0.5m 障碍
                       [2.0, 2.0, 0.1],
                       [3.0, -3.0, 0.1]], dtype=np.float32)
    rays = build_range_rays(points, num_rays=360, range_max=8.0,
                            z_min=-0.85, z_max=0.50)
    az = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    dirs = np.zeros((1, 360, 3), dtype=np.float64)
    dirs[0, :, 0] = np.cos(az)
    dirs[0, :, 1] = np.sin(az)

    scan = avoider.analyze(rays[None, :], dirs, lidar_z=1.0)
    assert abs(scan.front - 0.5) < 1e-6, scan.front
    assert scan.front < config.emergency_dist

    # 前方净距小于紧急阈值 -> 立即进入 BACKUP 后退
    recovery = avoider.recovery_command(0.0, scan)
    assert recovery is not None
    assert avoider.phase == LocalAvoidance.BACKUP
    assert recovery[0] < 0.0

    # 无障碍场景 -> 全新实例不触发恢复
    clear_rays = np.full(360, 8.0, dtype=np.float64)
    clear_scan = LocalAvoidance(config).analyze(
        clear_rays[None, :], dirs, lidar_z=1.0)
    assert LocalAvoidance(config).recovery_command(0.0, clear_scan) is None
    return scan.front


def main():
    check_compile()
    check_inflation()
    check_unreachable_across_wall()
    check_path_goes_around_wall()
    check_select_frontier_blacklist()
    check_frontier_goal_revalidation()
    check_visited_frontier_cooldown()
    front = check_rays_and_clearance()
    print(
        "PASS compiled=%d inflation=ok unreachable=filtered "
        "path=around-wall blacklist=ok goal_revalidation=ok "
        "visited_cooldown=ok front_clearance=%.3fm recovery=backup"
        % (len(COMPILE_TARGETS), front)
    )


if __name__ == "__main__":
    main()
