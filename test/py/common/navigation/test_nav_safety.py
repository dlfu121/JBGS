#!/usr/bin/env python3.8
"""Focused regressions for saved-map planning and corner following."""
import math
import os
import sys

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", ".."))

from nav_p2p import (  # noqa: E402
    CORNER_V_MIN,
    PATH_POINT_TOL,
    ROBOT_RADIUS,
    INFLATE,
    V_MAX,
    NavP2P,
    _advance_path_index,
    _corner_speed_limit,
    _inflate_occupancy,
    _remove_ariac_table_front_inflation,
    _inflate_ariac_tank_side,
    _points_in_ariac_assembly_table,
    _line_is_clear,
    _pgm_to_occupancy,
    _world_height_mask,
    PREFERRED_CLEARANCE,
    HARD_HEIGHT,
    PREFERRED_HEIGHT,
    compute_height_cost,
    _height_clearance_from_points,
)
from slam.bridge.dynamic_dwa import (  # noqa: E402
    DWAConfig,
    HolonomicDWA,
    path_corridor_clearance,
    path_lookahead,
)


def test_saved_map_unknown_is_not_free():
    image = np.asarray([[0, 205, 254]], dtype=np.int16)
    occupancy = _pgm_to_occupancy(
        image, occupied_thresh=0.65, free_thresh=0.25, negate=0)
    assert occupancy.tolist() == [[100, -1, 0]]


def test_inflation_clearance():
    occupancy = np.zeros((31, 31), dtype=np.int8)
    occupancy[15, 15] = 100
    occupancy[2, 2] = -1
    required = ROBOT_RADIUS + INFLATE
    cost, clearance = _inflate_occupancy(
        occupancy.ravel(), 31, 31, 0.05, required)

    assert math.isclose(required, 0.50, abs_tol=1e-9)
    assert cost[15, 24] == 1.0       # 0.45 m from occupied cell
    assert cost[15, 25] == 0.0       # exactly 0.50 m is permitted
    assert cost[2, 2] == 1.0         # unknown cell is also blocked
    assert math.isclose(clearance[15, 25], 0.50, abs_tol=1e-9)


def test_ariac_table_approach_removes_only_inflation():
    occupancy = np.zeros((20, 20), dtype=np.int8)
    occupancy[13, 10] = 100
    occupancy[12, 11] = -1
    cost, _ = _inflate_occupancy(
        occupancy.ravel(), 20, 20, 0.10, ROBOT_RADIUS + INFLATE)

    restored = _remove_ariac_table_front_inflation(
        occupancy, cost, 0.10, (2.40, 4.00))

    assert restored > 0
    assert cost[10, 10] == 0.0       # raw-free table-front inflation removed
    assert cost[13, 10] == 1.0       # original occupied cell remains hard
    assert cost[12, 11] == 1.0       # original unknown cell remains hard
    assert cost[16, 10] == 1.0       # inflation outside the strip remains

    table_returns = _points_in_ariac_assembly_table(np.asarray([
        [3.225, 5.60], [3.225, 5.80], [3.225, 5.36], [4.50, 5.80],
    ]))
    assert table_returns.tolist() == [True, True, False, False]


def test_ariac_tank_side_adds_local_clearance_only():
    occupancy = np.zeros((120, 160), dtype=np.int8)
    cost = np.zeros_like(occupancy, dtype=np.float64)
    added = _inflate_ariac_tank_side(
        occupancy, cost, 0.05, (-9.65, -7.0))

    assert added > 0
    # Inside the tank-side band is hard blocked, while the southern approach
    # remains open so the tank gauge goal can still be reached.
    inside_x = int((-5.50 + 9.65) / 0.05)
    inside_y = int((-3.00 + 7.0) / 0.05)
    south_y = int((-5.00 + 7.0) / 0.05)
    assert cost[inside_y, inside_x] == 1.0
    assert cost[south_y, inside_x] == 0.0


def test_line_of_sight_rejects_corner_cut():
    cost = np.zeros((4, 4), dtype=np.float64)
    assert _line_is_clear(cost, 0, 0, 3, 3)

    cost[0, 1] = 1.0
    assert not _line_is_clear(cost, 0, 0, 1, 1)
    assert _line_is_clear(cost, 0, 0, 0, 3)


def test_lazy_theta_returns_only_clear_edges():
    planner = NavP2P.__new__(NavP2P)
    planner.cost = np.zeros((9, 10), dtype=np.float64)
    planner.grid_shape = planner.cost.shape

    planner.cost[0, 1] = 1.0
    planner.cost[1, 0] = 1.0
    assert planner._astar(0, 0, 8, 8) is None

    planner.cost.fill(0.0)
    planner.cost[2:7, 4] = 1.0
    path = planner._astar(1, 4, 8, 4)
    assert path is not None
    assert all(_line_is_clear(planner.cost, *a, *b)
               for a, b in zip(path, path[1:]))


def test_edge_cost_and_planning_metrics():
    planner = NavP2P.__new__(NavP2P)
    planner.cost = np.zeros((8, 8), dtype=np.float64)
    planner.clearance = np.full((8, 8), PREFERRED_CLEARANCE, dtype=np.float64)
    planner.traversable = planner.cost == 0.0
    planner.grid_shape = planner.cost.shape
    planner.lambda_geo = 0.0
    assert math.isclose(planner.edge_cost((0, 0), (3, 4)), 5.0)

    planner.lambda_geo = 2.0
    planner.clearance[0, 1:4] = 0.55
    assert planner.edge_cost((0, 0), (3, 0)) > 3.0
    path = planner._astar(0, 0, 7, 7)
    assert path is not None
    assert planner.last_plan_stats["success"]
    for key in ("planning_time_ms", "expanded_nodes", "los_checks",
                "los_cells_checked", "path_length_m", "waypoint_count"):
        assert key in planner.last_plan_stats


def test_height_cost_and_table_clearance():
    assert math.isinf(compute_height_cost(HARD_HEIGHT - 0.01))
    assert 0.0 < compute_height_cost(0.90) < 1.0
    assert compute_height_cost(PREFERRED_HEIGHT) == 0.0

    points = np.asarray([[0.05, 0.05, 0.90], [0.15, 0.05, 0.65]],
                        dtype=np.float32)
    height = _height_clearance_from_points(
        points, 2, 4, 0.10, (0.0, 0.0))
    assert math.isclose(height[0, 0], 0.90, abs_tol=1e-6)
    assert math.isclose(height[0, 1], 0.65, abs_tol=1e-6)

    planner = NavP2P.__new__(NavP2P)
    planner.cost = np.zeros((1, 2), dtype=np.float64)
    planner.clearance = np.full((1, 2), 1.0)
    planner.traversable = planner.cost == 0.0
    planner.height_clearance = np.asarray([[0.90, 0.65]])
    planner.height_cost = np.asarray([[compute_height_cost(0.90),
                                       compute_height_cost(0.65)]])
    planner.lambda_height = 1.0
    planner.hard_height = HARD_HEIGHT
    planner.lambda_geo = 0.0
    assert math.isfinite(planner.edge_cost((0, 0), (0, 0)))
    assert math.isinf(planner.edge_cost((0, 0), (1, 0)))


def test_path_corner_is_not_skipped():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

    index = _advance_path_index(path, 1, 0.70, 0.0)
    assert index == 1
    index = _advance_path_index(path, 1, 1.0 - PATH_POINT_TOL / 2.0, 0.0)
    assert index == 2


def test_corner_speed_limit_applies_before_and_after_turn():
    corner = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    straight = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]

    assert math.isclose(_corner_speed_limit(corner, 1, 0.20, 0.0), V_MAX)
    assert CORNER_V_MIN < _corner_speed_limit(corner, 1, 0.75, 0.0) < V_MAX
    assert math.isclose(
        _corner_speed_limit(corner, 1, 1.0 - PATH_POINT_TOL / 2.0, 0.0),
        CORNER_V_MIN)
    assert math.isclose(
        _corner_speed_limit(corner, 2, 1.0, PATH_POINT_TOL / 2.0),
        CORNER_V_MIN)
    assert math.isclose(_corner_speed_limit(straight, 1, 0.9, 0.0), V_MAX)


def test_dynamic_obstacle_only_triggers_on_upcoming_path():
    path = [(0.0, 0.0), (4.0, 0.0)]
    blocking = np.asarray([[1.0, 0.20]])
    unrelated = np.asarray([[1.0, 1.50]])
    assert path_corridor_clearance(blocking, (0.0, 0.0), path, 1) < 0.62
    assert path_corridor_clearance(unrelated, (0.0, 0.0), path, 1) > 0.62
    assert np.allclose(path_lookahead((0.0, 0.0), path, 1, 1.2),
                       (1.2, 0.0))


def test_dynamic_height_filter_tracks_actual_lidar_height():
    # ARIAC lowers the base/lidar by 8 cm.  A grazing floor return must stay
    # excluded while a pedestrian return at the same XY remains available.
    points = np.asarray([
        [0.0, 0.0, -0.84],  # world z=0.03: floor/grazing return
        [0.0, 0.0, -0.60],  # world z=0.27: pedestrian/obstacle
        [0.0, 0.0, 0.50],   # world z=1.37: above safety volume
    ])
    assert _world_height_mask(points, sensor_world_z=0.87).tolist() == [
        False, True, False]


def test_holonomic_dwa_avoids_dynamic_obstacle_and_keeps_static_map_hard():
    planner = HolonomicDWA(DWAConfig(dynamic_clearance=0.42))
    path = [(0.0, 0.0), (3.0, 0.0)]
    obstacle = np.asarray([[0.85, 0.0]])
    command = planner.plan(
        (0.0, 0.0, 0.0), (1.2, 0.0), path, 1, obstacle,
        lambda x, y: 1.0 if abs(y) < 0.75 else -1.0)
    assert command is not None
    assert abs(command[1]) > 0.05 or command[0] <= 0.12

    blocked = planner.plan(
        (0.0, 0.0, 0.0), (1.2, 0.0), path, 1, obstacle,
        lambda x, y: -1.0)
    assert blocked is None


def main():
    test_saved_map_unknown_is_not_free()
    test_inflation_clearance()
    test_ariac_table_approach_removes_only_inflation()
    test_line_of_sight_rejects_corner_cut()
    test_lazy_theta_returns_only_clear_edges()
    test_edge_cost_and_planning_metrics()
    test_height_cost_and_table_clearance()
    test_path_corner_is_not_skipped()
    test_corner_speed_limit_applies_before_and_after_turn()
    test_dynamic_obstacle_only_triggers_on_upcoming_path()
    test_dynamic_height_filter_tracks_actual_lidar_height()
    test_holonomic_dwa_avoids_dynamic_obstacle_and_keeps_static_map_hard()
    print(
        "PASS unknown=blocked clearance=%.2fm corner_cut=blocked "
        "waypoint_gate=ok corner_slow=%.2fm/s"
        % (ROBOT_RADIUS + INFLATE, CORNER_V_MIN))


if __name__ == "__main__":
    main()
