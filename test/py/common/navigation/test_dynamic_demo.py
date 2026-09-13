"""Behavioral regressions for active sidestepping and continuous demo traffic."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from slam.bridge.dynamic_dwa import HolonomicDWA, LocalDetour, path_lookahead
from slam.bridge.pedestrian_motion import PedestrianTraffic, swept_separation


def surface(centres, radius=0.24):
    angle = np.linspace(0, 2 * np.pi, 20, endpoint=False)
    ring = radius * np.column_stack((np.cos(angle), np.sin(angle)))
    return np.concatenate([ring + centre for centre in centres])


@pytest.mark.parametrize('crowded_side', [-1, 1])
def test_robot_sidestep_chooses_less_crowded_passage(crowded_side):
    obstacles = surface([(1.7, 0), (1.5, crowded_side * 1.0),
                         (2.1, crowded_side * 1.5)])
    route = LocalDetour().plan((0, 0), (3.5, 0), obstacles, lambda x, y: 2.0)
    assert route
    target = path_lookahead((0, 0), route, 1)
    dwa = HolonomicDWA()
    command = dwa.plan((0, 0, 0), target, route, 1, obstacles, lambda x, y: 2.0)
    assert command is not None
    assert command[0] > 0.1
    assert command[1] * crowded_side < -0.1
    assert np.linalg.norm(dwa._simulate((0, 0, 0), command)[:, None, :2]
                          - obstacles[None, :, :], axis=2).min() >= dwa.config.dynamic_clearance


def test_robot_completes_detour_past_stationary_person():
    # A stopped person is the strongest test against waiting for traffic to
    # clear. The robot must pass it under repeated feedback, then rejoin.
    obstacles = surface([(1.7, 0), (1.7, 1.05)])
    detour, dwa = LocalDetour(), HolonomicDWA()
    pose = np.zeros(3)
    previous = (0., 0., 0.)
    travelled = [pose.copy()]
    for tick in range(110):
        if tick % 5 == 0:
            route = detour.plan(pose[:2], (4.0, 0), obstacles, lambda x, y: 2.0)
            assert route
        nearest = np.linalg.norm(np.asarray(route) - pose[:2], axis=1).argmin()
        index = min(nearest + 1, len(route) - 1)
        target = path_lookahead(pose[:2], route, index)
        previous = dwa.plan(pose, target, route, index, obstacles,
                            lambda x, y: 2.0, previous)
        assert previous is not None
        pose = dwa._simulate(pose, previous)[0]
        travelled.append(pose.copy())
        assert np.linalg.norm(obstacles - pose[:2], axis=1).min() >= dwa.config.dynamic_clearance
        if np.linalg.norm(pose[:2] - (4., 0.)) < .3:
            break
    assert pose[0] > 3.7
    assert abs(pose[1]) < .3
    assert np.min(np.asarray(travelled)[:, 1]) < -.65


def test_detour_respects_wall_and_cannot_escape_enclosed_corridor():
    points = surface([(1.5, 0)])
    assert not LocalDetour().plan((0, 0), (3.5, 0), points,
                                 lambda x, y: 1.0 if abs(y) < .4 else -1.0)
    route = LocalDetour().plan((0, 0), (3.5, 0), points,
                              lambda x, y: 1.0 if y > -.3 else -1.0)
    assert route and min(p[1] for p in route) >= -.3
    assert max(p[1] for p in route) > .65


def test_people_yield_without_overlap_or_teleport_and_resume():
    traffic = PedestrianTraffic([(-2., 0.), (0., -2.)])
    sample = lambda i, t: np.array((-2 + .4 * t, 0) if i == 0 else (0, -2 + .4 * t))
    previous = traffic.positions.copy()
    for _ in range(220):
        positions = traffic.step(.05, sample)
        assert swept_separation(previous[0], positions[0], previous[1], positions[1]) >= .85 - 1e-9
        assert np.linalg.norm(positions - previous, axis=1).max() <= .020001
        previous = positions
    assert np.all(traffic.times > 7.)  # both get through the intersection


def test_crossing_continues_when_dog_stops_and_clock_pauses_when_blocked():
    traffic = PedestrianTraffic([(0., 2.)])
    sample = lambda i, t: np.array((0., 2. - .4 * t))
    for _ in range(100):
        traffic.step(.05, sample, robot=np.array((2., 0.)), trigger_distance=3.2)
    assert traffic.positions[0, 1] < .1  # no dependence on dog progress
    traffic = PedestrianTraffic([(0., 2.)])
    for _ in range(100):
        traffic.step(.05, sample, robot=np.zeros(2))
    assert traffic.positions[0, 1] >= .95
    before = traffic.positions.copy()
    traffic.step(10., sample, robot=np.array((3., 0.)))
    assert 0 < np.linalg.norm(traffic.positions - before) <= .040001


def test_crossing_does_not_yield_to_robot_when_robot_gap_is_disabled():
    traffic = PedestrianTraffic([(0., 2.)], robot_gap=0.0)
    sample = lambda i, t: np.array((0., 2. - .4 * t))
    for _ in range(100):
        traffic.step(.05, sample, robot=np.zeros(2), trigger_distance=3.2)
    assert traffic.positions[0, 1] < .1


def test_detour_advances_dense_waypoints_without_turning_back_or_crossing_wall():
    from slam.bridge.dynamic_dwa import detour_path_index
    path = [(float(x), 0.) for x in np.linspace(0, 5, 51)]
    index = detour_path_index((2.0, -.8), path, 5, lambda x, y: 1.0)
    assert 19 <= index <= 21
    target = path_lookahead((2.0, -.8), path, index, 1.2)
    assert target[0] > 2.
    corner = [(0., 0.), (2., 0.), (2., 2.), (0., 2.)]
    # Closest later segment is across a wall: preserve the current segment.
    index = detour_path_index((0., .8), corner, 1,
                             lambda x, y: -1. if .9 < y < 1.1 else 1.)
    assert index == 1
