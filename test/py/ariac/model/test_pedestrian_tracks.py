"""Validate the demonstration lanes against the saved ARIAC geometry."""
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from slam.bridge import bridge_core


def test_ariac_tracks_are_continuous_disjoint_and_clear_of_equipment(monkeypatch):
    monkeypatch.setattr(bridge_core, 'SCENE_NAME', 'ariac')
    bridge = bridge_core.SlamBridge3D.__new__(bridge_core.SlamBridge3D)
    tracks = bridge._build_person_tracks()
    image = cv2.imread(str(ROOT / 'maps/ariac/ariac_map_3d.pgm'), 0)[::-1]
    clearance = distance_transform_edt(image >= 250) * .05
    samples = []
    for track in tracks:
        stop_time = track.get('stop_time')
        horizon = stop_time if stop_time is not None else track['period']
        times = np.linspace(0, horizon, 2001)
        positions = np.asarray([bridge._person_position(track, t) for t in times])
        if stop_time is None:
            assert np.allclose(positions[0], positions[-1])
        else:
            # One-shot lane: it holds the stop waypoint instead of looping.
            assert np.allclose(
                bridge._person_position(track, stop_time + 30.0), positions[-1])
        assert np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1)
                      / np.diff(times)) < track['speed_mps'] * 1.6
        ix = np.floor((positions[:, 0] + 9.6502943) / .05).astype(int)
        iy = np.floor((positions[:, 1] + 13.1527195) / .05).astype(int)
        # Pedestrian torso radius is .20 m; reserve extra geometry clearance.
        assert clearance[iy, ix].min() >= .34
        samples.append(positions[::10])
    # Different phase offsets/dwell durations cannot bring the new lanes
    # together: compare every point on one lane to every point on the other.
    assert len(samples) == 2
    for i in range(2):
        for j in range(i):
            assert np.linalg.norm(samples[i][:, None, :] - samples[j][None, :, :],
                                  axis=2).min() > .85


def test_second_person_steps_into_and_stops_in_front_of_the_dog(monkeypatch):
    """The red walker enters the dog's lane from the side, then holds there."""
    monkeypatch.setattr(bridge_core, 'SCENE_NAME', 'ariac')
    bridge = bridge_core.SlamBridge3D.__new__(bridge_core.SlamBridge3D)
    tracks = bridge._build_person_tracks()
    second = tracks[1]
    start = bridge._person_position(second, 0.0)

    # Tank -> hydrant is the east/south leg.  The dog's planner lane is the
    # straight segment between the stops, so at x=1.0 the corridor lies near
    # y=-8.96.  The walker spawns below that lane (out of the swept band) and
    # steps north onto it, so it first moves toward (not along) the corridor.
    assert start[0] == 1.0
    assert -9.9 < start[1] < -9.5          # ~0.75 m below the lane
    assert np.allclose(start, (1.0, -9.71), atol=1e-6)
    assert bridge._person_position(second, 0.5)[1] > start[1]  # climbing

    # Once on the lane the walker stops and stays parked in front of the dog
    # as a stationary obstacle the local planner must avoid.
    front = np.asarray((1.0, -8.96))
    for t in (second['stop_time'], 5.0, 20.0, 60.0):
        assert np.allclose(bridge._person_position(second, t), front, atol=1e-6)


def test_second_person_holds_on_the_dog_corridor(monkeypatch):
    """The red walker stops on the tank->hydrant lane in front of the dog."""
    monkeypatch.setattr(bridge_core, 'SCENE_NAME', 'ariac')
    bridge = bridge_core.SlamBridge3D.__new__(bridge_core.SlamBridge3D)
    track = bridge._build_person_tracks()[1]
    times = np.linspace(0.0, track['stop_time'], 1001)
    positions = np.asarray([bridge._person_position(track, t) for t in times])

    # The straight tank->hydrant corridor: y(x) = -8.1 - (1.9/14.65)*(x+5.65).
    xs = positions[:, 0]
    corridor_y = -8.1 - (1.9 / 14.65) * (xs + 5.65)
    lateral = np.abs(positions[:, 1] - corridor_y)
    # The side ingress ends on the lane, where the walker then holds.
    assert lateral.min() < 0.25
    assert np.allclose(positions[-1], (1.0, -8.96), atol=1e-6)
