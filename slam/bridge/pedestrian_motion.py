"""Continuous, yielding mocap traffic, independent of ROS and MuJoCo."""
import numpy as np


def swept_separation(a0, a1, b0, b1):
    """Minimum centre separation during two simultaneous linear steps."""
    delta = np.asarray(a0) - np.asarray(b0)
    velocity = (np.asarray(a1) - a0) - (np.asarray(b1) - b0)
    ratio = np.clip(-np.dot(delta, velocity) /
                    max(float(np.dot(velocity, velocity)), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(delta + ratio * velocity))


class PedestrianTraffic:
    def __init__(self, positions, person_gap=0.85, robot_gap=0.95):
        self.positions = np.asarray(positions, dtype=np.float64).copy()
        self.times = np.zeros(len(positions))
        self.started = np.zeros(len(positions), dtype=bool)
        self.person_gap = person_gap
        self.robot_gap = robot_gap

    def step(self, dt, sample, robot=None, trigger_distance=None):
        # A delayed callback must not fast-forward a mocap body through a dog
        # or another person. Pausing a track pauses its own clock, so resuming
        # never jumps to a distant wall-clock position.
        dt = max(0.0, min(float(dt), 0.10))
        old = self.positions.copy()
        result = old.copy()
        for index, position in enumerate(old):
            if (trigger_distance is None or robot is None
                    or np.linalg.norm(position - robot) <= trigger_distance):
                self.started[index] = True
            if not self.started[index]:
                continue
            candidate = np.asarray(sample(index, self.times[index] + dt))
            # Yield *before* two crossing walkers reach the hard gap. Waiting
            # only at contact distance can strand both at a junction. Lower
            # indices have priority over the next 2.5 seconds of track motion.
            future = np.asarray(sample(index, self.times[index] + 2.5))
            if any(self.started[other] and swept_separation(
                    position, future, old[other],
                    sample(other, self.times[other] + 2.5)) < self.person_gap
                   for other in range(index)):
                continue
            if robot is not None:
                before = float(np.linalg.norm(position - robot))
                after = float(np.linalg.norm(candidate - robot))
                if (swept_separation(position, candidate, robot, robot) < self.robot_gap
                        and after <= before):
                    continue
            # Earlier people have priority. Check their accepted sweep and
            # later people's stationary poses, so a subsequent yield remains
            # safe for every already accepted movement.
            if any(swept_separation(position, candidate, old[other], result[other])
                   < self.person_gap for other in range(len(old)) if other != index):
                continue
            result[index] = candidate
            self.times[index] += dt
        self.positions = result
        return result.copy()
