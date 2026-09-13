#!/usr/bin/env python3.8
"""Four-direction ARIAC shelves-aisle regression with headless path figures.

The test uses the actual ARIAC MuJoCo model and its shelf_1 ... shelf_6 mesh
geometry.  It
checks both sides of the intended aisle independently from the short-range
lidar trigger: a parallel shelf may be visible in the scan, but it must not
be treated as a forward threat unless a return enters the robot's swept body
corridor.  A synthetic blocking return then verifies that the same gate still
activates local DWA for a newly appearing object.
"""
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "slam", "bridge"))

from dynamic_dwa import (  # noqa: E402
    DWAConfig,
    HolonomicDWA,
    path_corridor_clearance,
)


XML = os.path.join(ROOT, "model", "robot", "ariac_lab_with_robot_3d.xml")
RESULT = os.path.join(ROOT, "test", "result", "ariac", "navigation", "shelves")
ROBOT_RADIUS = 0.31
TRIGGER_HALF_WIDTH = ROBOT_RADIUS + 0.12
DYNAMIC_CORRIDOR = 0.62
RANGE_MAX = 8.0
RAY_COUNT = 360
SAMPLE_STEP = 0.20

# The two centre aisles are traversable in both orientations.  They are
# intentionally expressed as endpoints, just like inspection/nav goals.
ROUTES = {
    # The ARIAC racks are arranged in three east-side columns.  x=16.7 is
    # the aisle between shelf_1 and shelf_3; y=8.0 is the cross-aisle between
    # the lower (1/3/5) and upper (2/4/6) rack rows.
    "south_to_north": ((16.7, 0.0), (16.7, 8.0), math.pi / 2.0),
    "north_to_south": ((16.7, 8.0), (16.7, 0.0), -math.pi / 2.0),
    # Stop at x=20.0 before the east wall enters the 3 m local look-ahead;
    # the tested segment still passes the shelf_1/3/5 row and stays in the
    # open cross-aisle rather than turning a boundary wall into the test goal.
    "west_to_east": ((12.0, 8.0), (20.0, 8.0), 0.0),
    "east_to_west": ((20.0, 8.0), (12.0, 8.0), math.pi),
}


def _descendant_bodies(model, root):
    result = set()
    for body in range(model.nbody):
        current = body
        while current > 0 and current != root:
            current = int(model.body_parentid[current])
        if current == root:
            result.add(body)
    return result


class WarehouseScan:
    """Minimal MuJoCo lidar harness matching the project's 2D point usage."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.dog = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self.site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        self.qadr = {
            name: self.model.jnt_qposadr[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in ("base_x", "base_y", "base_yaw")
        }
        self.home = self.data.xpos[self.dog][:2].copy()
        self.robot_geoms = np.asarray([
            geom for geom in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom]) in
            _descendant_bodies(self.model, self.dog)
        ], dtype=np.int32)
        theta = np.linspace(-math.pi, math.pi, RAY_COUNT, endpoint=False)
        self.dirs = np.column_stack((np.cos(theta), np.sin(theta),
                                     np.zeros(RAY_COUNT)))
        self.world_dirs = np.empty_like(self.dirs)
        self.geom_ids = np.zeros(RAY_COUNT, dtype=np.int32)
        self.distances = np.zeros(RAY_COUNT, dtype=np.float64)

    def set_pose(self, x, y, yaw):
        self.data.qpos[self.qadr["base_x"]] = x - self.home[0]
        self.data.qpos[self.qadr["base_y"]] = y - self.home[1]
        self.data.qpos[self.qadr["base_yaw"]] = yaw
        mujoco.mj_forward(self.model, self.data)

    def pose(self):
        return np.asarray([
            self.home[0] + self.data.qpos[self.qadr["base_x"]],
            self.home[1] + self.data.qpos[self.qadr["base_y"]],
            self.data.qpos[self.qadr["base_yaw"]],
        ], dtype=np.float64)

    def scan_points(self):
        yaw = float(self.pose()[2])
        c, s = math.cos(yaw), math.sin(yaw)
        self.world_dirs[:, 0] = c * self.dirs[:, 0] - s * self.dirs[:, 1]
        self.world_dirs[:, 1] = s * self.dirs[:, 0] + c * self.dirs[:, 1]
        self.world_dirs[:, 2] = 0.0
        mujoco.mj_multiRay(
            self.model, self.data, self.data.site_xpos[self.site],
            self.world_dirs.ravel(), None, 1, self.dog,
            self.geom_ids, self.distances, RAY_COUNT, RANGE_MAX)
        valid = ((self.distances > 0.0)
                 & (self.distances <= RANGE_MAX)
                 & ~np.isin(self.geom_ids, self.robot_geoms))
        origin = self.data.site_xpos[self.site][:2].copy()
        return origin + self.world_dirs[valid, :2] * self.distances[valid, None]

    def shelf_rectangles(self):
        """Return axis-aligned XY bounds for ARIAC shelf_1 ... shelf_6."""
        rectangles = []
        for body in range(self.model.nbody):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            if name not in {"shelf_%d" % index for index in range(1, 7)}:
                continue
            descendants = _descendant_bodies(self.model, body)
            geoms = np.where(np.isin(self.model.geom_bodyid, list(descendants)))[0]
            corners = []
            for geom in geoms:
                # ARIAC shelves are mesh geoms.  geom_aabb stores the local
                # mesh bounds; transform all eight corners into world XY.
                bounds = self.model.geom_aabb[geom].reshape(2, 3)
                rot = self.data.geom_xmat[geom].reshape(3, 3)
                centre = self.data.geom_xpos[geom]
                for mask in range(8):
                    local = np.asarray([
                        bounds[1, axis] if mask & (1 << axis)
                        else bounds[0, axis] for axis in range(3)])
                    corners.append((centre + rot.dot(local))[:2])
            xy = np.asarray(corners)
            rectangles.append((name, float(xy[:, 0].min()),
                               float(xy[:, 0].max()),
                               float(xy[:, 1].min()),
                               float(xy[:, 1].max())))
        return rectangles


def _segment_samples(start, goal):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    distance = float(np.linalg.norm(goal - start))
    count = max(1, int(math.ceil(distance / SAMPLE_STEP)))
    return [tuple(start + (goal - start) * i / count)
            for i in range(count + 1)]


def _rect_clearance(point, rect):
    _, xmin, xmax, ymin, ymax = rect
    dx = max(xmin - point[0], 0.0, point[0] - xmax)
    dy = max(ymin - point[1], 0.0, point[1] - ymax)
    return math.hypot(dx, dy)


def _plot_route(name, start, goal, rectangles, samples, ungated, gated):
    fig, ax = plt.subplots(figsize=(10, 7), dpi=150)
    for _, xmin, xmax, ymin, ymax in rectangles:
        ax.add_patch(patches.Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin,
            facecolor="#c69c6d", edgecolor="#5b3a29", alpha=0.72,
            linewidth=0.8))
    xy = np.asarray(samples)
    ax.plot(xy[:, 0], xy[:, 1], color="#1769aa", linewidth=2.2,
            label="planned aisle centreline")
    ax.scatter(*start, color="#16803c", s=36, zorder=3, label="start")
    ax.scatter(*goal, color="#b42318", s=36, zorder=3, label="goal")
    ax.set_title("Shelves corridor: %s\nmin ungated=%.2f m, gated=%.2f m"
                 % (name, ungated, gated))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.22)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULT, "shelves_%s.png" % name))
    plt.close(fig)


def _plot_all(results, rectangles):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), dpi=150)
    for ax, (name, result) in zip(axes.flat, results.items()):
        for _, xmin, xmax, ymin, ymax in rectangles:
            ax.add_patch(patches.Rectangle(
                (xmin, ymin), xmax - xmin, ymax - ymin,
                facecolor="#c69c6d", edgecolor="#5b3a29", alpha=0.72))
        xy = np.asarray(result["samples"])
        ax.plot(xy[:, 0], xy[:, 1], color="#1769aa", linewidth=2)
        ax.scatter(*result["start"], color="#16803c", s=20)
        ax.scatter(*result["goal"], color="#b42318", s=20)
        ax.set_title(name.replace("_", " "))
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.2)
    fig.suptitle("MuJoCo warehouse shelves — four approach directions")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULT, "shelves_all_directions.png"))
    plt.close(fig)


def _synthetic_trigger_check():
    path = [(0.0, 0.0), (4.0, 0.0)]
    side = np.asarray([[1.0, 0.50]])
    # The old centreline-only test would report 0.50 m and trigger.  The
    # swept-body gate intentionally ignores this safe parallel-side return.
    ungated = path_corridor_clearance(side, (0.0, 0.0), path, 1, 3.0)
    gated = path_corridor_clearance(
        side, (0.0, 0.0), path, 1, 3.0, TRIGGER_HALF_WIDTH)
    assert ungated < DYNAMIC_CORRIDOR
    assert math.isinf(gated)

    # A small newly appeared object plus a nearby return represents a person
    # or pallet intruding from one side of the aisle.  Default DWA should
    # choose a lateral component instead of treating the global path as an
    # unconditional straight-line command.
    blocking = np.asarray([[0.65, -0.30], [1.0, 0.0]])
    triggered = path_corridor_clearance(
        blocking, (0.0, 0.0), path, 1, 3.0, TRIGGER_HALF_WIDTH)
    assert triggered < DYNAMIC_CORRIDOR
    dwa = HolonomicDWA(DWAConfig(dynamic_clearance=0.42))
    command = dwa.plan(
        (0.0, 0.0, 0.0), (1.2, 0.0), path, 1, blocking,
        lambda _x, _y: 1.0)
    assert command is not None
    assert abs(command[1]) > 0.05 or abs(command[2]) > 0.10
    return ungated, gated, triggered, command


def main():
    os.makedirs(RESULT, exist_ok=True)
    harness = WarehouseScan()
    rectangles = harness.shelf_rectangles()
    assert len(rectangles) == 6, "expected six ARIAC rack bodies (shelf_1..shelf_6)"

    results = {}
    for name, (start, goal, yaw) in ROUTES.items():
        samples = _segment_samples(start, goal)
        clearances = []
        ungated = []
        gated = []
        for x, y in samples:
            # Geometric swept-body check catches an aisle that is too narrow,
            # independent of lidar sampling density.
            clearances.append(min(_rect_clearance((x, y), rect)
                                  for rect in rectangles) - ROBOT_RADIUS)
            harness.set_pose(x, y, yaw)
            points = harness.scan_points()
            ungated.append(path_corridor_clearance(
                points, (x, y), [start, goal], 1, 3.0))
            gated.append(path_corridor_clearance(
                points, (x, y), [start, goal], 1, 3.0,
                TRIGGER_HALF_WIDTH))
        min_clearance = float(min(clearances))
        min_ungated = float(min(ungated))
        min_gated = float(min(gated))
        # The aisle centreline must retain a healthy margin.  A gated scan is
        # allowed to be inf when no point is inside the forward swept band.
        assert min_clearance > 0.08, "%s shelf clearance %.3f m" % (
            name, min_clearance)
        assert min_gated > DYNAMIC_CORRIDOR, (
            "%s: parallel shelf entered gated threat corridor (%.3f m)" %
            (name, min_gated))
        results[name] = {
            "start": list(start), "goal": list(goal),
            "samples": [list(p) for p in samples],
            "min_robot_clearance_m": min_clearance,
            "min_ungated_clearance_m": min_ungated,
            "min_gated_clearance_m": min_gated,
            "gated_triggered": False,
        }
        _plot_route(name, start, goal, rectangles, samples,
                    min_ungated, min_gated)

    synthetic = _synthetic_trigger_check()
    payload = {
        "xml": XML,
        "robot_radius_m": ROBOT_RADIUS,
        "trigger_half_width_m": TRIGGER_HALF_WIDTH,
        "dynamic_corridor_threshold_m": DYNAMIC_CORRIDOR,
        "routes": results,
        "synthetic_side_return": {
            "ungated_m": synthetic[0], "gated_m": synthetic[1],
            "blocking_gated_m": synthetic[2],
            "dwa_command": list(synthetic[3]),
        },
    }
    with open(os.path.join(RESULT, "shelves_metrics.json"), "w",
              encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
    with open(os.path.join(RESULT, "shelves_metrics.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "route", "min_robot_clearance_m", "min_ungated_clearance_m",
            "min_gated_clearance_m", "gated_triggered"])
        writer.writeheader()
        for name, values in results.items():
            writer.writerow({"route": name, **{
                key: values[key] for key in writer.fieldnames[1:]}})
    with open(os.path.join(RESULT, "summary.md"), "w", encoding="utf-8") as stream:
        stream.write("# Shelves corridor regression\n\n")
        stream.write("四条路径均从两个方向通过货架区域；`gated` 是实际前方扫掠带门控后的净距。\n\n")
        stream.write("| route | robot clearance | ungated lidar | gated lidar |\n|---|---:|---:|---:|\n")
        for name, values in results.items():
            stream.write("| %s | %.3f | %.3f | %.3f |\n" % (
                name, values["min_robot_clearance_m"],
                values["min_ungated_clearance_m"], values["min_gated_clearance_m"]))
        stream.write("\n合成回波验证：侧向 0.50 m 回波由门控忽略（旧中心线逻辑为 %.2f m），正前方阻塞回波触发 DWA，命令为 `(%.2f, %.2f, %.2f)`。\n" % (
            synthetic[0], synthetic[3][0], synthetic[3][1], synthetic[3][2]))

    _plot_all(results, rectangles)
    print("PASS shelves routes=%d min_clearance=%.3fm min_gated=%.3fm results=%s" % (
        len(results), min(v["min_robot_clearance_m"] for v in results.values()),
        min(v["min_gated_clearance_m"] for v in results.values()), RESULT))


if __name__ == "__main__":
    main()
