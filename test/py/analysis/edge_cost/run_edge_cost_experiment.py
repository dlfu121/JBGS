#!/usr/bin/env python3.8
"""Reproducible A/B/C edge-cost experiment on a saved 3D map.

The planner is exercised without starting the ROS navigation loop.  The map,
grid conversion and path semantics are the same as NavP2P; the output plot is
an RViz-equivalent view of planning_map + nav_path for headless environments.
"""
import argparse
import csv
import datetime
import json
import math
import os
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../../.."))
sys.path.insert(0, ROOT)

from nav_p2p import (  # noqa: E402
    HARD_CLEARANCE,
    INFLATE,
    LAMBDA_GEO,
    NavP2P,
    PREFERRED_CLEARANCE,
    ROBOT_RADIUS,
    _inflate_occupancy,
    _pgm_to_occupancy,
    _read_ply_xyz,
    _height_clearance_from_points,
    compute_height_cost,
    HARD_HEIGHT,
    PREFERRED_HEIGHT,
    _line_cells,
)

SCENES = {
    "ariac": {
        "map_stem": "ariac_map_3d",
        # This long diagonal crosses several central ARIAC equipment regions.
        # The distance-only route enters a roughly 0.51 m clearance band,
        # while lambda_geo=2 takes a visibly different, safer route.
        "start_world": (-4.0, -4.0),
        "goal_world": (7.0, 7.0),
    },
    "warehouse": {
        "map_stem": "warehouse_map_3d",
        "start_world": (18.00, 1.00),
        "goal_world": (10.00, 10.00),
    },
}

# The saved ARIAC cloud contains a small connected patch of height-observed
# and 2D-traversable cells near (x≈2.3..3.3, y≈14.4..15.4).  The long report
# route does not cross that patch, so include short diagnostic probe segments
# in the height-profile figure.  They are visualization probes only (not
# mixed into the A/B/C route metrics) and make missing-vs-observed samples
# explicit instead of producing an empty plot.
HEIGHT_PROFILE_PROBES = {
    "ariac": (((239, 550), (259, 570)),
              ((239, 570), (259, 550)),
              ((239, 550), (259, 550))),
}


def _yaml(path):
    values = {}
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if ":" in line and not line.lstrip().startswith("#"):
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
    origin = [float(v) for v in values["origin"].strip("[]").split(",")]
    return float(values["resolution"]), origin


def _load_grid(pgm, yaml_path):
    resolution, origin = _yaml(yaml_path)
    image = np.asarray(Image.open(pgm), dtype=np.uint8)
    image = np.flipud(image)
    occupancy = _pgm_to_occupancy(image, 0.65, 0.25, 0)
    cost, clearance = _inflate_occupancy(
        occupancy.ravel(), image.shape[0], image.shape[1], resolution,
        ROBOT_RADIUS + INFLATE)
    return occupancy, cost, clearance, resolution, origin


def _grid(world, resolution, origin, cost):
    x = int(math.floor((world[0] - origin[0]) / resolution))
    y = int(math.floor((world[1] - origin[1]) / resolution))
    h, w = cost.shape
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError("endpoint outside map: %s" % (world,))
    return x, y


def _nearest_free(point, cost):
    if cost[point[1], point[0]] <= 0:
        return point
    h, w = cost.shape
    for radius in range(1, max(h, w)):
        for y in range(max(0, point[1] - radius), min(h, point[1] + radius + 1)):
            for x in range(max(0, point[0] - radius), min(w, point[0] + radius + 1)):
                if cost[y, x] <= 0:
                    return x, y
    raise ValueError("no free endpoint near %s" % (point,))


def _planner(cost, clearance, resolution, lambda_geo, height_clearance=None,
             lambda_height=0.0):
    planner = NavP2P.__new__(NavP2P)
    planner.cost = cost
    planner.clearance = clearance
    planner.traversable = cost <= 0.0
    planner.grid_shape = cost.shape
    planner.lambda_geo = lambda_geo
    planner.lambda_height = lambda_height
    planner.hard_height = HARD_HEIGHT
    planner.height_clearance = height_clearance
    planner.height_cost = (np.zeros(cost.shape, dtype=np.float64)
                           if height_clearance is None else
                           np.vectorize(compute_height_cost)(height_clearance))
    if height_clearance is not None:
        planner.traversable &= ((~np.isfinite(height_clearance)) |
                                (height_clearance > HARD_HEIGHT))
    planner.map = SimpleNamespace(
        info=SimpleNamespace(resolution=resolution))
    return planner


def _path_cells(path, cost):
    """Return all raster cells crossed by an any-angle waypoint path."""
    cells = []
    for a, b in zip(path, path[1:]):
        edge = _line_cells(cost, a[0], a[1], b[0], b[1]) or []
        if cells and edge and edge[0] == cells[-1]:
            edge = edge[1:]
        cells.extend(edge)
    return cells or list(path)


def _path_metrics(path, cost, clearance, resolution):
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(path, path[1:])) * resolution
    cells = _path_cells(path, cost)
    values = [float(clearance[y, x]) for x, y in cells]
    return length, min(values), sum(values) / len(values), len(cells)


def _path_height_profile(path, cost, height_clearance, resolution):
    cells = _path_cells(path, cost)
    distance = [0.0]
    values = [float(height_clearance[cells[0][1], cells[0][0]])]
    for a, b in zip(cells, cells[1:]):
        distance.append(distance[-1] + math.hypot(b[0] - a[0], b[1] - a[1]) * resolution)
        values.append(float(height_clearance[b[1], b[0]]))
    return np.asarray(distance), np.asarray(values)


def _focus_on_paths(ax, paths, origin, resolution, shape, padding=1.0):
    """Zoom comparison plots to the tested route while retaining context."""
    cells = [cell for path in paths.values() if path for cell in path]
    if not cells:
        return
    xy = np.asarray([
        (origin[0] + (x + 0.5) * resolution,
         origin[1] + (y + 0.5) * resolution)
        for x, y in cells
    ])
    map_x_max = origin[0] + shape[1] * resolution
    map_y_max = origin[1] + shape[0] * resolution
    ax.set_xlim(max(origin[0], float(xy[:, 0].min()) - padding),
                min(map_x_max, float(xy[:, 0].max()) + padding))
    ax.set_ylim(max(origin[1], float(xy[:, 1].min()) - padding),
                min(map_y_max, float(xy[:, 1].max()) + padding))


def _save_height_visuals(result_dir, cost, height_clearance, height_cost,
                         occupancy, origin, resolution, paths, scene):
    extent = (origin[0], origin[0] + occupancy.shape[1] * resolution,
              origin[1], origin[1] + occupancy.shape[0] * resolution)
    layers = ((height_clearance, "height_clearance.png", "Height clearance (m)", "viridis", 0.0, 1.5),
              (height_cost, "height_cost.png", "Height soft cost (0..1)", "magma", 0.0, 1.0),
              ((np.isfinite(height_clearance) & (height_clearance <= HARD_HEIGHT)).astype(float),
               "height_blocked.png", "Hard height obstacle mask", "Reds", 0.0, 1.0))
    for data, name, title, cmap, vmin, vmax in layers:
        fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
        shown = np.ma.masked_invalid(data) if name == "height_clearance.png" else data
        ax.imshow(shown, origin="lower", extent=extent, cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_title(title); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.set_aspect("equal")
        fig.colorbar(ax.images[0], ax=ax); fig.tight_layout()
        fig.savefig(os.path.join(result_dir, name)); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    max_distance = 0.0
    plotted_any = False
    unknown_label_added = False
    for name, path in paths.items():
        if path:
            d, h = _path_height_profile(path, cost, height_clearance, resolution)
            max_distance = max(max_distance, float(d[-1]) if len(d) else 0.0)
            finite = np.isfinite(h)
            if np.any(finite):
                # Break the line at unknown cells rather than joining two
                # unrelated observations across an unobserved interval.
                observed_h = np.where(finite, h, np.nan)
                ax.plot(d, observed_h, linewidth=2, label=name)
                ax.scatter(d[finite], h[finite], s=8, alpha=0.55)
                plotted_any = True
                if np.any(~finite):
                    # Unknown cells are shown at the preferred-height level
                    # with a faint dotted stroke, never as measured data.
                    unknown = np.where(~finite, PREFERRED_HEIGHT, np.nan)
                    label = "unknown (no return)" if not unknown_label_added else "_nolegend_"
                    ax.plot(d, unknown, color="0.55", linestyle=":", linewidth=1,
                            alpha=0.7, label=label)
                    unknown_label_added = True
            else:
                # Keep the route in the legend and report its missing cloud
                # evidence directly on the figure.
                ax.plot([], [], linewidth=2, label=name + " (no height returns)")
    ax.axhline(HARD_HEIGHT, color="red", linestyle="--", label="hard_height")
    ax.axhline(PREFERRED_HEIGHT, color="green", linestyle=":", label="preferred_height")
    if max_distance > 0.0:
        ax.set_xlim(0.0, max_distance * 1.02)
    if not plotted_any:
        ax.text(0.5, 0.5, "No finite height observations on tested paths",
                transform=ax.transAxes, ha="center", va="center")
    ax.set(title="Vertical clearance along planned paths", xlabel="distance (m)", ylabel="clearance (m)")
    ax.grid(alpha=0.25); ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(os.path.join(result_dir, "path_height_profile.png")); plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
    # _load_grid already converts PGM top-left storage to map bottom-left
    # storage, so do not flip a second time here.
    ax.imshow(occupancy, cmap="gray", vmin=0, vmax=100, origin="lower", extent=extent)
    colors = {"height_lambda_0": "#2563eb", "height_lambda_1": "#16a34a", "height_lambda_2": "#dc2626"}
    for name, path in paths.items():
        if path and name.startswith("height_lambda"):
            xy = np.asarray([(origin[0] + (x + .5) * resolution, origin[1] + (y + .5) * resolution) for x, y in path])
            ax.plot(xy[:, 0], xy[:, 1], color=colors[name], linewidth=2, label=name)
    _focus_on_paths(ax, paths, origin, resolution, occupancy.shape)
    ax.set_title("%s height-cost path comparison" % scene.upper()); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.set_aspect("equal"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(result_dir, "height_paths_comparison.png")); plt.close(fig)


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=sorted(SCENES), default="ariac",
                        help="saved map to test (default: ariac)")
    parser.add_argument("--result-dir",
                        help="override test output directory")
    return parser.parse_args()


def main():
    args = _parse_args()
    config = SCENES[args.scene]
    map_dir = os.path.join(ROOT, "maps", args.scene)
    stem = config["map_stem"]
    pgm = os.path.join(map_dir, stem + ".pgm")
    yaml_path = os.path.join(map_dir, stem + ".yaml")
    cloud = os.path.join(map_dir, stem + "_cloud.ply")
    result_dir = (os.path.abspath(args.result_dir) if args.result_dir else
                  os.path.join(ROOT, "test", "result", args.scene,
                               "analysis", "edge_cost"))
    for required in (pgm, yaml_path, cloud):
        if not os.path.isfile(required):
            raise FileNotFoundError("required saved-map input is missing: %s" % required)
    os.makedirs(result_dir, exist_ok=True)
    occupancy, cost, clearance, resolution, origin = _load_grid(pgm, yaml_path)
    points = _read_ply_xyz(cloud)
    height_clearance = _height_clearance_from_points(
        points, cost.shape[0], cost.shape[1], resolution, (origin[0], origin[1]))
    height_cost = np.zeros(cost.shape, dtype=np.float64)
    finite = np.isfinite(height_clearance) & (height_clearance < PREFERRED_HEIGHT)
    height_cost[finite] = np.vectorize(compute_height_cost)(height_clearance[finite])
    height_observed_cells = int(np.count_nonzero(np.isfinite(height_clearance)))
    height_observed_traversable_cells = int(np.count_nonzero(
        np.isfinite(height_clearance) & (cost <= 0.0)))
    start_world = config["start_world"]
    goal_world = config["goal_world"]
    requested_start = _grid(start_world, resolution, origin, cost)
    requested_goal = _grid(goal_world, resolution, origin, cost)
    start = _nearest_free(requested_start, cost)
    goal = _nearest_free(requested_goal, cost)
    if start != requested_start or goal != requested_goal:
        raise ValueError("configured %s endpoint is not traversable: %s -> %s, %s -> %s"
                         % (args.scene, requested_start, start, requested_goal, goal))

    # A is the pre-change distance reference; B is the unified-interface
    # distance mode.  Both must agree when lambda_geo=0.
    cases = [("A_baseline_reference", 0.0),
             ("B_unified_distance", 0.0),
             ("C_safety_aware", 2.0)]
    records = []
    paths = {}
    for name, weight in cases:
        planner = _planner(cost, clearance, resolution, weight)
        path = planner._astar(start[0], start[1], goal[0], goal[1])
        stats = dict(planner.last_plan_stats)
        if path is not None:
            length, min_c, mean_c, sampled = _path_metrics(
                path, cost, clearance, resolution)
            paths[name] = path
        else:
            length, min_c, mean_c, sampled = (None, None, None, 0)
        stats.update(case=name, lambda_geo=weight, start=list(start), goal=list(goal),
                     path_length_m=length, minimum_clearance_m=min_c,
                     mean_clearance_m=mean_c, sampled_path_cells=sampled)
        records.append(stats)

    height_paths = {}
    height_records = []
    for weight in (0.0, 1.0, 2.0):
        name = "height_lambda_%g" % weight
        planner = _planner(cost, clearance, resolution, 0.0,
                           height_clearance, weight)
        path = planner._astar(start[0], start[1], goal[0], goal[1])
        height_paths[name] = path
        stats = dict(planner.last_plan_stats, case=name,
                     lambda_height=weight, start=list(start), goal=list(goal))
        if path:
            length, min_c, mean_c, sampled = _path_metrics(
                path, cost, clearance, resolution)
            _, profile = _path_height_profile(
                path, cost, height_clearance, resolution)
            finite_profile = profile[np.isfinite(profile)]
            stats.update(path_length_m=length, minimum_clearance_m=min_c,
                         mean_clearance_m=mean_c,
                         sampled_path_cells=sampled,
                         minimum_height_clearance_m=(
                             float(np.min(finite_profile))
                             if finite_profile.size else None))
        else:
            stats.update(path_length_m=None, minimum_clearance_m=None,
                         mean_clearance_m=None, sampled_path_cells=0,
                         minimum_height_clearance_m=None)
        height_records.append(stats)

    # Add local height-observation probes to the profile visualization.  The
    # probes use the same height-aware planner and raster metrics, but remain
    # separate from the requested long-route benchmark records.
    for probe_index, (probe_start, probe_goal) in enumerate(
            HEIGHT_PROFILE_PROBES.get(args.scene, ()), 1):
        probe_name = "height_probe_%d" % probe_index
        probe_planner = _planner(cost, clearance, resolution, 0.0,
                                 height_clearance, 1.0)
        height_paths[probe_name] = probe_planner._astar(
            probe_start[0], probe_start[1], probe_goal[0], probe_goal[1])

    baseline = records[0]
    safety = records[-1]
    comparison = {
        "baseline_matches_unified_distance": all(
            records[0].get(key) == records[1].get(key)
            for key in ("success", "path_length_m", "minimum_clearance_m",
                        "mean_clearance_m", "sampled_path_cells")),
        "safety_path_length_increase_percent": (
            100.0 * (safety["path_length_m"] / baseline["path_length_m"] - 1.0)),
        "safety_minimum_clearance_increase_percent": (
            100.0 * (safety["minimum_clearance_m"] /
                     baseline["minimum_clearance_m"] - 1.0)),
    }
    height_interpretation = (
        "independent height evidence is available on traversable cells"
        if height_observed_traversable_cells else
        "no independent height evidence on traversable cells; the 2D occupancy "
        "and inflation layer already blocks every height-observed cell")
    report = {"scene": args.scene,
                   "generated_at": datetime.datetime.now().astimezone().isoformat(),
                   "map": pgm, "cloud": cloud, "resolution": resolution,
                   "origin": origin, "start_world": start_world,
                   "goal_world": goal_world, "start_grid": start,
                   "goal_grid": goal, "hard_clearance_m": HARD_CLEARANCE,
                   "preferred_clearance_m": PREFERRED_CLEARANCE,
                   "cases": records, "height_cases": height_records,
                   "comparison": comparison,
                   "hard_height_m": HARD_HEIGHT,
                   "preferred_height_m": PREFERRED_HEIGHT,
                   "height_observed_cells": height_observed_cells,
                   "height_observed_traversable_cells": height_observed_traversable_cells,
                   "height_case_interpretation": height_interpretation}
    with open(os.path.join(result_dir, "metrics.json"), "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    fields = ["experiment", "case", "lambda_geo", "lambda_height",
              "success", "planning_time_ms",
              "path_length_m", "minimum_clearance_m", "mean_clearance_m",
              "minimum_height_clearance_m", "sampled_path_cells",
              "expanded_nodes", "los_checks", "los_cells_checked",
              "waypoint_count"]
    with open(os.path.join(result_dir, "metrics.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for experiment, experiment_records in (("horizontal_clearance", records),
                                                ("height_clearance", height_records)):
            for record in experiment_records:
                row = {key: record.get(key) for key in fields}
                row["experiment"] = experiment
                writer.writerow(row)

    # RViz-equivalent path view: grayscale planning map and map-frame paths.
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
    ax.imshow(occupancy, cmap="gray", vmin=0, vmax=100,
              origin="lower", extent=(origin[0], origin[0] + occupancy.shape[1] * resolution,
                                       origin[1], origin[1] + occupancy.shape[0] * resolution))
    styles = [("A_baseline_reference", "#2563eb", "--"),
              ("B_unified_distance", "#16a34a", "-"),
              ("C_safety_aware", "#dc2626", "-")]
    for name, color, linestyle in styles:
        path = paths.get(name)
        if not path:
            continue
        xy = np.asarray([(origin[0] + (x + 0.5) * resolution,
                          origin[1] + (y + 0.5) * resolution) for x, y in path])
        ax.plot(xy[:, 0], xy[:, 1], linestyle=linestyle, color=color,
                linewidth=2.0, label=name)
    ax.scatter([start_world[0]], [start_world[1]], c="cyan", edgecolors="black",
               s=55, zorder=5, label="start")
    ax.scatter([goal_world[0]], [goal_world[1]], c="magenta", edgecolors="black",
               s=55, zorder=5, label="goal")
    _focus_on_paths(ax, paths, origin, resolution, occupancy.shape)
    ax.set_title("%s Lazy Theta* edge-cost comparison (RViz-equivalent)" % args.scene.upper())
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(result_dir, "paths_comparison.png"))
    plt.close(fig)

    _save_height_visuals(result_dir, cost, height_clearance, height_cost,
                         occupancy, origin, resolution,
                         dict(paths, **height_paths), args.scene)

    with open(os.path.join(result_dir, "screenshot_note.txt"), "w", encoding="utf-8") as stream:
        stream.write("No DISPLAY/RViz session was available during this run.\n")
        stream.write("paths_comparison.png is an equivalent map-frame view of planning_map + nav_path.\n")
    with open(os.path.join(result_dir, "summary.md"), "w", encoding="utf-8") as stream:
        stream.write("# %s edge-cost comparison\n\n" % args.scene.upper())
        stream.write("Map: `%s`  \n" % os.path.relpath(pgm, ROOT))
        stream.write("Route: `%s -> %s` (map frame)  \n\n" %
                     (start_world, goal_world))
        stream.write("| Case | lambda_geo | Success | Length (m) | Min clearance (m) | Mean clearance (m) | Planning (ms) |\n")
        stream.write("| --- | ---: | :---: | ---: | ---: | ---: | ---: |\n")
        for record in records:
            stream.write("| {case} | {lambda_geo:.1f} | {success} | {path_length_m:.3f} | {minimum_clearance_m:.3f} | {mean_clearance_m:.3f} | {planning_time_ms:.2f} |\n".format(**record))
        stream.write("\nSafety-aware path length change: `{:.2f}%`; minimum clearance change: `{:.2f}%`.  \n".format(
            comparison["safety_path_length_increase_percent"],
            comparison["safety_minimum_clearance_increase_percent"]))
        stream.write("Baseline/unified-distance equivalent: `{}`.  \n\n".format(
            comparison["baseline_matches_unified_distance"]))
        stream.write("## Height evidence\n\n")
        stream.write("Observed cells: `{}`; observed and 2D-traversable cells: `{}`.  \n".format(
            height_observed_cells, height_observed_traversable_cells))
        stream.write("Result: {}.  \n\n".format(height_interpretation))
        stream.write("Metrics sample every raster cell crossed by each any-angle edge.\n")
    print(json.dumps({"scene": args.scene, "result_dir": result_dir,
                      "start_grid": start, "goal_grid": goal,
                      "cases": records}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
