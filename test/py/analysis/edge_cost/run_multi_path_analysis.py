#!/usr/bin/env python3.8
"""Batch route benchmark for horizontal and vertical map costs on ARIAC.

This is deliberately offline: it uses the same saved PGM/PLY and Lazy Theta*
implementation as ``run_edge_cost_experiment.py``.  Every route is evaluated
with the same endpoint and raster metrics, making the output suitable for a
paper/report rather than a single illustrative path.
"""
import argparse
import csv
import datetime
import importlib.util
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../../.."))
BASE_PATH = os.path.join(HERE, "run_edge_cost_experiment.py")
SPEC = importlib.util.spec_from_file_location("edge_cost_base", BASE_PATH)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)

# Endpoints were selected in eight different open regions of the ARIAC map.
# They include long diagonals through equipment, cross-room routes, and a route
# with one endpoint deliberately close to a low-clearance band.
ROUTES = [
    ("R01_central_diagonal", (-4.0, -4.0), (7.0, 7.0)),
    ("R02_south_to_north_east", (-5.0, -9.0), (14.4, 10.8)),
    ("R03_east_to_north_west", (14.0, -9.0), (-5.0, 8.0)),
    ("R04_west_crossing", (-8.0, 0.8), (9.0, -1.6)),
    ("R05_upper_crossing", (-5.0, 13.0), (16.0, 3.0)),
    ("R06_inner_diagonal", (1.0, -6.0), (-5.0, 8.0)),
    ("R07_low_clearance_start", (14.0, -4.0), (-3.0, 3.0)),
    ("R08_outer_diagonal", (-8.0, 13.0), (16.8, -9.1)),
]

METHODS = [
    ("distance", "Distance only", 0.0, 0.0, "#2563eb"),
    ("geo_lambda_1", "Horizontal safety λg=1", 1.0, 0.0, "#16a34a"),
    ("geo_lambda_2", "Horizontal safety λg=2", 2.0, 0.0, "#dc2626"),
    ("height_lambda_1", "Height cost λh=1", 0.0, 1.0, "#9333ea"),
    ("height_lambda_2", "Height cost λh=2", 0.0, 2.0, "#f59e0b"),
]


def _world_xy(path, origin, resolution):
    return np.asarray([(origin[0] + (x + .5) * resolution,
                        origin[1] + (y + .5) * resolution) for x, y in path])


def _percent(new, old):
    if old in (None, 0) or new is None:
        return None
    return 100.0 * (new / old - 1.0)


def _run(args):
    scene = BASE.SCENES["ariac"]
    map_dir = os.path.join(ROOT, "maps", "ariac")
    stem = scene["map_stem"]
    pgm = os.path.join(map_dir, stem + ".pgm")
    yaml_path = os.path.join(map_dir, stem + ".yaml")
    cloud = os.path.join(map_dir, stem + "_cloud.ply")
    result_dir = os.path.abspath(args.result_dir or
                                 os.path.join(ROOT, "test", "result", "ariac",
                                              "analysis", "edge_cost", "multi_path"))
    os.makedirs(result_dir, exist_ok=True)

    occupancy, cost, clearance, resolution, origin = BASE._load_grid(pgm, yaml_path)
    points = BASE._read_ply_xyz(cloud)
    height_clearance = BASE._height_clearance_from_points(
        points, cost.shape[0], cost.shape[1], resolution, (origin[0], origin[1]))
    height_cost = np.zeros(cost.shape, dtype=np.float64)
    finite_soft = np.isfinite(height_clearance) & (height_clearance < BASE.PREFERRED_HEIGHT)
    if np.any(finite_soft):
        height_cost[finite_soft] = np.vectorize(BASE.compute_height_cost)(height_clearance[finite_soft])
    observed = int(np.count_nonzero(np.isfinite(height_clearance)))
    observed_traversable = int(np.count_nonzero(np.isfinite(height_clearance) & (cost <= 0.0)))

    # Resolve all endpoints once so each method sees exactly the same cells.
    route_cells = {}
    for route_id, start_world, goal_world in ROUTES:
        requested_start = BASE._grid(start_world, resolution, origin, cost)
        requested_goal = BASE._grid(goal_world, resolution, origin, cost)
        start = BASE._nearest_free(requested_start, cost)
        goal = BASE._nearest_free(requested_goal, cost)
        route_cells[route_id] = dict(start=start, goal=goal,
                                     requested_start=requested_start,
                                     requested_goal=requested_goal,
                                     start_world=start_world, goal_world=goal_world)

    records = []
    path_store = {}
    for route_id, *_ in ROUTES:
        info = route_cells[route_id]
        path_store[route_id] = {}
        for method, label, lambda_geo, lambda_height, color in METHODS:
            print("planning %s / %s" % (route_id, method), flush=True)
            use_height = lambda_height > 0.0
            planner = BASE._planner(cost, clearance, resolution, lambda_geo,
                                    height_clearance if use_height else None,
                                    lambda_height)
            path = planner._astar(info["start"][0], info["start"][1],
                                  info["goal"][0], info["goal"][1])
            stats = dict(planner.last_plan_stats)
            if path:
                length, min_c, mean_c, sampled = BASE._path_metrics(
                    path, cost, clearance, resolution)
                _, profile = BASE._path_height_profile(path, cost, height_clearance, resolution)
                finite_profile = profile[np.isfinite(profile)]
                min_h = float(np.min(finite_profile)) if finite_profile.size else None
                path_store[route_id][method] = path
            else:
                length = min_c = mean_c = min_h = None
                sampled = 0
                path_store[route_id][method] = None
            stats.update(route_id=route_id, method=method, label=label,
                         lambda_geo=lambda_geo, lambda_height=lambda_height,
                         start=list(info["start"]), goal=list(info["goal"]),
                         start_world=list(info["start_world"]), goal_world=list(info["goal_world"]),
                         path_length_m=length, minimum_clearance_m=min_c,
                         mean_clearance_m=mean_c, minimum_height_clearance_m=min_h,
                         sampled_path_cells=sampled)
            records.append(stats)

    fields = ["route_id", "method", "label", "lambda_geo", "lambda_height", "success",
              "planning_time_ms", "path_length_m", "minimum_clearance_m",
              "mean_clearance_m", "minimum_height_clearance_m", "sampled_path_cells",
              "expanded_nodes", "los_checks", "los_cells_checked", "waypoint_count",
              "start_world", "goal_world"]
    with open(os.path.join(result_dir, "multi_path_metrics.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in records:
            writer.writerow({key: row.get(key) for key in fields})

    # Aggregate statistics and per-route safety improvements relative to distance.
    aggregates = []
    for method, label, lambda_geo, lambda_height, color in METHODS:
        rows = [r for r in records if r["method"] == method]
        successful = [r for r in rows if r.get("success")]
        def mean(key):
            values = [r[key] for r in successful if r.get(key) is not None]
            return float(np.mean(values)) if values else None
        aggregates.append(dict(method=method, label=label, lambda_geo=lambda_geo,
                               lambda_height=lambda_height,
                               success_rate=len(successful) / len(rows),
                               mean_length_m=mean("path_length_m"),
                               mean_min_clearance_m=mean("minimum_clearance_m"),
                               mean_clearance_m=mean("mean_clearance_m"),
                               mean_height_clearance_m=mean("minimum_height_clearance_m"),
                               mean_planning_time_ms=mean("planning_time_ms"),
                               mean_expanded_nodes=mean("expanded_nodes")))
    baseline_by_route = {r["route_id"]: r for r in records if r["method"] == "distance"}
    improvements = []
    for r in records:
        if r["method"] == "distance":
            continue
        b = baseline_by_route[r["route_id"]]
        improvements.append(dict(route_id=r["route_id"], method=r["method"],
                                 length_change_percent=_percent(r.get("path_length_m"), b.get("path_length_m")),
                                 min_clearance_change_percent=_percent(r.get("minimum_clearance_m"), b.get("minimum_clearance_m")),
                                 planning_time_change_percent=_percent(r.get("planning_time_ms"), b.get("planning_time_ms"))))

    report = dict(scene="ariac", generated_at=datetime.datetime.now().astimezone().isoformat(),
                  map=pgm, cloud=cloud, resolution=resolution, origin=origin,
                  map_shape=list(cost.shape), occupied_cells=int(np.count_nonzero(cost > 0)),
                  traversable_cells=int(np.count_nonzero(cost <= 0)),
                  height_observed_cells=observed,
                  height_observed_traversable_cells=observed_traversable,
                  hard_height_m=BASE.HARD_HEIGHT, preferred_height_m=BASE.PREFERRED_HEIGHT,
                  routes=route_cells, methods=[m[:4] for m in METHODS],
                  records=records, aggregates=aggregates, improvements=improvements)
    with open(os.path.join(result_dir, "multi_path_metrics.json"), "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)

    extent = (origin[0], origin[0] + occupancy.shape[1] * resolution,
              origin[1], origin[1] + occupancy.shape[0] * resolution)
    # Eight small multiples make route changes readable even when paths overlap.
    fig, axes = plt.subplots(2, 4, figsize=(20, 10), dpi=150)
    for ax, (route_id, start_world, goal_world) in zip(axes.flat, ROUTES):
        ax.imshow(occupancy, cmap="gray", vmin=0, vmax=100, origin="lower", extent=extent)
        for method, label, _, _, color in METHODS:
            path = path_store[route_id][method]
            if path:
                xy = _world_xy(path, origin, resolution)
                ax.plot(xy[:, 0], xy[:, 1], color=color, lw=1.5, label=method)
        ax.scatter([start_world[0]], [start_world[1]], c="cyan", s=20, edgecolors="black")
        ax.scatter([goal_world[0]], [goal_world[1]], c="magenta", s=20, edgecolors="black")
        ax.set_title(route_id.replace("_", "\n"), fontsize=8); ax.set_aspect("equal")
        ax.set_xlim(min(start_world[0], goal_world[0]) - 1.2, max(start_world[0], goal_world[0]) + 1.2)
        ax.set_ylim(min(start_world[1], goal_world[1]) - 1.2, max(start_world[1], goal_world[1]) + 1.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, fontsize=9)
    fig.suptitle("ARIAC multi-route path comparison (8 routes)")
    fig.tight_layout(rect=(0, 0.06, 1, 0.95)); fig.savefig(os.path.join(result_dir, "multi_paths_grid.png")); plt.close(fig)

    # Pareto-style scatter: path length versus minimum horizontal clearance.
    fig, ax = plt.subplots(figsize=(10, 7), dpi=150)
    for method, label, _, _, color in METHODS:
        rows = [r for r in records if r["method"] == method and r.get("success")]
        ax.scatter([r["path_length_m"] for r in rows], [r["minimum_clearance_m"] for r in rows],
                   s=55, color=color, label=label, alpha=.85)
    ax.set(xlabel="path length (m)", ylabel="minimum horizontal clearance (m)",
           title="Safety/efficiency trade-off across routes"); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(result_dir, "length_clearance_scatter.png")); plt.close(fig)

    # Grouped bars for the main report numbers.
    x = np.arange(len(METHODS)); width = .36
    fig, axes = plt.subplots(1, 3, figsize=(17, 5), dpi=150)
    for i, (key, title, ylabel) in enumerate((("mean_length_m", "Mean path length", "m"),
                                               ("mean_min_clearance_m", "Mean minimum clearance", "m"),
                                               ("mean_planning_time_ms", "Mean planning time", "ms"))):
        vals = [a[key] or 0.0 for a in aggregates]
        axes[i].bar(x, vals, color=[m[4] for m in METHODS])
        axes[i].set_xticks(x); axes[i].set_xticklabels([m[0].replace("_", "\n") for m in METHODS], fontsize=8)
        axes[i].set_title(title); axes[i].set_ylabel(ylabel); axes[i].grid(axis="y", alpha=.25)
    fig.suptitle("Aggregate result over 8 routes (n=8 per method)")
    fig.tight_layout(); fig.savefig(os.path.join(result_dir, "aggregate_metrics.png")); plt.close(fig)

    # Height evidence map and path height profiles, with one panel per route.
    fig, axes = plt.subplots(2, 4, figsize=(20, 9), dpi=150)
    for ax, (route_id, *_rest) in zip(axes.flat, ROUTES):
        plotted_any = False
        finite_count = 0
        max_distance = 0.0
        for method, label, _, _, color in METHODS:
            path = path_store[route_id][method]
            if path:
                d, h = BASE._path_height_profile(path, cost, height_clearance, resolution)
                max_distance = max(max_distance, float(d[-1]) if len(d) else 0.0)
                finite = np.isfinite(h)
                finite_count += int(np.count_nonzero(finite))
                if np.any(finite):
                    # Do not connect samples across cells without height
                    # evidence; those gaps are genuinely unknown, not zero.
                    ax.plot(d, np.where(finite, h, np.nan), color=color,
                            lw=1.2, label=method)
                    ax.scatter(d[finite], h[finite], s=7, color=color,
                               alpha=0.55)
                    plotted_any = True
                else:
                    # Keep the method in the legend while making the reason
                    # for an apparently empty panel explicit.
                    ax.plot([], [], color=color, lw=1.2,
                            label=method + " (no height returns)")
        ax.axhline(BASE.HARD_HEIGHT, color="black", ls="--", lw=.8)
        ax.axhline(BASE.PREFERRED_HEIGHT, color="gray", ls=":", lw=.8)
        if not plotted_any:
            ax.text(0.5, 0.5, "No finite height observations\non this route",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=9, color="0.35")
        ax.text(0.02, 0.04, "finite samples: %d" % finite_count,
                transform=ax.transAxes, fontsize=7, color="0.35")
        if max_distance > 0.0:
            ax.set_xlim(0.0, max_distance * 1.02)
        ax.set_title(route_id.replace("_", "\n"), fontsize=8); ax.set_xlabel("distance (m)"); ax.set_ylabel("vertical clearance (m)"); ax.grid(alpha=.2)
    handles, labels = axes.flat[0].get_legend_handles_labels(); fig.legend(handles, labels, loc="lower center", ncol=5, fontsize=8)
    fig.suptitle("Height-clearance profiles sampled along every planned edge")
    fig.tight_layout(rect=(0, 0.06, 1, 0.95)); fig.savefig(os.path.join(result_dir, "height_profiles_grid.png")); plt.close(fig)

    # A compact machine-readable summary for quick inspection.
    with open(os.path.join(result_dir, "summary.md"), "w", encoding="utf-8") as stream:
        stream.write("# ARIAC multi-route cost-map benchmark\n\n")
        stream.write("8 fixed routes × 5 methods = **%d planning trials**. All methods use the same saved PGM/PLY and endpoint cells.\n\n" % len(records))
        stream.write("| Method | Success | Mean length (m) | Mean min clearance (m) | Mean height clearance (m) | Mean planning (ms) |\n|---|---:|---:|---:|---:|---:|\n")
        for a in aggregates:
            values = ["—" if a[k] is None else "%.3f" % a[k]
                      for k in ("mean_length_m", "mean_min_clearance_m", "mean_height_clearance_m")]
            planning = "—" if a["mean_planning_time_ms"] is None else "%.1f" % a["mean_planning_time_ms"]
            stream.write("| %s | %.0f%% | %s | %s | %s | %s |\n" % (a["label"], 100*a["success_rate"], values[0], values[1], values[2], planning))
        stream.write("\nHeight map evidence: %d observed cells, %d also 2D-traversable cells; hard height %.2f m, preferred %.2f m.\n" % (observed, observed_traversable, BASE.HARD_HEIGHT, BASE.PREFERRED_HEIGHT))
        stream.write("\nFiles: `multi_path_metrics.csv`, `multi_path_metrics.json`, `multi_paths_grid.png`, `length_clearance_scatter.png`, `aggregate_metrics.png`, `height_profiles_grid.png`.\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", help="output directory")
    args = parser.parse_args()
    report = _run(args)
    result_dir = os.path.abspath(args.result_dir) if args.result_dir else os.path.join(
        ROOT, "test", "result", "ariac", "analysis", "edge_cost", "multi_path")
    print(json.dumps({"result_dir": result_dir,
                      "trials": len(report["records"]), "aggregates": report["aggregates"]}, indent=2))


if __name__ == "__main__":
    main()
