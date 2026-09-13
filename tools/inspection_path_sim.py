#!/usr/bin/env python3
"""Offline multi-run planner simulation and inspection-route visualisation.

The program uses the saved ARIAC occupancy map and the inspection route, so it
does not require ROS or a running simulator.  It plans an A* path for every
leg (start -> cabinet -> tank -> hydrant -> start), renders each run, and
combines the renders into a contact sheet suitable for a technical report.
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def read_pgm(path: Path):
    with Image.open(path) as im:
        return np.asarray(im.convert("L"))


def world_to_rc(x, y, shape, origin, resolution):
    col = int(round((x - origin[0]) / resolution))
    row = shape[0] - 1 - int(round((y - origin[1]) / resolution))
    return row, col


def rc_to_world(row, col, shape, origin, resolution):
    return (origin[0] + col * resolution,
            origin[1] + (shape[0] - 1 - row) * resolution)


def astar(blocked, start, goal, rng):
    """8-connected A* with a tiny seeded cost perturbation for path variety."""
    h, w = blocked.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and
            0 <= goal[0] < h and 0 <= goal[1] < w):
        raise ValueError("start or goal is outside the map")
    # Snap a requested pose to its nearest free cell when it falls on an
    # occupied pixel because of map discretisation.
    def nearest_free(p):
        if not blocked[p]:
            return p
        for radius in range(1, 40):
            for dr in range(-radius, radius + 1):
                for dc in (-radius, radius):
                    q = (p[0] + dr, p[1] + dc)
                    if 0 <= q[0] < h and 0 <= q[1] < w and not blocked[q]:
                        return q
            for dc in range(-radius + 1, radius):
                for dr in (-radius, radius):
                    q = (p[0] + dr, p[1] + dc)
                    if 0 <= q[0] < h and 0 <= q[1] < w and not blocked[q]:
                        return q
        raise RuntimeError("no free cell near requested pose")
    start, goal = nearest_free(start), nearest_free(goal)
    moves = [(dr, dc, math.hypot(dr, dc))
             for dr in (-1, 0, 1) for dc in (-1, 0, 1)
             if (dr, dc) != (0, 0)]
    def heuristic(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1])
    frontier = [(heuristic(start), 0.0, start)]
    came = {}
    cost = {start: 0.0}
    while frontier:
        _, g, current = heapq.heappop(frontier)
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(came[path[-1]])
            return path[::-1]
        if g > cost.get(current, float("inf")) + 1e-12:
            continue
        for dr, dc, step in moves:
            nxt = (current[0] + dr, current[1] + dc)
            if not (0 <= nxt[0] < h and 0 <= nxt[1] < w) or blocked[nxt]:
                continue
            # Seeded noise changes equal-cost choices but is too small to make
            # the planner prefer a materially longer route.
            new_cost = g + step * (1.0 + rng.random() * 0.006)
            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                came[nxt] = current
                heapq.heappush(frontier, (new_cost + heuristic(nxt), new_cost, nxt))
    raise RuntimeError("goal is unreachable")


def inflate(gray, radius_cells):
    # map_server's ``negate: 0`` convention: dark pixels are occupied,
    # white pixels are free and 205 is the unknown trinary value.
    occupied = (gray < 89) | (gray == 205)
    # A small pure-numpy dilation avoids requiring ROS map tooling.
    blocked = occupied.copy()
    yy, xx = np.ogrid[-radius_cells:radius_cells + 1,
                       -radius_cells:radius_cells + 1]
    disk = xx * xx + yy * yy <= radius_cells * radius_cells
    ys, xs = np.where(disk)
    out = np.zeros_like(blocked)
    for dy, dx in zip(ys - radius_cells, xs - radius_cells):
        y0, y1 = max(0, dy), min(blocked.shape[0], blocked.shape[0] + dy)
        x0, x1 = max(0, dx), min(blocked.shape[1], blocked.shape[1] + dx)
        out[y0:y1, x0:x1] |= blocked[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=6, help="number of simulations")
    ap.add_argument("--seed", type=int, default=20260909)
    ap.add_argument("--map", type=Path, default=Path("maps/ariac/ariac_map_3d.pgm"))
    ap.add_argument("--route", type=Path, default=Path("_ultimate_task/inspect/inspection_route.json"))
    ap.add_argument("--out-dir", type=Path, default=Path("inspection_report/path_simulation"))
    ap.add_argument("--inflation-m", type=float, default=0.45)
    args = ap.parse_args()
    if args.runs < 1:
        ap.error("--runs must be positive")
    route = json.loads(args.route.read_text(encoding="utf-8"))
    gray = read_pgm(args.map)
    resolution = 0.0500000007
    origin = (-9.6502943, -13.1527195)
    blocked = inflate(gray, max(1, int(round(args.inflation_m / resolution))))
    poses = [(0.0, 0.0, "start")]
    for stop in route["stops"]:
        x, y, _ = stop["pose_map"]
        poses.append((x, y, stop["id"]))
    if route.get("return_to_start", True):
        poses.append(poses[0])
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rng_master = random.Random(args.seed)
    records, image_paths = [], []
    for run in range(1, args.runs + 1):
        rng = random.Random(rng_master.randrange(2**31))
        full = []
        leg_lengths = []
        try:
            for i in range(len(poses) - 1):
                a, b = poses[i], poses[i + 1]
                # Keep goal poses fixed; vary only planner tie-breaking.
                path = astar(blocked, world_to_rc(a[0], a[1], gray.shape, origin, resolution),
                             world_to_rc(b[0], b[1], gray.shape, origin, resolution), rng)
                xy = [rc_to_world(r, c, gray.shape, origin, resolution) for r, c in path]
                leg_lengths.append(sum(math.dist(p, q) for p, q in zip(xy, xy[1:])))
                full.extend(xy if not full else xy[1:])
            total = sum(leg_lengths)
            fig, ax = plt.subplots(figsize=(8, 7), dpi=130)
            ax.imshow(gray, cmap="gray", origin="upper", extent=[origin[0], origin[0] + gray.shape[1] * resolution,
                                                                    origin[1], origin[1] + gray.shape[0] * resolution])
            xs, ys = zip(*full)
            ax.plot(xs, ys, color="#e4572e", linewidth=1.7, alpha=.95, label=f"planned path ({total:.1f} m)")
            ax.scatter([p[0] for p in poses[:-1]], [p[1] for p in poses[:-1]], c="#1769aa", s=28, zorder=3)
            for j, p in enumerate(poses[:-1]):
                ax.annotate(f"{j}: {p[2]}", (p[0], p[1]), xytext=(4, 4), textcoords="offset points", fontsize=7)
            ax.set_title(f"Autonomous inspection path simulation {run:02d}  |  {len(poses)-1} legs")
            ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=.15)
            out = args.out_dir / f"run_{run:02d}.png"
            fig.savefig(out, bbox_inches="tight"); plt.close(fig)
            image_paths.append(out)
            records.append({"run": run, "status": "success", "path_length_m": round(total, 3), "legs": [round(x, 3) for x in leg_lengths]})
        except Exception as exc:
            records.append({"run": run, "status": "failed", "error": str(exc)})
    # Contact sheet with a consistent canvas size.
    thumbs = [Image.open(p).convert("RGB") for p in image_paths]
    if thumbs:
        w, h = max(im.width for im in thumbs), max(im.height for im in thumbs)
        cols = min(3, len(thumbs)); rows = math.ceil(len(thumbs) / cols)
        sheet = Image.new("RGB", (cols * w, rows * h), "white")
        for i, im in enumerate(thumbs): sheet.paste(im, ((i % cols) * w, (i // cols) * h))
        sheet.save(args.out_dir / "inspection_paths_montage.png")
    (args.out_dir / "simulation_metrics.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    (args.out_dir / "README.md").write_text("# 多次自主巡检寻路模拟\n\n运行 `python3 tools/inspection_path_sim.py --runs %d` 生成各次路径图和 `inspection_paths_montage.png` 汇总图。路径使用保存的 ARIAC 占据栅格地图和巡检路线，属于离线规划验证，不替代 ROS 在线导航遥测。\n" % args.runs, encoding="utf-8")
    print(json.dumps({"runs": args.runs, "successful": sum(r["status"] == "success" for r in records), "out_dir": str(args.out_dir)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
