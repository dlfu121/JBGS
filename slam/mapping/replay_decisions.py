#!/usr/bin/env python3
"""离线决策回放：用已保存的成品地图当 ground truth，模拟逐步建图 + 探索决策。

不启动 MuJoCo / rtabmap / ROS，几分钟内在真实地图上回放
frontier_explorer.py 的决策流，并打印每次决策的原因，用于快速抓逻辑 bug。

原理：
  - 加载 yaml+pgm 成品地图，0=障碍(100)，其余=可走(0)，作为 ground truth
  - 已见图 seen 初始全 False（对应 unknown），每 0.5s 或检测前从当前位置
    向 360° 发 FAN_RAYS 条射线，遇 occupied 即停（其后为遮挡阴影保持
    unknown），binary_fill_holes 补相邻射线间的针孔 —— 等价理想 SLAM 逐
    步建图；射线太稀会在地图上留针孔 unknown，把已知区打碎成假 frontier
  - 每个决策周期：检测 frontier -> Dijkstra 可达性 -> 打分选目标 ->
    规划路径 -> 沿路径移动 -> 到达后停车等"地图更新"(~3s) -> 再找下一个
  - 连续 5 次无可达 frontier 判完成（与线上逻辑一致）
  - 复用线上同一个 frontier_explorer 的检测函数和 explore_planner，
    保证决策逻辑与真机完全一致
  - 性能节流与线上一致：EDT 重建 + frontier 重检只在必要时刻做
    （重检 3s 一次 / 无目标立即一次），不逐 0.1s 步跑

用法:
  python3.8 slam/mapping/replay_decisions.py \
      --map maps/ariac/ariac_map_3d.yaml \
      --start 4.0,4.6 \
      [--max-steps 600] [--out /tmp/opencode/replay_coverage.png]
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bridge"))

from explore_planner import ExplorePlanner  # noqa: E402
from frontier_explorer import (  # noqa: E402
    FRONTIER_MIN_SIZE,
    FRONTIER_GOAL_BIAS,
    FRONTIER_REACHED_DIST,
    REACHED_NEAR_DIST,
    FRONTIER_NO_GOAL_RETRY,
    PLAN_INFLATE,
    PLAN_WAYPOINT_SPACING,
    PLAN_MAX_GOAL_DIST,
    PLAN_NEAR_GOAL_SNAP,
    GOAL_BLACKLIST_RADIUS,
)

RANGE_MAX = 8.0          # 与桥接雷达一致
# 全向扇形射线数。要 8m 处相邻射线弧间距<一格(0.05m)，需 >2π·8/0.05≈1005 条；
# 太少会在地图上留"针孔 unknown"，把已知区打碎成假 frontier（旧 360 条 7.5m 盘内
# 残留 48.9% unknown，导致机器人卡在原地）。1200 条时残留≈真实遮挡。
FAN_RAYS = 1200
REVEAL_STEP = 5          # 每隔几步(0.5s)全向揭示一次；机器人每步仅移动~3cm
NAV_SPEED_MAX = 0.30     # 与探索器一致
LOOKAHEAD = 0.45
REPLAN_SEC = 1.0
RECHECK_SEC = 3.0        # frontier 重检测间隔
ROTATE_SPEED = 0.5       # 原地扫描角速度
DT = 0.1                 # 仿真步长（秒）
RECHECK_STEP = max(1, int(RECHECK_SEC / DT))   # frontier 重检周期（步）
# 到达后停车等待（线上 ROTATE_SCAN 是等 rtabmap 出图再补扫，只需几秒）。
# 注意：本回放是全向 360° 雷达、地图即时揭示，转一整圈没有任何信息增益，
# 早期版本用 2π/ROTATE_SPEED≈12.5s 纯浪费步数，300 步只能走到 1 个目标。
SCAN_SEC = 3.0
SCAN_STEPS = int(SCAN_SEC / DT)                # 到达后停车等待时长（步）
WANDER_WZ = 0.3 * DT * 3                        # 无目标时的慢转观察


class FakeInfo:
    def __init__(self, w, h, res, ox, oy):
        class Pos:
            def __init__(self, x, y, z=0.0):
                self.x = x
                self.y = y
                self.z = z
        self.width = w
        self.height = h
        self.resolution = res
        self.origin = type("O", (), {"position": Pos(ox, oy)})()


class FakeGrid:
    """模拟 nav_msgs/OccupancyGrid，喂给线上的 _detect_frontiers。"""
    def __init__(self, occ, resolution, origin):
        self.info = FakeInfo(occ.shape[1], occ.shape[0], resolution, origin[0], origin[1])
        self.data = occ.astype(np.int8).flatten().tolist()


class ReplayBot:
    """只复用 frontier_explorer 的检测/选目标逻辑，不走 ROS。"""

    def __init__(self, resolution, origin):
        self.grid = None
        self.planner = ExplorePlanner(
            inflate=PLAN_INFLATE,
            waypoint_spacing=PLAN_WAYPOINT_SPACING,
            max_goal_dist=PLAN_MAX_GOAL_DIST,
            near_goal_snap=PLAN_NEAR_GOAL_SNAP,
            blacklist_radius=GOAL_BLACKLIST_RADIUS,
        )
        self._failed_goals = []
        self.resolution = resolution
        self.origin = origin
        self.last_grid = None

    # ---- 与线上完全一致的检测/选择（直接复用方法体）----
    def _detect_frontiers(self):
        from frontier_explorer import FrontierExplorer
        return FrontierExplorer._detect_frontiers(self)

    def _select_frontier(self, frontiers):
        from frontier_explorer import FrontierExplorer
        return FrontierExplorer._select_frontier(self, frontiers)

    def _rebuild_cost(self):
        g = self.grid
        occ = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        self.planner.update_map(occ, g.info.resolution,
                                (g.info.origin.position.x, g.info.origin.position.y))

    def _dijkstra(self, pose):
        self.planner.compute_dijkstra((pose[0], pose[1]))


def load_map(yaml_path):
    """加载 yaml+pgm，返回 (ground_truth_occ(h,w) int8: 0 free / 100 occ, res, origin)。"""
    d = {}
    with open(yaml_path) as f:
        for line in f:
            k, _, v = line.partition(":")
            d[k.strip()] = v.strip()
    img_path = d["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    res = float(d["resolution"])
    ox, oy = float(d["origin"].split("[")[1].split("]")[0].split(",")[0]), \
             float(d["origin"].split("[")[1].split("]")[0].split(",")[1])
    negate = int(d.get("negate", 0))

    raw = open(img_path, "rb").read()
    parts = raw.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    px = np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w)

    # map_server 语义（negate=0）：255 白=free，0 黑=occupied；205=unknown 视作 free
    occ = np.where(px == 0, 100, 0).astype(np.int8)
    return occ, res, (ox, oy)


def reveal(truth, seen, x, y, res, ox, oy, rmax):
    """全向扇形射线，逐步"揭示"已见栅格（模拟理想 SLAM，无漂移）。

    从 (x,y) 向 360° 发射 FAN_RAYS 条射线，逐个标记射线经过的栅格；遇
    occupied 即停（其后为遮挡阴影，保持 unknown，与真实雷达一致）。逐次
    结果并进 seen(h,w) bool。最后 binary_fill_holes 清掉相邻射线间的
    亚格针孔——SLAM 的逆传感器模型会填充扇形，而不是只画射线上的点。
    """
    h, w = truth.shape
    rmax_cells = int(math.ceil(rmax / res))
    vis = np.zeros_like(seen)
    for k in range(FAN_RAYS):
        a = k * (2 * math.pi / FAN_RAYS)
        ca = math.cos(a)
        sa = math.sin(a)
        ax, ay = x, y
        for _ in range(rmax_cells):
            ax += res * ca
            ay += res * sa
            gi = int(math.floor((ax - ox) / res))
            gj = int(math.floor((ay - oy) / res))
            if not (0 <= gi < w and 0 <= gj < h):
                break
            vis[gj, gi] = True
            if truth[gj, gi] == 100:
                break
    vis = ndimage.binary_fill_holes(vis)
    seen |= vis


def seen_to_known(truth, seen):
    """由 seen 布尔图合成 nav_msgs 语义占用格：-1 unknown / 0 free / 100 occ。"""
    known = np.full(truth.shape, -1, dtype=np.int8)
    known[seen] = 0
    known[seen & (truth == 100)] = 100
    return known


def yaw_to_target(dx, dy):
    return math.atan2(dy, dx)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="maps/ariac/ariac_map_3d.yaml")
    ap.add_argument("--start", default="4.0,4.6", help="世界系起点 x,y")
    ap.add_argument("--max-steps", type=int, default=800, help="最大仿真步数(每步0.1s)")
    ap.add_argument("--out", default="/tmp/opencode/replay_coverage.png")
    args = ap.parse_args()

    truth, res, origin = load_map(args.map)
    h, w = truth.shape
    seen = np.zeros((h, w), dtype=bool)   # 已"看见"的栅格（理想 SLAM 无漂移）
    known = None
    bot = ReplayBot(res, origin)

    sx, sy = map(float, args.start.split(","))
    pose = [sx, sy, 0.0]
    no_frontier = 0
    goal = None
    path = []
    path_idx = 0
    rotate_until = None
    total_free_truth = int(np.sum(truth == 0))
    last_detect_step = -10 ** 9   # 上一次做 EDT+检测的步号，用于节流

    print(f"ground truth: {w}x{h} res={res:.3f} origin=({origin[0]:.2f},{origin[1]:.2f})")
    print(f"start: world({sx},{sy}) 自由区={total_free_truth} 格 ({100*total_free_truth/(h*w):.1f}%)\n")

    for step in range(args.max_steps):
        x, y, yaw = pose
        t = step * DT

        # ---- 决策节流（与线上一致：EDT/重检只在必要时刻做，不逐 0.1s 步跑）----
        scanning = rotate_until is not None and step < rotate_until
        if not scanning:
            if goal is None:
                # 无目标：走到重检间隔才检测一次（线上 FIND_FRONTIER 也是 3s 一次）
                do_detect = (step - last_detect_step) >= RECHECK_STEP
            else:
                # 有目标行驶中：只在重检周期打印当前候选（不打断当前目标）
                do_detect = (step % RECHECK_STEP == 0)
        else:
            do_detect = False  # 原地扫描期间不抢新目标、不重建 EDT

        # 建图：每 0.5s（或检测前）全向揭示一次（廉价扇区画格，等价理想 SLAM）。
        # 机器人每步只动 ~3cm，无需逐 0.1s 步重扫。
        if step % REVEAL_STEP == 0 or do_detect or step == args.max_steps - 1:
            reveal(truth, seen, x, y, res, origin[0], origin[1], RANGE_MAX)
            known = seen_to_known(truth, seen)
            bot.grid = FakeGrid(known, res, origin)

        if do_detect:
            last_detect_step = step
            known_free = int(np.sum(known == 0))
            unknown = int(np.sum(known == -1))
            cov = 100.0 * known_free / max(total_free_truth, 1)
            bot._rebuild_cost()          # EDT 只在此刻重建一次
            bot._dijkstra(pose)
            frontiers = bot._detect_frontiers()
            if not frontiers:
                print(f"[{t:6.1f}s] pose=({x:.2f},{y:.2f}) 无 frontier, cov={cov:.1f}%")
                no_frontier += 1
                if no_frontier >= FRONTIER_NO_GOAL_RETRY:
                    print(f"\n=== 完成: 连续 {no_frontier} 次无可达 frontier "
                          f"(与线上 FRONTIER_NO_GOAL_RETRY 一致) ===")
                    break
                goal = None
                path = []
                pose[2] = yaw + WANDER_WZ     # 慢转观察，等下次重检
                continue

            # ---- 打印全部候选与可达性（这就是"能看到决策"的核心）----
            best = bot._select_frontier(frontiers)
            info = []
            for fx, fy, size in frontiers:
                d = bot.planner.reachable_dist((fx, fy))
                reach = "✓" if (math.isfinite(d) and d <= PLAN_MAX_GOAL_DIST) else "✗"
                mark = " ←选中" if best and abs(fx - best[0]) < 1e-6 and abs(fy - best[1]) < 1e-6 else ""
                info.append(f"({fx:.1f},{fy:.1f}) s={size}{reach}{mark}")
            print(f"[{t:6.1f}s] pose=({x:.2f},{y:.2f},{yaw:.2f}) "
                  f"frontiers={len(frontiers)} cov={cov:.1f}% "
                  f"unknown={100.0*unknown/(h*w):.1f}%")
            print(f"          候选: {', '.join(info[:6])}")

            # 无目标时才选新目标（行驶中仅打印，不打断当前目标）
            if goal is None:
                no_frontier = 0
                if best is None:
                    print(f"  [选目标] 无可达 frontier (frontiers={len(frontiers)})，随机游走")
                    pose[2] = yaw + WANDER_WZ
                    continue
                goal = (best[0], best[1])
                path = bot.planner.plan_goal(goal)
                path_idx = 0
                if path:
                    print(f"  [选目标] {goal} 可达 {best[2]:.1f}m，路径 {len(path)} 点")
                else:
                    print(f"  [选目标] {goal} 规划失败，拉黑")
                    bot._failed_goals.append(goal)
                    goal = None
                    pose[2] = yaw + WANDER_WZ
                    continue

        # ---- 运动 ----
        if scanning:
            pose[2] = yaw + ROTATE_SPEED * DT
            if step == rotate_until - 1:
                print(f"  [扫描完成] 找下一个 frontier")
                rotate_until = None
                goal = None
                path = []
        elif goal is not None:
            dist = math.hypot(goal[0] - x, goal[1] - y)
            if dist < FRONTIER_REACHED_DIST:
                print(f"  [到达] {goal} 距离 {dist:.2f}m，停车等地图更新 "
                      f"(~{SCAN_STEPS * DT:.1f}s)")
                rotate_until = step + SCAN_STEPS
                goal = None
                path = []
            elif path:
                # 纯追踪沿路径移动
                while (path_idx < len(path) - 1
                       and math.hypot(path[path_idx][0] - x, path[path_idx][1] - y) < 0.35):
                    path_idx += 1
                look = path[path_idx]
                err = angle_diff(math.atan2(look[1] - y, look[0] - x), yaw)
                wz = np.clip(2.5 * err, -1.0, 1.0)
                vx = NAV_SPEED_MAX * max(0.25, 1.0 - abs(err) / math.pi)
                if abs(err) > 0.5:
                    vx = 0.0
                pose[0] += vx * math.cos(yaw) * DT
                pose[1] += vx * math.sin(yaw) * DT
                pose[2] += wz * DT
        else:
            pose[2] = yaw + WANDER_WZ   # 无目标且未到重检时刻：慢转观察

    # 最终报告
    known = seen_to_known(truth, seen)
    known_free = int(np.sum(known == 0))
    cov = 100.0 * known_free / max(total_free_truth, 1)
    print(f"\n=== 最终覆盖 = {cov:.1f}% ({known_free}/{total_free_truth} 自由格已揭示) ===")
    unknown_remain = int(np.sum(known == -1))
    print(f"剩余 unknown: {unknown_remain} 格 ({100.0*unknown_remain/(h*w):.1f}%)")
    print(f"拉黑目标: {bot._failed_goals}")

    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[truth == 0] = (240, 240, 240)
        rgb[truth == 100] = (80, 80, 80)
        rgb[known == -1] = (70, 120, 180)
        rgb[known == 100] = (20, 20, 20)
        plt.imsave(args.out, rgb)
        print(f"覆盖图已保存: {args.out}")


if __name__ == "__main__":
    main()
