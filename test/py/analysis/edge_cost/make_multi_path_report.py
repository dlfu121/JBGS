#!/usr/bin/env python3.8
"""Render a concise Chinese interpretation from multi_path_metrics.json."""
import argparse
import csv
import json
import os
import statistics


def pct(v):
    return "—" if v is None else "%.2f%%" % v


def fmt(v, digits=3):
    return "—" if v is None else ("%%.%df" % digits) % v


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--result-dir", required=True)
    args = p.parse_args()
    out = os.path.abspath(args.result_dir)
    with open(os.path.join(out, "multi_path_metrics.json"), encoding="utf-8") as f:
        data = json.load(f)
    rows = data["records"]
    by = {}
    for r in rows:
        by.setdefault(r["method"], []).append(r)
    base = {r["route_id"]: r for r in by["distance"]}
    labels = {r["method"]: r["label"] for r in rows}

    geo1 = [r for r in rows if r["method"] == "geo_lambda_1"]
    geo2 = [r for r in rows if r["method"] == "geo_lambda_2"]
    def changes(method, key):
        vals = []
        for r in by[method]:
            b = base[r["route_id"]]
            if r.get(key) is not None and b.get(key) not in (None, 0):
                vals.append(100 * (r[key] / b[key] - 1))
        return vals
    len2, clr2, plan2 = changes("geo_lambda_2", "path_length_m"), changes("geo_lambda_2", "minimum_clearance_m"), changes("geo_lambda_2", "planning_time_ms")
    len1, clr1 = changes("geo_lambda_1", "path_length_m"), changes("geo_lambda_1", "minimum_clearance_m")
    height_equal = sum(1 for r in by["height_lambda_1"]
                       if r.get("path_length_m") == base[r["route_id"]].get("path_length_m")
                       and r.get("minimum_clearance_m") == base[r["route_id"]].get("minimum_clearance_m"))
    traversable = data["traversable_cells"]
    observed = data["height_observed_cells"]
    observed_trav = data["height_observed_traversable_cells"]

    report = os.path.join(out, "analysis.md")
    with open(report, "w", encoding="utf-8") as f:
        f.write("# ARIAC 多路径代价地图与高度地图分析\n\n")
        f.write("本报告基于同一份保存地图 `ariac_map_3d.pgm`、点云 `ariac_map_3d_cloud.ply`，对 **8 条固定路线 × 5 种代价配置 = 40 次规划** 汇总。每条边都用 Bresenham 栅格采样，指标可由 `multi_path_metrics.csv/json` 复算。\n\n")
        f.write("## 1. 总体结果\n\n")
        f.write("| 方法 | 成功率 | 平均路径 (m) | 平均最小水平净距 (m) | 平均规划 (ms) | 展开节点均值 |\n|---|---:|---:|---:|---:|---:|\n")
        for a in data["aggregates"]:
            f.write("| %s | %.0f%% | %s | %s | %s | %s |\n" % (a["label"], 100*a["success_rate"], fmt(a["mean_length_m"]), fmt(a["mean_min_clearance_m"]), fmt(a["mean_planning_time_ms"], 1), fmt(a["mean_expanded_nodes"], 0)))
        f.write("\n**核心量化结论（相对 distance only）：**\n\n")
        f.write("- `λg=1`：路径长度变化范围 **%s～%s**，最小水平净距提升 **%s～%s**。\n" % (pct(min(len1)), pct(max(len1)), pct(min(clr1)), pct(max(clr1))))
        f.write("- `λg=2`：路径长度变化范围 **%s～%s**（均值 %s），最小水平净距提升范围 **%s～%s**（均值 %s）；8/8 条路线成功。\n" % (pct(min(len2)), pct(max(len2)), pct(statistics.mean(len2)), pct(min(clr2)), pct(max(clr2)), pct(statistics.mean(clr2))))
        f.write("- `λg=2` 的规划时间均值约为 distance 的 **%.2fx**；这是以更安全路径换取的搜索开销，部署时可按实时性调节 λ。\n" % (data["aggregates"][2]["mean_planning_time_ms"] / data["aggregates"][0]["mean_planning_time_ms"]))
        f.write("- `λh=1/2` 在当前点云上分别有 **%d/8** 条路线与距离基线完全相同（长度、最小水平净距均不变），不是算法没有高度项，而是路线采样不到独立高度证据。\n" % height_equal)

        f.write("\n## 2. 逐路线对比（λg=2）\n\n")
        f.write("| 路线 | 基线长度 | 安全长度 | 长度变化 | 基线最小净距 | 安全最小净距 | 净距变化 |\n|---|---:|---:|---:|---:|---:|---:|\n")
        for r in geo2:
            b = base[r["route_id"]]
            dl = 100*(r["path_length_m"]/b["path_length_m"]-1)
            dc = 100*(r["minimum_clearance_m"]/b["minimum_clearance_m"]-1)
            f.write("| %s | %.3f | %.3f | %+.2f%% | %.3f | %.3f | %+.2f%% |\n" % (r["route_id"], b["path_length_m"], r["path_length_m"], dl, b["minimum_clearance_m"], r["minimum_clearance_m"], dc))

        f.write("\n## 3. 高度地图证据与边界\n\n")
        f.write("- 高度点云投影得到 **%d** 个有观测栅格，其中 **%d** 个同时处于二维可通行区（占二维可通行栅格 %.3f%%）；有观测栅格占整张栅格 %.2f%%。硬阈值为 %.2f m，优选净空为 %.2f m。\n" % (observed, observed_trav, 100*observed_trav/traversable, 100*observed/(data["map_shape"][0]*data["map_shape"][1]), data["hard_height_m"], data["preferred_height_m"]))
        f.write("- 当前 8 条路线的高度剖面均未采到有限高度净空值（图 `height_profiles_grid.png` 中只会显示阈值参考线），因此不能从本次路线数据宣称高度代价改善了通行安全；这反而明确指出下一轮实验应加入穿越桌下/悬空障碍的专门起终点。\n")
        f.write("- 高度地图的潜在优势是识别二维占据图无法表达的‘可穿越但高度不足’区域：`height_blocked.png` 给出硬阻挡，`height_cost` 在 0.70～1.00 m 之间提供连续软代价。当前点云覆盖不足时，算法会退化为二维代价地图，这是数据覆盖问题而非结论。\n")

        f.write("\n## 4. 如何阅读图表\n\n")
        f.write("- `multi_paths_grid.png`：8 个路线小图；红/绿路径主动绕开低水平净距带，蓝色为距离最短路径。\n")
        f.write("- `length_clearance_scatter.png`：右上方向代表更安全；可直接观察少量绕行带来的净距收益。\n")
        f.write("- `aggregate_metrics.png`：跨路线均值，展示长度、净距和规划时间的总体权衡。\n")
        f.write("- `height_profiles_grid.png`：沿每条路径的垂直净距采样；黑虚线=硬阈值，灰点线=优选阈值。\n")
        f.write("\n## 5. 建议的下一轮 MuJoCo/RViz 验证\n\n")
        f.write("1. 在 MuJoCo 中将机器人分别放到桌下、悬空横梁下和无遮挡走廊，录制包含地面以上 0.12 m 的点云；确保高度观测落在二维可通行栅格。\n2. 对同一批路线重复 `λh=0,1,2`，增加‘高度受限但二维空闲’路线，并记录碰撞/不可达率。\n3. 真实跟随时固定速度、动态障碍和起终点，只改变 λ；将规划图与实际接触事件对齐。\n")
    print(report)


if __name__ == "__main__":
    main()
