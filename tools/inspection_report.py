#!/usr/bin/env python3
"""Generate inspection-capability metrics, CSV/Markdown tables and a PNG dashboard.

Examples
--------
python3 tools/inspection_report.py
python3 tools/inspection_report.py --records _ultimate_task/inspect/record \
    --telemetry run_telemetry.csv --out-dir inspection_report

Telemetry is optional.  If supplied, the CSV may contain columns named
``time_s``, ``x_m``, ``y_m``, ``collision``, ``dynamic_avoid`` and
``inspection_stop`` (extra columns are ignored).
"""
from __future__ import annotations
import argparse, csv, json, math, os
from collections import defaultdict
from pathlib import Path

EXPECTED = ["cabinet_voltage", "tank_pressure", "hydrant_pressure"]

def load_records(folder: Path):
    rows = []
    for p in sorted(folder.glob("*.json")):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not d.get("stop_id"):
            continue
        d["_file"] = p.name
        d["_image_exists"] = bool(d.get("image") and (folder / d["image"]).is_file())
        rows.append(d)
    return rows

def metrics(records, telemetry=None):
    by_stop = defaultdict(list)
    for r in records:
        by_stop[r["stop_id"]].append(r)
    stops = sorted(set(EXPECTED) | set(by_stop))
    out = []
    for stop in stops:
        rs = by_stop.get(stop, [])
        # Require the PNG, not only a filename in metadata.
        ok = [r for r in rs if r.get("_image_exists")]
        times = [float(r["simulation_time"]) for r in ok
                 if str(r.get("simulation_time", "")).replace(".", "", 1).isdigit()]
        out.append({"stop_id": stop, "captures": len(ok),
                    "success": int(bool(ok)),
                    "first_simulation_s": min(times) if times else "",
                    "last_simulation_s": max(times) if times else "",
                    "target": (ok or rs or [{"target": ""}])[0].get("target", "")})
    total = len(stops)
    successful = sum(x["success"] for x in out)
    summary = {"expected_stops": total, "covered_stops": successful,
               "coverage_rate": successful / total if total else 0.0,
               "capture_count": sum(x["captures"] for x in out),
               "additional_archive_captures": max(0, sum(x["captures"] for x in out)-successful)}
    if telemetry:
        summary.update(telemetry_metrics(telemetry))
    return out, summary

def telemetry_metrics(path: Path):
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    def flag(name):
        return sum(str(r.get(name, "")).strip().lower() in ("1", "true", "yes", "y", "on") for r in rows)
    result = {"telemetry_samples": len(rows), "collision_events": flag("collision"),
              "dynamic_avoid_samples": flag("dynamic_avoid")}
    ts = []
    for r in rows:
        try: ts.append(float(r.get("time_s", "")))
        except (TypeError, ValueError): pass
    if len(ts) > 1: result["telemetry_duration_s"] = max(ts)-min(ts)
    # Integrate planar distance when pose is available.
    points=[]
    for r in rows:
        try: points.append((float(r["x_m"]), float(r["y_m"])))
        except (KeyError, TypeError, ValueError): pass
    if len(points)>1:
        result["path_length_m"] = sum(math.dist(a,b) for a,b in zip(points, points[1:]))
    return result

def write_outputs(per_stop, summary, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "inspection_metrics.csv").open("w", newline="", encoding="utf-8") as f:
        w=csv.DictWriter(f, fieldnames=per_stop[0].keys()); w.writeheader(); w.writerows(per_stop)
    (out_dir / "inspection_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    lines=["# 自主巡检能力验证报告", "", "## 汇总指标", "", "| 指标 | 数值 |", "|---|---:|"]
    labels={"expected_stops":"计划巡检点", "covered_stops":"成功覆盖点位", "coverage_rate":"归档点位覆盖率", "capture_count":"有效拍摄数", "additional_archive_captures":"额外历史拍摄数", "telemetry_duration_s":"遥测时长 (s)", "path_length_m":"实际路径长度 (m)", "collision_events":"碰撞事件数", "dynamic_avoid_samples":"动态避障采样数"}
    for k,v in summary.items():
        if k in labels:
            val=f"{v*100:.1f}%" if k=="coverage_rate" else (f"{v:.2f}" if isinstance(v,float) else v)
            lines.append(f"| {labels[k]} | {val} |")
    lines += ["", "## 分点结果", "", "| 点位 | 目标 | 有效拍摄 | 是否覆盖 | 首次仿真时间 (s) |", "|---|---|---:|:---:|---:|"]
    for r in per_stop: lines.append(f"| {r['stop_id']} | {r['target']} | {r['captures']} | {'是' if r['success'] else '否'} | {r['first_simulation_s']} |")
    (out_dir / "inspection_summary.md").write_text("\n".join(lines)+"\n", encoding="utf-8")

def plot(per_stop, summary, out: Path):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-inspection-report")
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"]=["DejaVu Sans"]; plt.rcParams["axes.unicode_minus"]=False
    fig, ax = plt.subplots(2,2, figsize=(12,7), constrained_layout=True)
    names=[r["stop_id"] for r in per_stop]; counts=[r["captures"] for r in per_stop]
    ax[0,0].bar(names, counts, color=["#2ca02c" if c else "#d62728" for c in counts]); ax[0,0].set_title("Inspection captures by stop"); ax[0,0].set_ylabel("valid images"); ax[0,0].tick_params(axis="x", rotation=25)
    ax[0,1].bar(["covered", "missing"], [summary["covered_stops"], summary["expected_stops"]-summary["covered_stops"]], color=["#1f77b4","#dddddd"]); ax[0,1].set_ylim(0,max(1,summary["expected_stops"])); ax[0,1].set_title(f"Coverage rate: {summary['coverage_rate']*100:.1f}%")
    vals=[summary.get("capture_count",0), summary.get("additional_archive_captures",0), summary.get("collision_events",0), summary.get("dynamic_avoid_samples",0)]
    ax[1,0].bar(["captures","additional","collisions","dynamic avoid"], vals, color="#ff7f0e"); ax[1,0].set_title("Operational indicators"); ax[1,0].tick_params(axis="x", rotation=20)
    ax[1,1].axis("off"); text="\n".join([f"{k}: {v:.2f}" if isinstance(v,float) else f"{k}: {v}" for k,v in summary.items()]); ax[1,1].text(0,1,text,va="top",family="monospace",fontsize=10); ax[1,1].set_title("Summary")
    fig.suptitle("Autonomous inspection capability report", fontsize=15)
    fig.savefig(out, dpi=160); plt.close(fig)

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument("--records", type=Path, default=Path("_ultimate_task/inspect/record")); ap.add_argument("--telemetry", type=Path); ap.add_argument("--out-dir", type=Path, default=Path("inspection_report")); args=ap.parse_args()
    rec=load_records(args.records); per, summary=metrics(rec,args.telemetry); write_outputs(per,summary,args.out_dir); plot(per,summary,args.out_dir/"inspection_dashboard.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2)); print(f"输出目录: {args.out_dir.resolve()}")
if __name__ == "__main__": main()
