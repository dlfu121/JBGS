"""Plot screwdriver-relative arm-base ranges for the 4f8ba74 and 255c14c scenes.

The current configs sample screwdriver position in Cartesian X/Y workspaces.
The yaw range is plotted separately as an angular sector so the two notions are
not confused.
"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Arc, Polygon, Rectangle
import numpy as np


OUT = Path("reports/screwdriver_relative_ranges.png")


def draw_position(ax, title, xlim, ylim, fixed, color):
    ax.set_title(title, fontsize=14, pad=10)
    ax.add_patch(
        Rectangle(
            (xlim[0], ylim[0]),
            xlim[1] - xlim[0],
            ylim[1] - ylim[0],
            facecolor=color,
            edgecolor=color,
            alpha=0.28,
            linewidth=2.5,
            label="机械臂根部位置范围",
        )
    )
    ax.scatter([0], [0], marker="+", s=100, color="black", linewidths=2, label="螺丝刀手柄中心")
    ax.scatter(
        [fixed[0]],
        [fixed[1]],
        marker="*",
        s=150,
        color="#c0392b",
        edgecolor="black",
        linewidth=0.7,
            label="固定手柄中心",
        zorder=5,
    )
    ax.annotate(
        "固定手柄中心",
        xy=fixed,
        xytext=(fixed[0] + 18, fixed[1] + 18),
        fontsize=9,
        arrowprops={"arrowstyle": "->", "color": "#555555", "lw": 0.8},
    )
    ax.axhline(0, color="#888888", linewidth=0.8)
    ax.axvline(0, color="#888888", linewidth=0.8)
    ax.set_xlabel("ΔX：机械臂根部相对螺丝刀手柄中心 (mm)")
    ax.set_ylabel("ΔY：机械臂根部相对螺丝刀手柄中心 (mm)")
    ax.set_xlim(-760, 120)
    ax.set_ylim(-380, 380)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.text(
        0.02,
        0.97,
        f"X: [{xlim[0]}, {xlim[1]}] mm\nY: [{ylim[0]}, {ylim[1]}] mm",
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.85},
    )


def draw_yaw_sector(ax, title, color):
    ax.set_title(title, fontsize=14, pad=10)
    radius = 280
    angles = np.linspace(np.deg2rad(55), np.deg2rad(125), 100)
    points = np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))
    sector = np.vstack(([0, 0], points, [0, 0]))
    ax.add_patch(
        Polygon(
            sector,
            closed=True,
            facecolor=color,
            edgecolor=color,
            alpha=0.28,
            linewidth=2.5,
            label="yaw 角度扇区",
        )
    )
    ax.plot([0, points[0, 0]], [0, points[0, 1]], color=color, linewidth=2)
    ax.plot([0, points[-1, 0]], [0, points[-1, 1]], color=color, linewidth=2)
    ax.add_patch(Arc((0, 0), 2 * radius, 2 * radius, angle=0, theta1=55, theta2=125, color=color, linewidth=2))
    fixed_angle = np.deg2rad(70)
    ax.plot(
        [0, radius * np.cos(fixed_angle)],
        [0, radius * np.sin(fixed_angle)],
        color="#c0392b",
        linewidth=2,
        linestyle="--",
        label="固定 yaw ≈ +70°",
    )
    ax.scatter([0], [0], marker="+", s=100, color="black", linewidths=2, label="螺丝刀中心")
    ax.text(115, 245, "yaw = 55°", fontsize=9, color=color)
    ax.text(-155, 245, "yaw = 125°", fontsize=9, color=color)
    ax.text(15, 125, "固定 yaw ≈ 70°", fontsize=9, color="#c0392b", rotation=70)
    ax.set_xlabel("世界 X 方向 (示意，mm)")
    ax.set_ylabel("世界 Y 方向 (示意，mm)")
    ax.set_xlim(-330, 330)
    ax.set_ylim(-80, 330)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.text(
        0.03,
        0.96,
        "这是姿态角范围，\n不是位置采样范围",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color="#7f1d1d",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#fff7ed", "edgecolor": "#f59e0b"},
    )


def main() -> None:
    # Matplotlib registers the TTC under the JP family name even though it
    # contains the required CJK glyphs on this system.
    cjk_font = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    if Path(cjk_font).exists():
        font_manager.fontManager.addfont(cjk_font)
        plt.rcParams["font.family"] = "Noto Sans CJK JP"
    else:
        plt.rcParams["font.family"] = "DejaVu Sans"
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    fig.subplots_adjust(left=0.08, right=0.98, top=0.88, bottom=0.10, wspace=0.25, hspace=0.28)
    fig.suptitle(
        "机械臂根部相对于螺丝刀的定位范围\n"
        "位置范围与姿态角范围分开表示",
        fontsize=18,
        fontweight="bold",
    )

    draw_position(
        axes[0, 0],
        "4f8ba74 半成品：实际位置范围（XY）",
        (-620, -450),
        (-100, 300),
        (-606.32, -82.41),
        "#2563eb",
    )
    draw_yaw_sector(axes[0, 1], "4f8ba74 半成品：姿态角范围", "#2563eb")
    draw_position(
        axes[1, 0],
        "255c14c 稳定版：实际位置范围（XY）",
        (-670, -520),
        (-150, 150),
        (-606.32, -82.41),
        "#16a34a",
    )
    draw_yaw_sector(axes[1, 1], "255c14c 稳定版：姿态角范围", "#16a34a")

    fig.text(
        0.5,
        0.025,
        "结论：当前两个版本的位置采样由 X/Y 矩形工作区决定；只有 yaw=[55°,125°] 是角度扇区。"
        "若要让机械臂根部落在真正的扇形位置区域，需要另行启用/定义圆弧位置采样。",
        ha="center",
        fontsize=11,
        color="#7f1d1d",
    )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=180, bbox_inches="tight")
    print(OUT)


if __name__ == "__main__":
    main()
