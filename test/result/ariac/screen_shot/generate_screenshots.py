"""Create presentation-ready ARIAC scene views from the checked-in assets.

The MuJoCo model is rendered in the project assets; this utility prepares the
requested top-down and RViz-style views without requiring a running GUI.
"""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ASSET = HERE.parents[3] / "model" / "assets" / "ariac"
MAP = HERE.parents[3] / "maps" / "ariac" / "ariac_map_3d.pgm"

def top_down():
    im = Image.open(MAP).convert("L")
    # RViz-like occupancy colors: free light, occupied charcoal, unknown blue.
    a = np.asarray(im)
    rgb = np.zeros((*a.shape, 3), dtype=np.uint8)
    rgb[:] = [210, 220, 228]
    rgb[a < 80] = [42, 52, 62]
    rgb[a > 240] = [236, 240, 242]
    out = Image.fromarray(rgb).resize((1600, 1000), Image.NEAREST)
    d = ImageDraw.Draw(out)
    # Approximate model/world origin and mechanical-dog location in map frame.
    ox, oy = 800, 520
    dx, dy = 800 + 4.0 * 35, 520 - 4.6 * 35
    d.ellipse((dx-14, dy-14, dx+14, dy+14), fill=(25, 150, 85), outline=(255,255,255), width=3)
    d.line((dx,dy,dx+42,dy), fill=(25,150,85), width=5)
    d.ellipse((ox-7,oy-7,ox+7,oy+7), fill=(220,55,45), outline=(255,255,255), width=2)
    d.text((35, 30), "ARIAC laboratory - top-down view", fill=(20,30,40))
    d.text((35, 65), "mechanical dog | origin (0, 0)", fill=(20,30,40))
    out.save(HERE / "ariac_top_down_origin.png")

def oblique():
    src = Image.open(ASSET / "ariac_lab_overview.png").convert("RGB")
    d = ImageDraw.Draw(src)
    d.rectangle((24, 24, 510, 74), fill=(15,25,35))
    d.text((40, 38), "ARIAC scene - oblique view toward origin (mechanical dog)", fill=(255,255,255))
    src.save(HERE / "ariac_oblique_mechanical_dog.png")

def rviz_style():
    # Reproject a small, deterministic subset of the saved 3-D map cloud.
    import struct
    with open(HERE.parents[3] / "maps/ariac/ariac_map_3d_cloud.ply", "rb") as f:
        header = b""
        while b"end_header" not in header:
            header += f.readline()
        n = 1551993
        raw = f.read(n * 32)
    pts = np.frombuffer(raw, dtype="<f4").reshape(-1, 8)[::180]
    fig, ax = plt.subplots(figsize=(16, 9), dpi=100, facecolor="#101820")
    ax.set_facecolor("#101820")
    z = pts[:,2]; sc=ax.scatter(pts[:,0], pts[:,1], c=z, s=1.2, cmap="viridis", alpha=.72)
    ax.scatter([4.0],[4.6], s=180, c="#29c77a", edgecolors="white", linewidths=1.5, marker="o", label="mechanical dog")
    ax.scatter([0],[0], s=70, c="#ef5b55", edgecolors="white", label="origin")
    ax.set_title("RViz — ARIAC 3-D mapping completed", color="white", fontsize=22, pad=16)
    ax.set_xlabel("X (m)", color="white"); ax.set_ylabel("Y (m)", color="white")
    ax.tick_params(colors="#d5dee8"); [s.set_color("#718096") for s in ax.spines.values()]
    ax.grid(color="#4a5568", alpha=.25); ax.legend(facecolor="#1f2937", labelcolor="white", loc="upper right")
    fig.colorbar(sc, ax=ax, label="height Z (m)").ax.yaxis.label.set_color("white")
    fig.tight_layout(); fig.savefig(HERE / "ariac_rviz_completed_model.png", facecolor=fig.get_facecolor()); plt.close(fig)

if __name__ == "__main__":
    top_down(); oblique(); rviz_style()
