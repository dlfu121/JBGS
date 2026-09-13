#!/usr/bin/env python3
"""从原 3D 场景提取 dog_base（含雷达/手臂）注入 warehouse 场景。

生成: warehouse_with_robot_3d.xml （slam_bridge_3d.py 可直接加载）

用法:
    python3 gen_warehouse_robot.py              # 默认起点 (-9, -9)
    python3 gen_warehouse_robot.py -9 -9        # 指定起点
"""
import os
import re
import sys

os.environ["MUJOCO_GL"] = "glfw"

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, "robot_template_3d_py38.xml")
TARGET = os.path.join(HERE, "..", "scenes", "warehouse_with_obstacles_mujoco.xml")
OUTPUT = os.path.join(HERE, "warehouse_with_robot_3d.xml")

# 起始位置
START_X = float(sys.argv[1]) if len(sys.argv) > 1 else -9.0
START_Y = float(sys.argv[2]) if len(sys.argv) > 2 else -9.0


def main():
    with open(SOURCE, "r") as f:
        src_lines = f.read().split("\n")
    with open(TARGET, "r") as f:
        tgt = f.read()

    # --- 从源文件提取各部分 ---
    # 1. dog 相关 mesh/material 定义
    asset_start = asset_end = None
    for i, line in enumerate(src_lines):
        if "<asset>" in line:
            asset_start = i
        if "</asset>" in line:
            asset_end = i
            break

    asset_lines = []
    for i in range(asset_start, asset_end + 1):
        line = src_lines[i]
        if ('mesh name="dog_' in line or 'mesh name="arm_' in line
                or 'mesh name="hand_' in line):
            asset_lines.append(line)
        if 'material name="dog_mat"' in line:
            asset_lines.append(line)

    # 2. dog_base body (含 lidar sites)
    dog_start = worldbody_end = None
    for i, line in enumerate(src_lines):
        if '<body name="dog_base"' in line:
            dog_start = i
        if "</worldbody>" in line:
            worldbody_end = i
            break
    dog_body_lines = src_lines[dog_start:worldbody_end]
    # 替换起始位置
    dog_body_lines[0] = re.sub(
        r'pos="[^"]*"',
        f'pos="{START_X} {START_Y} 0"',
        dog_body_lines[0])

    # 3. actuator
    act_start = act_end = None
    for i, line in enumerate(src_lines):
        if "<actuator>" in line:
            act_start = i
        if "</actuator>" in line:
            act_end = i
            break
    act_lines = src_lines[act_start:act_end + 1]

    # 4. sensor
    sens_start = sens_end = None
    for i, line in enumerate(src_lines):
        if "<sensor>" in line:
            sens_start = i
        if "</sensor>" in line:
            sens_end = i
            break
    sens_lines = src_lines[sens_start:sens_end + 1]

    # 5. keyframe
    key_start = key_end = None
    for i, line in enumerate(src_lines):
        if "<keyframe" in line:
            key_start = i
        if "</keyframe>" in line:
            key_end = i
            break
    key_lines = src_lines[key_start:key_end + 1] if key_start else []

    # --- 注入到 warehouse XML ---
    # 插入 asset (在 </default> 后)
    asset_block = "  <asset>\n" + "\n".join(asset_lines) + "\n  </asset>\n"
    # 在 </default> 后插入 asset
    tgt = tgt.replace("</default>\n", "</default>\n\n" + asset_block + "\n", 1)

    # 插入 dog_base body (在 </worldbody> 前)
    dog_block = "\n".join(dog_body_lines)
    tgt = tgt.replace("  </worldbody>", dog_block + "\n  </worldbody>", 1)

    # 插入 actuator + sensor (在 </mujoco> 前)
    # 注意：不复制 keyframe，因为仓库场景的自由度与原场景不同
    tail = "\n".join(act_lines) + "\n\n" + "\n".join(sens_lines)
    tgt = tgt.replace("</mujoco>", "\n" + tail + "\n</mujoco>", 1)

    # 写出
    with open(OUTPUT, "w") as f:
        f.write(tgt)
    model = mujoco.MjModel.from_xml_path(OUTPUT)
    print(f"已生成: {OUTPUT}")
    print(f"机器人起点: ({START_X}, {START_Y})")
    print(f"验证通过: nbody={model.nbody}, njnt={model.njnt}, ncam={model.ncam}")


if __name__ == "__main__":
    main()
