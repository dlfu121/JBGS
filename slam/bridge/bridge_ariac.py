#!/usr/bin/env python3.8
"""ARIAC-scene entry point for the shared 3-D MuJoCo/ROS bridge.

This is a separate entry point from ``bridge_warehouse.py``.  Both use the
same low-level bridge implementation, but this file binds the ARIAC scene
before importing it, so ARIAC never runs a warehouse entry-point script.
"""

import os


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ["SLAM_SCENE_NAME"] = "ariac"
# Honour a scene selected by the launcher (the VLA workflow supplies the
# freshly randomized XML) while retaining the ordinary ARIAC default.
os.environ.setdefault("MUJOCO_SCENE_XML", os.path.join(
    PROJECT_ROOT, "model", "robot", "ariac_lab_with_robot_3d.xml"))

try:
    from .bridge_core import SlamBridge3D, main
except ImportError:
    from bridge_core import SlamBridge3D, main


if __name__ == "__main__":
    main()
