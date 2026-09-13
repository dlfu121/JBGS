#!/usr/bin/env python3.8
"""Warehouse-scene entry point for the shared 3-D MuJoCo/ROS bridge.

The implementation lives in :mod:`bridge_core`; this file deliberately binds
the warehouse XML and scene before importing it.  ARIAC has its own entry
point in ``bridge_ariac.py`` and never executes this file.
"""

import os


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ["SLAM_SCENE_NAME"] = "warehouse"
os.environ["MUJOCO_SCENE_XML"] = os.path.join(
    PROJECT_ROOT, "model", "robot", "warehouse_with_robot_3d.xml")

try:
    from .bridge_core import SlamBridge3D, main
except ImportError:
    from bridge_core import SlamBridge3D, main


if __name__ == "__main__":
    main()
