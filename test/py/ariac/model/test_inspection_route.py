#!/usr/bin/env python3.8
"""Validate inspection arm poses against the compiled ARIAC model."""

import json
import math
import os
import unittest
from pathlib import Path

os.environ["MUJOCO_GL"] = "glfw"

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[4]
ROUTE_PATH = ROOT / "_ultimate_task" / "inspect" / "inspection_route.json"
MODEL_PATH = ROOT / "model" / "robot" / "ariac_lab_with_robot_3d.xml"


class InspectionRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with ROUTE_PATH.open(encoding="utf-8") as stream:
            cls.route = json.load(stream)
        cls.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        cls.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_SENSOR)

    def test_every_stop_centers_gauge_with_valid_three_second_dwell(self):
        model = self.model
        camera = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self.route["camera"])
        self.assertGreaterEqual(camera, 0)
        base_joints = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ("base_x", "base_y", "base_yaw")
        ]
        base_qpos = [model.jnt_qposadr[index] for index in base_joints]
        arm_joints = [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT,
                "arm_l/joint_%d" % index)
            for index in range(1, 7)
        ]
        arm_qpos = [model.jnt_qposadr[index] for index in arm_joints]
        travel_pose = np.asarray(self.route["travel_arm_pose"], dtype=float)
        travel_limits = model.jnt_range[arm_joints]
        self.assertTrue(np.all(travel_pose >= travel_limits[:, 0]))
        self.assertTrue(np.all(travel_pose <= travel_limits[:, 1]))

        for stop in self.route["stops"]:
            with self.subTest(stop=stop["id"]):
                self.assertEqual(float(stop["dwell_sec"]), 3.0)
                pose = np.asarray(stop["left_arm_pose"], dtype=float)
                limits = model.jnt_range[arm_joints]
                self.assertTrue(np.all(pose >= limits[:, 0]))
                self.assertTrue(np.all(pose <= limits[:, 1]))

                data = mujoco.MjData(model)
                x, y, yaw_degrees = stop["pose_map"]
                data.qpos[base_qpos] = (x, y, math.radians(yaw_degrees))
                data.qpos[arm_qpos] = pose
                mujoco.mj_forward(model, data)

                rotation = data.cam_xmat[camera].reshape(3, 3)
                camera_view = -rotation[:, 2]
                target_ray = (np.asarray(stop["gauge_world"], dtype=float)
                              - data.cam_xpos[camera])
                target_ray /= np.linalg.norm(target_ray)
                angle_degrees = math.degrees(math.acos(np.clip(
                    float(camera_view @ target_ray), -1.0, 1.0)))
                self.assertLess(angle_degrees, 3.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
