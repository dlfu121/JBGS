#!/usr/bin/env python3.8
"""Regression checks for robot left/right mounting and camera placement."""

import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ["MUJOCO_GL"] = "glfw"

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[4]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from model.robot.initial_pose import reset_to_ready


ROBOT_DIR = ROOT / "model" / "robot"
ALL_MODELS = (
    ROBOT_DIR / "robot.xml",
    ROBOT_DIR / "robot_py38.xml",
    ROBOT_DIR / "robot_template_3d_py38.xml",
    ROBOT_DIR / "ariac_lab_with_robot_3d.xml",
    ROBOT_DIR / "warehouse_with_robot_3d.xml",
)
COMPATIBLE_MODELS = ALL_MODELS


class RobotSidednessTest(unittest.TestCase):
    def test_all_xml_structures_use_matching_arm_and_hand_sides(self):
        for path in ALL_MODELS:
            with self.subTest(model=path.name):
                root = ET.parse(str(path)).getroot()
                parents = {
                    child: parent
                    for parent in root.iter()
                    for child in parent
                }
                bodies = {
                    body.get("name"): body for body in root.iter("body")
                }
                meshes = {
                    mesh.get("name"): mesh.get("file", "")
                    for mesh in root.iter("mesh")
                }

                self.assertEqual(
                    float(bodies["arm_l/base_link"].get("pos").split()[1]),
                    -0.175)
                self.assertEqual(
                    float(bodies["arm_r/base_link"].get("pos").split()[1]),
                    0.175)
                self.assertIn("hand_l/left_hand_base", bodies)
                self.assertIn("hand_r/right_hand_base", bodies)
                self.assertTrue(meshes["hand_l/left_hand_base"].endswith(
                    "left_hand_base.STL"))
                self.assertTrue(meshes["hand_r/right_hand_base"].endswith(
                    "right_hand_base.STL"))

                cameras = [
                    camera for camera in root.iter("camera")
                    if camera.get("name") == "lefthand_camera"
                ]
                self.assertEqual(len(cameras), 1)
                self.assertEqual(
                    parents[cameras[0]].get("name"),
                    "hand_l/left_hand_base")
                np.testing.assert_allclose(
                    np.fromstring(cameras[0].get("quat"), sep=" "),
                    (0.0, np.sqrt(0.5), -np.sqrt(0.5), 0.0),
                    atol=1e-6)

    def test_compiled_models_place_left_arm_on_robot_left(self):
        for path in COMPATIBLE_MODELS:
            with self.subTest(model=path.name):
                model = mujoco.MjModel.from_xml_path(str(path))
                data = mujoco.MjData(model)
                reset_to_ready(mujoco, model, data)

                def body_id(name):
                    result = mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_BODY, name)
                    self.assertGreaterEqual(result, 0, name)
                    return result

                dog = body_id("dog_base")
                arm_l = body_id("arm_l/base_link")
                arm_r = body_id("arm_r/base_link")
                hand_l = body_id("hand_l/left_hand_base")
                rotation = data.xmat[dog].reshape(3, 3)
                forward = -rotation[:, 0]
                left = np.cross(rotation[:, 2], forward)
                left_offset = float((data.xpos[arm_l] - data.xpos[dog]) @ left)
                right_offset = float((data.xpos[arm_r] - data.xpos[dog]) @ left)
                self.assertGreater(left_offset, 0.0)
                self.assertLess(right_offset, 0.0)

                camera = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_CAMERA, "lefthand_camera")
                self.assertEqual(int(model.cam_bodyid[camera]), hand_l)
                self.assertLess(
                    float(np.linalg.norm(
                        data.cam_xpos[camera] - data.xpos[hand_l])),
                    0.1)

                hand_rotation = data.xmat[hand_l].reshape(3, 3)
                camera_rotation = data.cam_xmat[camera].reshape(3, 3)
                camera_view = -camera_rotation[:, 2]
                camera_up = camera_rotation[:, 1]
                np.testing.assert_allclose(
                    camera_view, hand_rotation[:, 2], atol=1e-6)
                np.testing.assert_allclose(
                    camera_up, -hand_rotation[:, 0], atol=1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
