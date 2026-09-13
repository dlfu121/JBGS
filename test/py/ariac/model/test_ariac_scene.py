#!/usr/bin/env python3.8
"""Regression checks for the organized ARIAC scene and SLAM integration."""

import re
import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[4]
SCENE_XML = ROOT / "model" / "scenes" / "ariac_lab.xml"
ROBOT_XML = ROOT / "model" / "robot" / "ariac_lab_with_robot_3d.xml"
SOURCE_ROBOT_XML = ROOT / "model" / "robot" / "robot_template_3d_py38.xml"
RUN_SLAM = ROOT / "slam" / "run_slam_3d.sh"
RUN_NAV = ROOT / "slam" / "run_nav_saved.sh"
SHARED_BRIDGE = ROOT / "slam" / "bridge" / "bridge_warehouse.py"


class AriacSceneTest(unittest.TestCase):
    @staticmethod
    def _literal_assignment(source, name):
        tree = ast.parse(source)
        for node in tree.body:
            if (isinstance(node, ast.Assign) and
                    any(isinstance(target, ast.Name) and target.id == name
                        for target in node.targets)):
                return ast.literal_eval(node.value)
        raise AssertionError("missing literal assignment: %s" % name)

    def test_scene_assets_are_organized_and_resolvable(self):
        self.assertFalse((ROOT / "ariac_mujoco").exists())
        self.assertTrue((ROOT / "model" / "assets" / "ariac" /
                         "conversion_report.json").is_file())
        for xml_path in (SCENE_XML, ROBOT_XML):
            tree = ET.parse(str(xml_path))
            for mesh in tree.findall("./asset/mesh"):
                if "file" not in mesh.attrib:
                    # Gauge warning sectors are defined inline with vertices.
                    continue
                mesh_path = (xml_path.parent / mesh.attrib["file"]).resolve()
                self.assertTrue(mesh_path.is_file(), str(mesh_path))

    def test_mujoco_323_loads_static_and_robot_scenes(self):
        self.assertEqual(mujoco.__version__, "3.2.3")
        static_model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
        robot_model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        self.assertGreaterEqual(static_model.nmesh, 95)
        self.assertGreater(robot_model.nbody, static_model.nbody)
        self.assertGreater(robot_model.nu, 0)
        self.assertGreater(robot_model.nsensor, 0)
        self.assertGreaterEqual(mujoco.mj_name2id(
            robot_model, mujoco.mjtObj.mjOBJ_BODY, "dog_base"), 0)
        self.assertGreaterEqual(mujoco.mj_name2id(
            robot_model, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame"), 0)

    def test_gauge_scene_objects_reach_static_and_robot_models(self):
        expected_bodies = (
            "cabinet",
            "hydrant",
            "hydrant_2",
            "hydrant_3",
            "tank",
            "tank_front_gauge_pedestal",
        )
        expected_geoms = (
            "cabinet_meter_case",
            "hydrant_gauge_face",
            "hydrant_gauge_needle",
            "tank_front_support_pressure_gauge",
            "tank_pillar_gauge_face",
            "tank_pillar_gauge_needle",
        )
        for xml_path in (SCENE_XML, ROBOT_XML):
            model = mujoco.MjModel.from_xml_path(str(xml_path))
            for name in expected_bodies:
                self.assertGreaterEqual(mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, name), 0,
                    "%s missing body %s" % (xml_path, name))
            for name in expected_geoms:
                self.assertGreaterEqual(mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_GEOM, name), 0,
                    "%s missing geom %s" % (xml_path, name))

    def test_dynamic_pedestrian_mocaps_are_present_and_hidden_by_default(self):
        """The ARIAC bridge can toggle both inspection pedestrians."""
        model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        data = mujoco.MjData(model)
        names = ("dynamic_person", "dynamic_person_2")
        self.assertEqual(model.nmocap, len(names))
        for index, name in enumerate(names):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(body, 0, "missing pedestrian body %s" % name)
            mocap = int(model.body_mocapid[body])
            self.assertGreaterEqual(mocap, 0)
            np.testing.assert_allclose(
                data.mocap_pos[mocap], (-20.0 - 2.0 * index, -20.0, 0.0))

    def test_lidar_uses_original_robot_mount(self):
        model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        source = mujoco.MjModel.from_xml_path(str(SOURCE_ROBOT_XML))
        lidar = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        source_lidar = mujoco.mj_name2id(
            source, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        np.testing.assert_allclose(
            model.site_pos[lidar], source.site_pos[source_lidar], atol=1e-12)
        self.assertAlmostEqual(float(model.site_pos[lidar][2]), 0.95)
        self.assertEqual(mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "lidar3d_mast"), -1)

    def test_robot_start_is_not_inside_imported_wall_hulls(self):
        model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        dog_body = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        wall_prefix = "walls_base_link_collision_collision_"
        for contact in data.contact:
            geom_names = (
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
            )
            self.assertFalse(any(
                name and name.startswith(wall_prefix) for name in geom_names),
                "robot starts inside imported wall convex hull")

        start = data.xpos[dog_body]
        np.testing.assert_allclose(start[:2], (4.0, 4.6), atol=1e-12)

    def test_planar_joints_follow_world_map_axes(self):
        """Changing visual heading must not rotate the navigation slides."""
        model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        data = mujoco.MjData(model)
        dog = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        joints = [mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ("base_x", "base_y")]
        qpos = [model.jnt_qposadr[index] for index in joints]

        for values, expected in (((1.0, 0.0), (5.0, 4.6)),
                                 ((0.0, 1.0), (4.0, 5.6))):
            mujoco.mj_resetData(model, data)
            data.qpos[qpos] = values
            mujoco.mj_forward(model, data)
            np.testing.assert_allclose(
                data.xpos[dog][:2], expected, atol=1e-12)

    def test_ariac_patrol_covers_passable_east_corridor(self):
        source = SHARED_BRIDGE.read_text(encoding="utf-8")
        waypoints = self._literal_assignment(
            source, "ARIAC_PATROL_WAYPOINTS")
        northbound = [
            (19.9, -2.0),
            (19.9, 4.0),
            (19.9, 8.0),
            (19.9, 12.0),
            (19.9, 17.0),
        ]
        start = waypoints.index(northbound[0])
        self.assertEqual(waypoints[start:start + len(northbound)], northbound)
        self.assertEqual(
            waypoints[start + len(northbound):
                      start + 2 * len(northbound) - 1],
            list(reversed(northbound[:-1])))

        model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        angles = np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False)
        directions = np.column_stack((
            np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        minimum = float("inf")
        for y in np.linspace(-2.0, 17.0, 96):
            for z in (0.3, 0.7):
                geom_ids = np.full(len(angles), -1, dtype=np.int32)
                distances = np.full(len(angles), -1.0)
                mujoco.mj_multiRay(
                    model, data, np.asarray((19.9, y, z)),
                    directions.ravel(), None, 1, -1,
                    geom_ids, distances, len(angles), 8.0)
                valid = distances[distances > 1e-4]
                if valid.size:
                    minimum = min(minimum, float(valid.min()))
        self.assertGreaterEqual(minimum, 0.90)

        self.assertIn(
            'PATROL_V = (0.70 if SCENE_NAME == "ariac" else 0.50)',
            source)

    def test_slam_defaults_to_ariac_bridge(self):
        script = RUN_SLAM.read_text(encoding="utf-8")
        self.assertRegex(script, r'(?m)^SCENE="ariac"$')
        self.assertRegex(
            script,
            r'if \[ "\$SCENE" = "ariac" \]; then\s+' +
            r'BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"')
        self.assertIn("ariac|warehouse", script)

        nav_script = RUN_NAV.read_text(encoding="utf-8")
        self.assertRegex(nav_script, r'(?m)^SCENE="ariac"$')
        self.assertIn('BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"', nav_script)


if __name__ == "__main__":
    unittest.main(verbosity=2)
