#!/usr/bin/env python3
"""Interactive tuner for the three ARIAC inspection arm poses.

The tool loads the same MuJoCo model used by ``run_inspection.py`` and freezes
the robot at one route stop.  Six matplotlib sliders control
``arm_l/joint_1`` ... ``arm_l/joint_6`` (the sliders are displayed in degrees,
while ``inspection_route.json`` stores radians).  The left-wrist camera image
is rendered continuously, and the current pose can be written back to the
selected stop in the route file.

Example (run from ``worker_scene``)::

    python3 _ultimate_task/inspect/tune_inspection_arm.py
    python3 _ultimate_task/inspect/tune_inspection_arm.py --stop tank_pressure

The MuJoCo 3-D viewer is useful for checking collisions.  The matplotlib
window contains the camera image, stop selector, six joint sliders, Save and
Reset buttons.  Saving creates ``inspection_route.json.bak`` first and then
replaces the JSON atomically.


cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
python3 _ultimate_task/inspect/tune_inspection_arm.py
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

# tune_camera.py uses the desktop GLFW/Qt backends.  Set these before importing
# MuJoCo/matplotlib so an inherited headless EGL setting cannot win.
# Force the desktop backend, just like tune_camera.py.  A shell may export
# MUJOCO_GL=egl for headless jobs; that backend cannot create this interactive
# viewer and would fail during import.
os.environ["MUJOCO_GL"] = "glfw"
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-inspection-arm-tuner")

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons, Slider
import glfw
import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
WORKER_SCENE = HERE.parents[1]
DEFAULT_ROUTE = HERE / "inspection_route.json"
DEFAULT_SCENE = WORKER_SCENE / "model" / "robot" / "ariac_lab_with_robot_3d.xml"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FRAME_PERIOD_SEC = 0.01

# Allow the script to be launched by path from any working directory while
# reusing the project's shared ready-pose helper.
if str(WORKER_SCENE) not in sys.path:
    sys.path.insert(0, str(WORKER_SCENE))
from model.robot.initial_pose import reset_to_ready


def _joint_id(model, name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise RuntimeError("模型中找不到关节: %s" % name)
    return jid


class InspectionArmTuner:
    JOINT_NAMES = tuple("arm_l/joint_%d" % i for i in range(1, 7))

    def __init__(self, route_path, scene_path, initial_stop=None):
        self.route_path = Path(route_path).expanduser().resolve()
        self.scene_path = Path(scene_path).expanduser().resolve()
        with self.route_path.open(encoding="utf-8") as stream:
            self.route = json.load(stream)
        stops = self.route.get("stops")
        if self.route.get("scene") != "ariac" or not stops:
            raise ValueError("路线文件必须是包含 stops 的 ARIAC 配置")
        self.stops = list(stops)
        self.stop_ids = [str(stop["id"]) for stop in self.stops]
        self.stop_index = self._resolve_stop(initial_stop)

        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        reset_to_ready(mujoco, self.model, self.data, strict=False)
        self.camera_name = str(self.route.get("camera", "lefthand_camera"))
        self.camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if self.camera_id < 0:
            raise RuntimeError("模型中找不到相机: %s" % self.camera_name)
        self.joint_ids = np.asarray(
            [_joint_id(self.model, name) for name in self.JOINT_NAMES],
            dtype=np.int32)
        self.qpos_addresses = np.asarray(
            [self.model.jnt_qposadr[jid] for jid in self.joint_ids],
            dtype=np.int32)
        self.base_addresses = np.asarray([
            self.model.jnt_qposadr[_joint_id(self.model, name)]
            for name in ("base_x", "base_y", "base_yaw")
        ], dtype=np.int32)
        self.joint_limits = self.model.jnt_range[self.joint_ids].copy()
        self.pose = np.zeros(6, dtype=np.float64)
        self._set_stop(self.stop_index)

        self.viewer = None
        self.offscreen_window = None
        self.scene = None
        self.context = None
        self.cam_view = None
        self._init_rendering()

        self.fig = None
        self.image_artist = None
        self.info_text = None
        self.status_text = None
        self.sliders = []
        self.radio = None
        self.save_button = None
        self.reset_button = None
        self._updating_controls = False
        self._running = True
        self._last_frame = 0

    def _resolve_stop(self, value):
        if value is None:
            return 0
        text = str(value)
        if text in self.stop_ids:
            return self.stop_ids.index(text)
        try:
            index = int(text)
        except ValueError:
            raise ValueError("未知巡检点 %r，可选: %s" %
                             (value, ", ".join(self.stop_ids)))
        if not 0 <= index < len(self.stops):
            raise ValueError("巡检点索引超出范围: %s" % value)
        return index

    @property
    def stop(self):
        return self.stops[self.stop_index]

    def _set_stop(self, index):
        stop = self.stops[index]
        if len(stop.get("pose_map", ())) != 3:
            raise ValueError("%s 的 pose_map 必须有 3 个数" % stop["id"])
        if len(stop.get("left_arm_pose", ())) != 6:
            raise ValueError("%s 的 left_arm_pose 必须有 6 个数" % stop["id"])
        x, y, yaw_degrees = [float(v) for v in stop["pose_map"]]
        self.data.qpos[self.base_addresses] = (x, y, math.radians(yaw_degrees))
        self.pose = np.asarray(stop["left_arm_pose"], dtype=np.float64).copy()
        self._clip_pose()
        self.data.qpos[self.qpos_addresses] = self.pose
        mujoco.mj_forward(self.model, self.data)

    def _clip_pose(self):
        self.pose = np.clip(self.pose, self.joint_limits[:, 0],
                            self.joint_limits[:, 1])

    def _init_rendering(self):
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.cam.distance = 2.0
        self.viewer.cam.azimuth = 140
        self.viewer.cam.elevation = -25
        if not glfw.init():
            raise RuntimeError("GLFW 初始化失败")
        self._glfw = glfw
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        self.offscreen_window = glfw.create_window(
            CAMERA_WIDTH, CAMERA_HEIGHT, "Inspection camera", None, None)
        if not self.offscreen_window:
            glfw.terminate()
            raise RuntimeError("无法创建离屏 GLFW 窗口")
        glfw.make_context_current(self.offscreen_window)
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        self.cam_view = mujoco.MjvCamera()
        self.cam_view.type = mujoco.mjtCamera.mjCAMERA_FIXED

    def _capture_image(self):
        self.cam_view.fixedcamid = self.camera_id
        viewport = mujoco.MjrRect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT)
        # The live/capture paths in bridge_core.py hide the wrist and hand
        # meshes so they cannot cover the gauge.  Do the same here, otherwise a
        # pose tuned in this window would not faithfully match run_inspection.
        hidden = []
        for geom_id in range(self.model.ngeom):
            name = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "")
            if name.startswith(("arm_l/", "hand_l/")):
                hidden.append((geom_id, float(self.model.geom_rgba[geom_id, 3])))
                self.model.geom_rgba[geom_id, 3] = 0.0
        try:
            mujoco.mjv_updateScene(
                self.model, self.data, mujoco.MjvOption(), mujoco.MjvPerturb(),
                self.cam_view, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
            mujoco.mjr_render(viewport, self.scene, self.context)
            rgb = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            mujoco.mjr_readPixels(rgb, None, viewport, self.context)
        finally:
            for geom_id, alpha in hidden:
                self.model.geom_rgba[geom_id, 3] = alpha
        return np.ascontiguousarray(np.flipud(rgb))

    def _apply_pose(self):
        self._clip_pose()
        self.data.qpos[self.qpos_addresses] = self.pose
        mujoco.mj_forward(self.model, self.data)

    def _init_figure(self):
        plt.ion()
        self.fig = plt.figure(figsize=(10, 9))
        try:
            self.fig.canvas.manager.set_window_title("Inspection Arm Tuner")
        except Exception:
            pass
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        ax_img = self.fig.add_axes([0.05, 0.43, 0.90, 0.52])
        self.image_artist = ax_img.imshow(
            np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8))
        ax_img.axis("off")
        self.info_text = self.fig.text(0.05, 0.955, "", family="monospace",
                                       fontsize=9)
        self.fig.text(
            0.05, 0.405,
            "滑块单位：度；保存到 JSON 时自动转换为弧度。相机使用 XML 固定方向。"
            "  S 保存，R 复位，Esc 退出。", fontsize=8, color="dimgray")

        ax_radio = self.fig.add_axes([0.05, 0.16, 0.18, 0.20])
        self.radio = RadioButtons(ax_radio, self.stop_ids, active=self.stop_index)
        self.radio.on_clicked(self._on_stop_selected)
        ax_radio.set_title("巡检点", fontsize=9)

        ax_save = self.fig.add_axes([0.05, 0.105, 0.18, 0.038])
        self.save_button = Button(ax_save, "保存当前点位")
        self.save_button.on_clicked(lambda event: self.save())
        ax_reset = self.fig.add_axes([0.05, 0.055, 0.18, 0.038])
        self.reset_button = Button(ax_reset, "复位当前点位")
        self.reset_button.on_clicked(
            lambda event: self._reset_current_stop())
        self.status_text = self.fig.text(0.28, 0.075, "", fontsize=9)

        self.sliders = []
        for index in range(6):
            lo, hi = np.degrees(self.joint_limits[index])
            value = float(np.degrees(self.pose[index]))
            ax = self.fig.add_axes([0.30, 0.35 - index * 0.043, 0.62, 0.026])
            slider = Slider(
                ax, "J%d" % (index + 1), float(lo), float(hi), valinit=value,
                valfmt="%.2f°", color="steelblue")
            slider.on_changed(lambda value, i=index: self._on_slider(i, value))
            self.sliders.append(slider)

    def _sync_sliders(self):
        self._updating_controls = True
        try:
            for index, slider in enumerate(self.sliders):
                value = float(np.degrees(self.pose[index]))
                if abs(slider.val - value) > 1e-9:
                    slider.set_val(value)
        finally:
            self._updating_controls = False

    def _on_slider(self, index, value):
        if self._updating_controls:
            return
        self.pose[index] = math.radians(float(value))
        self._apply_pose()
        self.status_text.set_text("未保存的修改")
        self.status_text.set_color("darkorange")

    def _on_stop_selected(self, label):
        if label not in self.stop_ids:
            return
        self.stop_index = self.stop_ids.index(label)
        self._set_stop(self.stop_index)
        self._sync_sliders()
        self.status_text.set_text("已切换到 %s" % label)
        self.status_text.set_color("black")

    def _reset_current_stop(self):
        self._set_stop(self.stop_index)
        self._sync_sliders()
        self.status_text.set_text("已复位；尚未保存")
        self.status_text.set_color("black")

    def _on_key(self, event):
        key = (event.key or "").lower()
        if key == "s":
            self.save()
        elif key == "r":
            self._reset_current_stop()
        elif key == "escape":
            self._running = False

    def _update_figure(self, image):
        self.image_artist.set_data(image)
        stop = self.stop
        self.info_text.set_text(
            "%s | target=%s | camera=%s\n"
            "q(rad) = %s\nq(deg) = %s" % (
                stop["id"], stop["target"], self.camera_name,
                " ".join("%+.6f" % value for value in self.pose),
                " ".join("%+.2f" % math.degrees(value) for value in self.pose)))
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def save(self):
        """Write only the selected stop's six joint values, preserving JSON."""
        self._apply_pose()
        values = [float("%.6f" % value) for value in self.pose]
        self.stops[self.stop_index]["left_arm_pose"] = values
        backup = Path(str(self.route_path) + ".bak")
        try:
            shutil.copy2(self.route_path, backup)
            fd, temp_name = tempfile.mkstemp(
                prefix=self.route_path.name + ".", suffix=".tmp",
                dir=str(self.route_path.parent), text=True)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    json.dump(self.route, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp_name, self.route_path)
            finally:
                if os.path.exists(temp_name):
                    os.unlink(temp_name)
            # Reload catches malformed output immediately.
            with self.route_path.open(encoding="utf-8") as stream:
                json.load(stream)
            self.status_text.set_text(
                "已保存 %s（备份：%s）" % (self.stop["id"], backup.name))
            self.status_text.set_color("darkgreen")
            print("[保存] %s -> %s" % (self.stop["id"], self.route_path))
        except Exception as exc:
            self.status_text.set_text("保存失败：%s" % exc)
            self.status_text.set_color("red")
            print("[保存] 失败：%s" % exc)

    def run(self):
        print("巡检机械臂调参器：六个滑块控制左臂关节，单位为度")
        print("当前点位: %s；S 保存，R 复位，Esc 退出" % self.stop["id"])
        self._init_figure()
        try:
            while self._running and self.viewer.is_running():
                self._apply_pose()
                image = self._capture_image()
                self._update_figure(image)
                self.viewer.sync()
                time.sleep(FRAME_PERIOD_SEC)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(self.fig)
            if self.viewer is not None:
                self.viewer.close()
            if self.offscreen_window is not None:
                self._glfw.destroy_window(self.offscreen_window)
                self._glfw.terminate()
            print("已退出")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", default=str(DEFAULT_ROUTE),
                        help="inspection_route.json 路径")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE),
                        help="ARIAC MuJoCo 场景 XML 路径")
    parser.add_argument("--stop", default=None,
                        help="启动点位 id 或索引（默认第一个；例如 tank_pressure）")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    InspectionArmTuner(args.route, args.scene, args.stop).run()
