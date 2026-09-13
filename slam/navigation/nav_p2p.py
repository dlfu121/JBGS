#!/usr/bin/env python3.8
"""点对点导航节点：3D SLAM 地图 + TF 定位 + Lazy Theta* 规划 + 路径跟随。"""

# Mrs.yoki,Keep up the good work!

"""链路:
  /map(占据栅格) + TF(map->odom->base_footprint)  ->  Lazy Theta* 规划
  /nav_goal(目标点)  ->  触发规划
  Lazy Theta* 路径(any-angle)  ->  安全折点跟随(全向底盘, 可侧移)  ->  /cmd_vel
  bridge_ariac.py 收到外部 /cmd_vel 会自动停止巡视、交出控制权。

必须 python3.8 跑（同 slam/bridge/bridge_ariac.py，Foxy 的 rclpy 只有 cpython-38 扩展），
use_sim_time 必须 true（桥接发布 /clock）。

地图优先级: 优先订阅 rtabmap 在线 /map（最新，含动态障碍）；若一直收不到
则回退加载已保存的场景地图（静态，可离线测试）。

订阅:
  /map        nav_msgs/OccupancyGrid      占据栅格地图
  /nav_goal   geometry_msgs/PoseStamped   目标点 (map 系)
  TF           map -> odom -> base_footprint
发布:
  /cmd_vel    geometry_msgs/Twist
  /nav_path   nav_msgs/Path
  /nav_status std_msgs/String              状态: IDLE/PLANNING/FOLLOWING/ALIGNING/ARRIVED/NO_MAP/NO_POSE/STUCK/UNREACHABLE

用法:
  # 终端1: 先跑 3D SLAM + 桥接，让狗巡视建图:
  ./slam/mapping/run_slam_3d.sh --patrol
  # 终端2: 起导航节点（先 source /opt/ros/foxy/setup.bash）:
  python3.8 slam/navigation/nav_p2p.py
  # 终端3: 发目标点（map 系）。当前 ARIAC 狗起点是 map 原点，
  #        即世界(4.0, 4.6)；可用 slam/navigation/send_goal.py --world 自动换算。
  ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
"""
import heapq
import math
import os
import sys
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE_DIR = os.path.join(HERE, "..", "bridge")
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

from dynamic_dwa import (  # noqa: E402
    DWAConfig, HolonomicDWA, LocalDetour,
    detour_path_index, path_corridor_clearance,
    path_lookahead)

SAVED_MAP = os.path.join(HERE, "..", "..", "maps", "ariac",
                         "ariac_map_3d.pgm")
SAVED_YAML = os.path.join(HERE, "..", "..", "maps", "ariac",
                          "ariac_map_3d.yaml")

# --- 规划/跟随参数 ---
ROBOT_RADIUS = 0.32          # 底盘碰撞盒半宽 0.315 + 余量
INFLATE = 0.18               # 地图/定位/跟踪误差的额外膨胀余量(m)
GOAL_TOL = 0.08              # 到点判定半径(m)，桌前目标采用较小误差
GOAL_YAW_TOL = math.radians(1.5)  # 最终朝向误差
PATH_POINT_TOL = 0.12        # 必须接近当前折点后才进入下一段(m)
CORNER_SLOW_DIST = 0.55      # 急转弯前后减速范围(m)
CORNER_V_MIN = 0.30          # 急转弯附近线速度上限(m/s)
CORNER_MIN_ANGLE = math.radians(30.0)
V_MAX = 1.0                  # 最大线速度(m/s)，10Hz 感知下每帧最多 10cm
W_MAX = 1.1                  # 最大角速度(rad/s)
DEVIATE_REPLAN = 0.6         # 偏离路径多远重规划(m)
STUCK_TIMEOUT = 3.0          # 被堵判定时间(s)
STUCK_DIST = 0.03            # 该时间内移动不足(m) 视为被堵
MAX_PLAN_FAIL = 3            # 连续规划失败几次后报 UNREACHABLE
HARD_CLEARANCE = ROBOT_RADIUS + INFLATE  # 小于此值的栅格不可通行
PREFERRED_CLEARANCE = 0.85    # 软代价开始消退的安全净距(m)
LAMBDA_GEO = 0.0              # 0=原始最短路；>0=安全感知路径
ROBOT_HEIGHT = 0.60
HEIGHT_SAFETY_MARGIN = 0.10
HARD_HEIGHT = ROBOT_HEIGHT + HEIGHT_SAFETY_MARGIN
PREFERRED_HEIGHT = 1.00
LAMBDA_HEIGHT = 1.0
HEIGHT_POINT_MIN_Z = 0.12
SAVED_CLOUD = os.path.join(HERE, "..", "..", "maps", "ariac",
                           "ariac_map_3d_cloud.ply")

# ARIAC VLA needs to park beneath the assembly-table overhang.  Keep the
# saved map's real occupied/unknown cells hard, but do not apply the circular
# mobile-base inflation to the known-free strip immediately in front of that
# table.  Coordinates are in the saved ARIAC map frame (world - (4.0, 4.6)).
ARIAC_TABLE_APPROACH_X = (2.70, 4.10)
ARIAC_TABLE_APPROACH_Y = (4.70, 5.60)
ARIAC_ASSEMBLY_TABLE_X = (2.70, 4.10)
# The robot-facing table edge was trimmed from world Y=9.96 to Y=10.20.
ARIAC_ASSEMBLY_TABLE_Y = (5.58, 6.26)

# The second inspection leg runs south along the east side of the horizontal
# tank.  The saved lidar map contains a thin/irregular silhouette there, so a
# circular 0.50 m inflation still leaves the planned centerline only a few
# centimetres from the tank.  Add a scene-specific rectangular safety band on
# that side.  Coordinates are in the ARIAC map frame (world - dog_home).
# Keep the band north of the gauge approach so the tank goal remains reachable
# from the south instead of being swallowed by the extra inflation.
ARIAC_TANK_SIDE_X = (-5.78, -5.20)
ARIAC_TANK_SIDE_Y = (-4.60, -1.10)

# --- 状态 ---
S_IDLE, S_PLANNING, S_FOLLOWING = "IDLE", "PLANNING", "FOLLOWING"
S_ARRIVED, S_NO_MAP, S_NO_POSE, S_STUCK, S_UNREACH = "ARRIVED", "NO_MAP", "NO_POSE", "STUCK", "UNREACHABLE"
S_DYNAMIC = "DYNAMIC_AVOID"
S_ALIGNING = "ALIGNING"

# --- 条件式实时局部避障 ---
# Only returns not explained by the saved static map trigger DWA.  Feeding
# every known wall/shelf return into the local planner makes it take over
# permanently even with --no-dynamic-person and prevents goal arrival.
# Keep the local safety layer in world-height coordinates.  The ARIAC
# launcher lowers dog_base by 8 cm so its wheel mesh sits on the floor; a
# fixed lidar-frame cutoff consequently admitted grazing floor returns as
# obstacles directly under the robot and made DWA command permanent zero.
DYNAMIC_WORLD_Z_MIN = 0.10  # ignore floor/grazing returns below 10 cm
DYNAMIC_WORLD_Z_MAX = 1.25  # ignore returns above the robot safety volume
DYNAMIC_STATIC_MARGIN = 0.18  # 靠近静态障碍的回波不视为新增动态物体
DYNAMIC_VOXEL = 0.12
DYNAMIC_CORRIDOR = 0.62
# Start a visible detour within the local planning window.
DYNAMIC_FORWARD = 2.8
# Trigger local avoidance only for returns in the robot's forward swept body
# band.  Parallel shelf faces are deliberately outside this band in a narrow
# aisle, while a new object actually blocking the aisle remains visible to
# DWA and is scored against the full point cloud.
LOCAL_TRIGGER_HALF_WIDTH = ROBOT_RADIUS + 0.12
DYNAMIC_RELEASE_SEC = 1.0
DYNAMIC_CLOUD_TIMEOUT = 0.45
DWA_PERIOD = 0.10


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _world_height_mask(points, sensor_world_z):
    """Select local-frame points inside the robot's world-height envelope."""
    world_z = points[:, 2] + float(sensor_world_z)
    return ((world_z > DYNAMIC_WORLD_Z_MIN)
            & (world_z < DYNAMIC_WORLD_Z_MAX))


def _pgm_to_occupancy(img, occupied_thresh, free_thresh, negate):
    """Convert a trinary map-server PGM to 100/0/-1 occupancy values."""
    probability = (img if negate else 255 - img).astype(np.float64) / 255.0
    occupancy = np.full(img.shape, -1, dtype=np.int8)
    occupancy[probability > occupied_thresh] = 100
    occupancy[probability < free_thresh] = 0

    # map_saver uses 205 as the trinary unknown marker.  Preserve it even if
    # a hand-edited YAML free_thresh would otherwise classify it as free.
    occupancy[img == 205] = -1
    return occupancy


def _inflate_occupancy(data, height, width, resolution, clearance):
    """Build a binary cost map and metric obstacle-clearance field."""
    occupancy = np.asarray(data, dtype=np.float64).reshape(height, width)
    blocked = (occupancy < 0.0) | (occupancy > 50.0)
    distance = distance_transform_edt(~blocked) * resolution
    return np.where(distance < clearance, 1.0, 0.0), distance


def _remove_ariac_table_front_inflation(occupancy, cost, resolution, origin):
    """Restore raw-free cells in the VLA assembly-table approach strip.

    This deliberately removes only inflation, never original occupied or
    unknown cells.  It is therefore a local approach policy rather than a
    general map-erasing operation.
    """
    occupancy = np.asarray(occupancy).reshape(cost.shape)
    rows, columns = np.indices(cost.shape)
    cell_x = float(origin[0]) + (columns + 0.5) * float(resolution)
    cell_y = float(origin[1]) + (rows + 0.5) * float(resolution)
    x0, x1 = ARIAC_TABLE_APPROACH_X
    y0, y1 = ARIAC_TABLE_APPROACH_Y
    approach = ((cell_x >= x0) & (cell_x <= x1) &
                (cell_y >= y0) & (cell_y < y1))
    restored = approach & (occupancy == 0)
    cost[restored] = 0.0
    return int(np.count_nonzero(restored))


def _inflate_ariac_tank_side(occupancy, cost, resolution, origin):
    """Add extra hard clearance along the tank's east side.

    Only raw-free cells are changed.  Existing occupied/unknown cells remain
    untouched, and the south end is deliberately open for the tank gauge
    standoff pose.
    """
    occupancy = np.asarray(occupancy).reshape(cost.shape)
    rows, columns = np.indices(cost.shape)
    cell_x = float(origin[0]) + (columns + 0.5) * float(resolution)
    cell_y = float(origin[1]) + (rows + 0.5) * float(resolution)
    x0, x1 = ARIAC_TANK_SIDE_X
    y0, y1 = ARIAC_TANK_SIDE_Y
    side = ((cell_x >= x0) & (cell_x <= x1) &
            (cell_y >= y0) & (cell_y <= y1))
    added = side & (occupancy == 0) & (cost <= 0.0)
    cost[added] = 1.0
    return int(np.count_nonzero(added))


def _points_in_ariac_assembly_table(points):
    """Return a mask for lidar returns belonging to the known table body."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    x0, x1 = ARIAC_ASSEMBLY_TABLE_X
    y0, y1 = ARIAC_ASSEMBLY_TABLE_Y
    return ((points[:, 0] >= x0) & (points[:, 0] <= x1) &
            (points[:, 1] >= y0) & (points[:, 1] <= y1))


def _point_in_ariac_table_approach(point):
    x0, x1 = ARIAC_TABLE_APPROACH_X
    y0, y1 = ARIAC_TABLE_APPROACH_Y
    return x0 <= point[0] <= x1 and y0 <= point[1] < y1


def compute_height_cost(clearance_h, hard_h=HARD_HEIGHT,
                        preferred_h=PREFERRED_HEIGHT):
    """Map vertical clearance to a hard/soft cost in [0, 1]."""
    if clearance_h <= hard_h:
        return float("inf")
    if clearance_h >= preferred_h:
        return 0.0
    ratio = (preferred_h - clearance_h) / max(preferred_h - hard_h, 1e-9)
    return float(max(0.0, min(1.0, ratio * ratio)))


def _height_clearance_from_points(points, height, width, resolution, origin,
                                  min_z=HEIGHT_POINT_MIN_Z):
    """Project 3D points onto the existing map grid and keep lowest overhead z."""
    result = np.full((height, width), np.inf, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return result
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > min_z)
    points = points[valid]
    if points.size == 0:
        return result
    ix = np.floor((points[:, 0] - origin[0]) / resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - origin[1]) / resolution).astype(np.int64)
    valid = ((ix >= 0) & (ix < width) & (iy >= 0) & (iy < height))
    ids = iy[valid] * width + ix[valid]
    np.minimum.at(result.ravel(), ids, points[valid, 2])
    return result


def _read_ply_xyz(path):
    """Read x/y/z from the binary PLY exported by RTAB-Map."""
    with open(path, "rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY header is incomplete")
            text = line.decode("ascii").strip()
            header.append(text)
            if text == "end_header":
                break
        vertex_count = next(int(line.split()[2]) for line in header
                            if line.startswith("element vertex "))
        properties = []
        in_vertex = False
        for line in header:
            if line.startswith("element "):
                in_vertex = line.startswith("element vertex ")
            elif in_vertex and line.startswith("property "):
                fields = line.split()
                if len(fields) != 3 or fields[1] != "float":
                    raise ValueError("PLY vertex properties must be float")
                properties.append(fields[2])
        if not properties or not all(name in properties for name in ("x", "y", "z")):
            raise ValueError("PLY vertex has no x/y/z properties")
        dtype = np.dtype([(name, "<f4") for name in properties])
        data = np.fromfile(stream, dtype=dtype, count=vertex_count)
        if len(data) != vertex_count:
            raise ValueError("PLY vertex data is incomplete")
        return np.column_stack((data["x"], data["y"], data["z"]))


def _line_cells(cost, x0, y0, x1, y1):
    """Yield Bresenham cells and reject diagonal corner cutting consistently."""
    height, width = cost.shape
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    step_x = 1 if x1 > x0 else -1
    step_y = 1 if y1 > y0 else -1
    error = dx - dy
    x, y = x0, y0

    cells = []
    while True:
        if not (0 <= x < width and 0 <= y < height) or cost[y, x] > 0:
            return None
        cells.append((x, y))
        if x == x1 and y == y1:
            return cells

        twice_error = 2 * error
        move_x = twice_error > -dy
        move_y = twice_error < dx
        next_x = x + step_x if move_x else x
        next_y = y + step_y if move_y else y

        if move_x and move_y:
            # Passing diagonally between either occupied orthogonal neighbour
            # clips that cell's corner for a finite-radius robot.
            if not (0 <= next_x < width and 0 <= next_y < height):
                return None
            if cost[y, next_x] > 0 or cost[next_y, x] > 0:
                return None
        if move_x:
            error -= dy
        if move_y:
            error += dx
        x, y = next_x, next_y


def _line_is_clear(cost, x0, y0, x1, y1):
    """Conservative Bresenham check which also forbids diagonal corner cuts."""
    cells = _line_cells(cost, x0, y0, x1, y1)
    return cells is not None


def _advance_path_index(path, index, x, y, tolerance=PATH_POINT_TOL):
    """Advance only after reaching each sparse Theta* waypoint."""
    if not path:
        return 0
    index = max(0, min(index, len(path) - 1))
    while index < len(path) - 1:
        px, py = path[index]
        if math.hypot(px - x, py - y) > tolerance:
            break
        index += 1
    return index


def _corner_angle(path, index):
    if not (0 < index < len(path) - 1):
        return 0.0
    ax = path[index][0] - path[index - 1][0]
    ay = path[index][1] - path[index - 1][1]
    bx = path[index + 1][0] - path[index][0]
    by = path[index + 1][1] - path[index][1]
    norm = math.hypot(ax, ay) * math.hypot(bx, by)
    if norm < 1e-9:
        return 0.0
    cosine = max(-1.0, min(1.0, (ax * bx + ay * by) / norm))
    return math.acos(cosine)


def _corner_speed_limit(path, target_index, x, y):
    """Limit speed while approaching or leaving a meaningful path corner."""
    limit = V_MAX
    for corner_index in (target_index - 1, target_index):
        if _corner_angle(path, corner_index) < CORNER_MIN_ANGLE:
            continue
        cx, cy = path[corner_index]
        distance = math.hypot(cx - x, cy - y)
        if distance >= CORNER_SLOW_DIST:
            continue
        span = max(CORNER_SLOW_DIST - PATH_POINT_TOL, 1e-6)
        ratio = max(0.0, (distance - PATH_POINT_TOL) / span)
        limit = min(limit, CORNER_V_MIN + (V_MAX - CORNER_V_MIN) * ratio)
    return limit


class NavP2P(Node):
    def __init__(self, use_saved=False, scene="ariac", lambda_geo=LAMBDA_GEO,
                 lambda_height=LAMBDA_HEIGHT, robot_height=ROBOT_HEIGHT,
                 height_safety_margin=HEIGHT_SAFETY_MARGIN,
                 preferred_height=PREFERRED_HEIGHT):
        super().__init__("nav_p2p")
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self._use_saved = use_saved
        self.scene = scene
        self.lambda_geo = max(0.0, float(lambda_geo))
        self.lambda_height = max(0.0, float(lambda_height))
        self.robot_height = float(robot_height)
        self.hard_height = self.robot_height + float(height_safety_margin)
        self.preferred_height = float(preferred_height)
        self.map = None            # 当前 OccupancyGrid（在线或保存的）
        self.cost = None           # 膨胀后代价栅格(np.ndarray float, 0 可走/1 不可走)
        self.clearance = None      # 每个栅格中心到障碍/未知区的距离(m)
        self.traversable = None    # 唯一的栅格可通行判断源
        self.height_clearance = None
        self.height_cost = None
        self.grid_shape = None
        self.path = []             # list of (x, y) map 系
        self.goal = None           # (x, y) map 系
        self.goal_yaw = None       # 可选最终朝向；零四元数表示不约束
        self.path_idx = 0
        self.status = S_IDLE
        self.plan_fail = 0
        self.last_pose = None
        self._stuck_t0 = None
        self._stuck_pos = None
        self.last_plan_stats = {}
        # Point-cloud returns in the local planning area.  ``dynamic_points``
        # is kept as a diagnostic subset (returns that are not explained by
        # the saved static map), while ``local_obstacle_points`` is the actual
        # reactive-obstacle input to DWA.  Keeping all usable returns here is
        # important: a static object can be present in the live scene but be
        # missing from the saved map, and a known object can still be unsafe
        # if localization/map alignment is imperfect.
        self.dynamic_points = np.empty((0, 2), dtype=np.float64)
        self.local_obstacle_points = np.empty((0, 2), dtype=np.float64)
        self._last_cloud_time = None
        self._dynamic_until = 0.0
        self._dynamic_resume_until = 0.0
        self._dynamic_active = False
        self._last_dwa_time = -1e9
        self._last_dwa_log = -1e9
        self._dwa_command = (0.0, 0.0, 0.0)
        self._last_command = (0.0, 0.0, 0.0)
        self.dwa = HolonomicDWA(DWAConfig(
            max_forward=min(V_MAX, 0.55), max_yaw_rate=W_MAX,
            dynamic_clearance=ROBOT_RADIUS + 0.15))
        self.detour = LocalDetour(clearance=self.dwa.config.dynamic_clearance)
        self._last_detour_time = -1e9

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "nav_path", 10)
        self.pub_status = self.create_publisher(String, "nav_status", 10)

        # 使用 TRANSIENT_LOCAL 接收 rtabmap 的最新 /map 发布。
        from rclpy.qos import (QoSProfile, QoSHistoryPolicy,
                               QoSReliabilityPolicy, QoSDurabilityPolicy)
        planning_map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_planning_map = self.create_publisher(
            OccupancyGrid, "planning_map", planning_map_qos)
        self.pub_height_cost = self.create_publisher(
            OccupancyGrid, "height_cost_map", planning_map_qos)

        map_qos = QoSProfile(depth=2,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_map = self.create_subscription(OccupancyGrid, "map",
                                                self.on_map, map_qos)
        self.sub_goal = self.create_subscription(PoseStamped, "nav_goal",
                                                 self.on_goal, 10)
        pc_qos = QoSProfile(depth=2,
                            history=QoSHistoryPolicy.KEEP_LAST,
                            reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_cloud = self.create_subscription(
            PointCloud2, "pointcloud", self.on_pointcloud, pc_qos)

        self.create_timer(0.02, self.tick)     # 50 Hz 跟随

        if use_saved:
            # --use-saved 模式：立即加载地图，不订阅 /map
            self.get_logger().info("--use-saved 模式，加载 %s" % SAVED_MAP)
            self.map = self._load_saved_map()
            if self.map is not None:
                self._rebuild_cost()
                self._publish_planning_map()
                self._publish_height_cost_map()
            else:
                self.get_logger().error("加载保存地图失败")
        else:
            # 在线模式：订阅 /map，3 秒后回退到保存地图
            self._fallback_timer = self.create_timer(3.0, self._try_saved_map)

        self.get_logger().info(
            "nav_p2p up: robot_radius=%.2f inflate=%.2f path_point_tol=%.2f "
            "saved_map=%s" % (ROBOT_RADIUS, INFLATE, PATH_POINT_TOL,
                              os.path.basename(SAVED_MAP) if os.path.exists(SAVED_MAP) else "无"))

    # ---------------- 地图 ----------------
    def on_map(self, msg):
        if self._use_saved:
            return
        self.map = msg
        self._rebuild_cost()
        self._publish_planning_map()
        self._publish_height_cost_map()
        if self.goal is not None and self.status in (S_FOLLOWING, S_IDLE, S_STUCK, S_UNREACH):
            self._plan()

    def _try_saved_map(self):
        """收不到在线 /map 时回退到已保存地图（离线测试用）。"""
        if self.map is not None:
            self._fallback_timer.cancel()
            return
        if not os.path.exists(SAVED_MAP):
            return
        self.get_logger().warn("3s 内未收到 /map，回退加载 %s" % SAVED_MAP)
        self.map = self._load_saved_map()
        if self.map is not None:
            self._rebuild_cost()
            self._publish_planning_map()
            self._publish_height_cost_map()
            self._fallback_timer.cancel()
            if self.goal is not None:
                self._plan()

    def _load_saved_map(self):
        """解析保存的 PGM/YAML 地图文件为 OccupancyGrid。"""
        try:
            with open(SAVED_MAP, "rb") as f:
                magic = f.readline().strip()
                if magic != b"P5":
                    raise ValueError("不是 P5 PGM")
                while True:
                    line = f.readline()
                    if line.startswith(b"#"):
                        continue
                    parts = line.split()
                    if len(parts) >= 2 and len(parts) < 3:
                        w, h = int(parts[0]), int(parts[1])
                        continue
                    if len(parts) == 1:
                        break
                data = np.frombuffer(f.read(), dtype=np.uint8).astype(np.int16)
                if data.size != w * h:
                    raise ValueError("PGM 数据长度不符")
                img = data.reshape(h, w)
            # PGM 从上到下存储，OccupancyGrid 从下到上（origin 在左下角），需翻转
            img = np.flipud(img)
            res, origin = 0.05, [-2.97, -11.5, 0.0]
            occupied_thresh, free_thresh, negate = 0.65, 0.196, 0
            if os.path.exists(SAVED_YAML):
                y = {}
                for ln in open(SAVED_YAML, encoding="utf-8"):
                    if ":" in ln and not ln.strip().startswith("#"):
                        k, v = ln.split(":", 1)
                        y[k.strip()] = v.strip()
                res = float(y.get("resolution", res))
                origin = [float(x) for x in
                          y.get("origin", "[-2.97, -11.5, 0]").strip("[]").split(",")]
                occupied_thresh = float(y.get("occupied_thresh", occupied_thresh))
                free_thresh = float(y.get("free_thresh", free_thresh))
                negate = int(y.get("negate", negate))
            g = OccupancyGrid()
            g.header.frame_id = "map"
            g.header.stamp = self.get_clock().now().to_msg()
            g.info.resolution = res
            g.info.width = w
            g.info.height = h
            g.info.origin.position.x, g.info.origin.position.y = origin[0], origin[1]
            # 黑=占据、白=空闲、205=未知，按 map_server trinary 语义加载。
            occ = _pgm_to_occupancy(
                img, occupied_thresh, free_thresh, negate)
            g.data = [int(v) for v in occ.flatten()]
            self.get_logger().info(
                "加载保存地图 %dx%d res=%.3f origin=(%.2f, %.2f)"
                % (w, h, res, origin[0], origin[1]))
            return g
        except Exception as e:
            self.get_logger().error("加载保存地图失败: %s" % e)
            return None

    def _rebuild_cost(self):
        """占据栅格 -> EDT 距离场 -> 膨胀后代价栅格。"""
        m = self.map
        self.grid_shape = (m.info.height, m.info.width)
        self.cost, self.clearance = _inflate_occupancy(
            m.data, m.info.height, m.info.width, m.info.resolution,
            ROBOT_RADIUS + INFLATE)
        if self.scene == "ariac":
            origin = m.info.origin.position
            restored = _remove_ariac_table_front_inflation(
                m.data, self.cost, m.info.resolution, (origin.x, origin.y))
            self.get_logger().info(
                "ARIAC 桌前接近区已取消障碍膨胀: %d cells" % restored)
            tank_added = _inflate_ariac_tank_side(
                m.data, self.cost, m.info.resolution, (origin.x, origin.y))
            self.get_logger().info(
                "ARIAC 水箱东侧增加安全膨胀: %d cells (x=%.2f..%.2f, y=%.2f..%.2f)"
                % (tank_added, ARIAC_TANK_SIDE_X[0], ARIAC_TANK_SIDE_X[1],
                   ARIAC_TANK_SIDE_Y[0], ARIAC_TANK_SIDE_Y[1]))
        self.traversable = self.cost <= 0.0
        self._rebuild_height_cost()

    def _rebuild_height_cost(self):
        """Project the saved 3D cloud onto this map's existing 2D grid."""
        self.height_clearance = np.full(self.grid_shape, np.inf, dtype=np.float64)
        if not os.path.exists(SAVED_CLOUD) or self.map is None:
            self.height_cost = np.zeros(self.grid_shape, dtype=np.float64)
            return
        try:
            points = _read_ply_xyz(SAVED_CLOUD)
            origin = self.map.info.origin.position
            self.height_clearance = _height_clearance_from_points(
                points, self.grid_shape[0], self.grid_shape[1],
                self.map.info.resolution, (origin.x, origin.y))
            self.height_cost = np.zeros(self.grid_shape, dtype=np.float64)
            finite = np.isfinite(self.height_clearance)
            soft = finite & (self.height_clearance < self.preferred_height)
            span = max(self.preferred_height - self.hard_height, 1e-9)
            ratio = ((self.preferred_height - self.height_clearance[soft]) / span)
            self.height_cost[soft] = np.clip(ratio, 0.0, 1.0) ** 2
            self.traversable &= ((~finite) |
                                 (self.height_clearance > self.hard_height))
        except (OSError, ValueError) as error:
            self.get_logger().warn("高度代价地图不可用: %s" % error)
            self.height_cost = np.zeros(self.grid_shape, dtype=np.float64)

    # ---------------- 目标与规划 ----------------
    def on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y)
        q = msg.pose.orientation
        q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        self.goal_yaw = None if q_norm < 1e-6 else math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.plan_fail = 0
        # Never retain and follow a path belonging to an older goal when the
        # replacement goal turns out to be invalid or unreachable.
        self.path = []
        self.path_idx = 0
        self._stuck_pos, self._stuck_t0 = None, None
        self._dynamic_until = 0.0
        self._dynamic_resume_until = 0.0
        self._dynamic_active = False
        self._dwa_command = (0.0, 0.0, 0.0)
        self.detour.reset()
        self._last_detour_time = -1e9
        yaw_text = ("不约束" if self.goal_yaw is None
                    else "%.1f deg" % math.degrees(self.goal_yaw))
        self.get_logger().info(
            "收到目标 (%.2f, %.2f), 最终朝向=%s, 开始规划"
            % (self.goal[0], self.goal[1], yaw_text))
        if self.map is None:
            self._set_status(S_NO_MAP)
            return
        self._plan()

    def on_pointcloud(self, msg):
        """Convert live lidar returns into the local obstacle point set.

        The old implementation discarded every return that was already close
        to an obstacle in the saved map.  That was useful for isolating
        pedestrians, but it also meant that the controller had no local
        obstacle layer for a newly placed static object (or for a stale map).
        Keep all height-filtered returns inside the planning map for reactive
        collision checking, and retain the old novel-only subset for logs and
        diagnostics.
        """
        self._last_cloud_time = self._now_sec()
        if self.map is None or self.clearance is None or msg.width == 0:
            self.dynamic_points = np.empty((0, 2), dtype=np.float64)
            self.local_obstacle_points = np.empty((0, 2), dtype=np.float64)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, Time(), Duration(seconds=0.05))
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
            points = np.column_stack([
                raw[:, offset:offset + 4].copy().view(np.float32).ravel()
                for offset in (0, 4, 8)])
            points = points[np.isfinite(points).all(axis=1)]
            # The bridge publishes a level lidar frame, so map-frame height
            # is the local point height plus the TF translation.  Do not use
            # a hard-coded lidar-relative cutoff: the composed ARIAC model's
            # base/root height is intentionally configurable.
            points = points[_world_height_mask(
                points, transform.transform.translation.z)]
            if not len(points):
                self.dynamic_points = np.empty((0, 2), dtype=np.float64)
                self.local_obstacle_points = np.empty((0, 2), dtype=np.float64)
                return

            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            map_xy = np.column_stack((tx + c * points[:, 0] - s * points[:, 1],
                                      ty + s * points[:, 0] + c * points[:, 1]))
            origin = self.map.info.origin.position
            resolution = self.map.info.resolution
            ix = np.floor((map_xy[:, 0] - origin.x) / resolution).astype(np.int64)
            iy = np.floor((map_xy[:, 1] - origin.y) / resolution).astype(np.int64)
            height, width = self.grid_shape
            inside = ((ix >= 0) & (ix < width) & (iy >= 0) & (iy < height))
            # Points outside the planning map cannot be scored safely by the
            # local planner, so do not let them create a false avoidance
            # command near the map boundary.
            map_xy = map_xy[inside]
            ix = ix[inside]
            iy = iy[inside]
            if not len(map_xy):
                self.dynamic_points = np.empty((0, 2), dtype=np.float64)
                self.local_obstacle_points = np.empty((0, 2), dtype=np.float64)
                return

            novel = self.clearance[iy, ix] > DYNAMIC_STATIC_MARGIN

            def voxelize(values):
                if not len(values):
                    return np.empty((0, 2), dtype=np.float64)
                voxel = np.floor(values / DYNAMIC_VOXEL).astype(np.int64)
                _, unique = np.unique(voxel, axis=0, return_index=True)
                return values[np.sort(unique)]

            # During the deliberate VLA close approach, the known assembly
            # table face must agree with the global inflation exception above;
            # otherwise its lidar returns would immediately make DWA stop.
            # Only the fixed table footprint is ignored.  A person or object
            # in the raw-free approach strip remains in this live safety layer.
            local_points = map_xy
            if (self.scene == "ariac" and self.goal is not None and
                    _point_in_ariac_table_approach(self.goal)):
                local_points = map_xy[~_points_in_ariac_assembly_table(map_xy)]
            self.local_obstacle_points = voxelize(local_points)
            self.dynamic_points = voxelize(map_xy[novel])
        except Exception as error:
            self.dynamic_points = np.empty((0, 2), dtype=np.float64)
            self.local_obstacle_points = np.empty((0, 2), dtype=np.float64)
            self.get_logger().warn("动态点云变换失败: %s" % error,
                                   throttle_duration_sec=2.0)

    def _world_to_grid(self, x, y):
        o = self.map.info.origin.position
        r = self.map.info.resolution
        i = int(math.floor((x - o.x) / r))
        j = int(math.floor((y - o.y) / r))
        return i, j

    def _grid_to_world(self, i, j):
        o = self.map.info.origin.position
        r = self.map.info.resolution
        return o.x + (i + 0.5) * r, o.y + (j + 0.5) * r

    def _plan(self):
        if self.map is None or self.cost is None or self.goal is None:
            return
        pose = self._get_pose()
        if pose is None:
            # A map can be loaded correctly while localization is not yet
            # available.  Reporting this as NO_MAP made the saved-map launch
            # look broken even when the real issue was a missing TF chain.
            self._set_status(S_NO_POSE)
            self.get_logger().warn(
                "地图已加载但暂时没有 map->base_footprint 位姿；"
                "请确认 map->odom 静态 TF 和 bridge 的 odom->base_footprint 已发布",
                throttle_duration_sec=3.0)
            return
        self._set_status(S_PLANNING)
        sx, sy = self._world_to_grid(pose[0], pose[1])
        gx, gy = self._world_to_grid(self.goal[0], self.goal[1])
        h, w = self.grid_shape
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            self.plan_fail += 1
            self.get_logger().warn("目标超出地图范围")
            self._set_status(S_UNREACH)
            return
        path = self._astar(sx, sy, gx, gy)
        if path is None:
            self.plan_fail += 1
            self.get_logger().warn("Lazy Theta* 规划失败 (起点或目标被障碍包围?)")
            self._set_status(S_UNREACH)
            return
        self.plan_fail = 0
        min_clearance = min(self.clearance[j, i] for i, j in path)
        self.path = [self._grid_to_world(i, j) for i, j in path]
        self.path_idx = min(1, len(self.path) - 1)
        self.get_logger().info(
            "规划成功: %d 个路径点, 长度 %.2f m, 最小净距 %.2f m, "
            "展开 %d, LOS %d, 用时 %.2f ms, lambda_geo=%.2f"
            % (len(self.path), self.last_plan_stats.get("path_length_m", 0.0),
               min_clearance, self.last_plan_stats.get("expanded_nodes", 0),
               self.last_plan_stats.get("los_checks", 0),
               self.last_plan_stats.get("planning_time_ms", 0.0),
               self.lambda_geo))
        self._set_status(S_FOLLOWING)
        self._publish_path()

    def _check_plan_fail(self):
        if self.plan_fail >= MAX_PLAN_FAIL:
            self._set_status(S_UNREACH)

    def _astar(self, sx, sy, gx, gy):
        """Lazy Theta* — any-angle path planning on inflated cost grid."""
        h, w = self.grid_shape
        self._plan_stats = {
            "planning_time_ms": 0.0, "expanded_nodes": 0,
            "los_checks": 0, "los_cells_checked": 0,
            "waypoint_count": 0, "path_length_m": 0.0,
            "minimum_clearance_m": 0.0, "mean_clearance_m": 0.0,
            "success": False,
        }
        started = time.perf_counter()
        if not self.is_traversable(sx, sy) or not self.is_traversable(gx, gy):
            self._finish_plan_stats(started, None)
            return None

        SQRT2 = 1.4142135623730951
        # 8-connected neighbors
        DIRS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))

        start_key = sx * h + sy
        goal_key = gx * h + gy
        gcost = {start_key: 0.0}
        parent = {start_key: (sx, sy)}
        openq = [(math.hypot(sx - gx, sy - gy), sx, sy)]
        closed = set()
        reached = False

        while openq:
            _, cx, cy = heapq.heappop(openq)
            ckey = cx * h + cy
            if ckey in closed or ckey not in parent:
                continue

            self._plan_stats["expanded_nodes"] += 1
            # Lazy check: verify LOS to parent (修复: 原实现校验祖父点, 且父点为起点时
            # 跳过校验, 导致穿过障碍的"起点->终点"捷径永远不被发现)
            px, py = parent[ckey]
            if (px, py) != (cx, cy):
                if not self._line_of_sight(px, py, cx, cy):
                    # LOS failed: reconnect through a closed neighbour whose
                    # direct edge is itself collision-free.
                    best_g = 1e18
                    best_p = None
                    for dx, dy, cost in DIRS:
                        nx, ny = cx + dx, cy + dy
                        if not (0 <= nx < w and 0 <= ny < h):
                            continue
                        nkey = nx * h + ny
                        if (nkey not in closed or nkey not in gcost
                                or not self.line_of_sight((nx, ny), (cx, cy))):
                            continue
                        g_via = gcost[nkey] + self.edge_cost((nx, ny), (cx, cy))
                        if g_via < best_g:
                            best_g = g_via
                            best_p = (nx, ny)
                    if best_p is None:
                        # This optimistic Lazy-Theta node is unreachable.
                        # Remove its stale low cost so a later valid route may
                        # discover it again with a different parent.
                        parent.pop(ckey, None)
                        gcost.pop(ckey, None)
                        continue
                    parent[ckey] = best_p
                    gcost[ckey] = best_g

            if (cx, cy) == (gx, gy):
                closed.add(ckey)
                reached = True
                break
            closed.add(ckey)

            for dx, dy, step_cost in DIRS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if not self.neighbor_is_traversable(cx, cy, nx, ny):
                    continue
                nkey = nx * h + ny
                if nkey in closed:
                    continue

                # Try path through grandparent (Theta* update)
                px, py = parent[ckey]
                pkey = px * h + py
                g_pp = gcost[pkey] + self.edge_cost((px, py), (nx, ny))
                g_c = gcost[ckey] + self.edge_cost((cx, cy), (nx, ny))

                if g_pp < g_c:
                    # Lazy: assume LOS to grandparent passes
                    new_g = g_pp
                    new_parent = (px, py)
                else:
                    new_g = g_c
                    new_parent = (cx, cy)

                if new_g < gcost.get(nkey, 1e18):
                    gcost[nkey] = new_g
                    parent[nkey] = new_parent
                    hh = math.hypot(nx - gx, ny - gy)
                    heapq.heappush(openq, (new_g + hh, nx, ny))

        if not reached:
            self._finish_plan_stats(started, None)
            return None

        # Reconstruct path
        rev = []
        cur = (gx, gy)
        while cur != (sx, sy):
            rev.append(cur)
            ckey = cur[0] * h + cur[1]
            cur = parent[ckey]
        rev.append((sx, sy))
        path = list(reversed(rev))
        self._finish_plan_stats(started, path)
        return path

    def _line_of_sight(self, x0, y0, x1, y1):
        return self.line_of_sight((x0, y0), (x1, y1))

    def is_traversable(self, x, y):
        """唯一的栅格可通行接口；边界、未知和膨胀区均不可通行。"""
        mask = getattr(self, "traversable", None)
        if mask is None:
            mask = self.cost <= 0.0
        h, w = mask.shape
        return 0 <= x < w and 0 <= y < h and bool(mask[y, x])

    def neighbor_is_traversable(self, x0, y0, x1, y1):
        if not self.is_traversable(x1, y1):
            return False
        if x1 != x0 and y1 != y0:
            return self.is_traversable(x0, y1) and self.is_traversable(x1, y0)
        return True

    def line_of_sight(self, a, b):
        """唯一 LOS 接口；统计检查次数并禁止 corner cutting。"""
        if not hasattr(self, "_plan_stats"):
            self._plan_stats = {"los_checks": 0, "los_cells_checked": 0}
        self._plan_stats.setdefault("los_checks", 0)
        self._plan_stats.setdefault("los_cells_checked", 0)
        self._plan_stats["los_checks"] += 1
        cells = _line_cells(self.cost, a[0], a[1], b[0], b[1])
        if cells is None:
            return False
        ok = True
        for x, y in cells:
            self._plan_stats["los_cells_checked"] += 1
            if not self.is_traversable(x, y):
                ok = False
                break
        return ok

    def clearance_cost(self, x, y):
        """Return a normalized soft penalty: 0 at preferred clearance, 1 at hard limit."""
        if not self.is_traversable(x, y):
            return 1.0
        c = float(self.clearance[y, x])
        if c >= PREFERRED_CLEARANCE:
            return 0.0
        span = max(PREFERRED_CLEARANCE - HARD_CLEARANCE, 1e-9)
        return max(0.0, min(1.0, (PREFERRED_CLEARANCE - c) / span))

    def edge_cost(self, a, b):
        """Metric edge cost plus optional mean clearance penalty."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        cells = _line_cells(self.cost, a[0], a[1], b[0], b[1])
        if cells is None:
            return float("inf")
        total = length
        if getattr(self, "lambda_geo", LAMBDA_GEO) > 0.0 and getattr(
                self, "clearance", None) is not None:
            geo_penalties = [self.clearance_cost(x, y) for x, y in cells]
            total = length * (1.0 + getattr(self, "lambda_geo", LAMBDA_GEO) *
                              (sum(geo_penalties) / max(1, len(geo_penalties))))
        height_cost = getattr(self, "height_cost", None)
        if height_cost is not None:
            height_clearance = getattr(self, "height_clearance", None)
            if height_clearance is not None and any(
                    float(height_clearance[y, x]) <= getattr(
                        self, "hard_height", HARD_HEIGHT)
                    for x, y in cells):
                return float("inf")
            height_values = [float(height_cost[y, x]) for x, y in cells]
            total += getattr(self, "lambda_height", LAMBDA_HEIGHT) * (
                sum(height_values) / max(1, len(height_values)))
        return total

    def _finish_plan_stats(self, started, path):
        stats = getattr(self, "_plan_stats", {})
        stats["planning_time_ms"] = (time.perf_counter() - started) * 1000.0
        if path:
            stats["waypoint_count"] = len(path)
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(path, path[1:]))
            if getattr(self, "map", None) is not None:
                length *= float(self.map.info.resolution)
            stats["path_length_m"] = length
            clearance = getattr(self, "clearance", None)
            clear = ([float(clearance[y, x]) for x, y in path]
                     if clearance is not None else [0.0])
            stats["minimum_clearance_m"] = min(clear)
            stats["mean_clearance_m"] = sum(clear) / len(clear)
            stats["success"] = True
        self.last_plan_stats = dict(stats)

    # ---------------- 位姿与跟随 ----------------
    def _get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time(), Duration(seconds=1.0))
        except Exception:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        return (x, y, yaw)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _static_clearance_at(self, x, y):
        """DWA callback: negative means outside the static traversable map."""
        if self.map is None or self.clearance is None:
            return -1.0
        ix, iy = self._world_to_grid(x, y)
        if not self.is_traversable(ix, iy):
            return -1.0
        return float(self.clearance[iy, ix])

    def _dynamic_command(self, pose):
        """Run DWA while a novel obstacle intersects the path corridor."""
        if self._dynamic_active:
            self.path_idx = detour_path_index(
                pose[:2], self.path, self.path_idx, self._static_clearance_at)
        now = self._now_sec()
        cloud_fresh = (self._last_cloud_time is not None
                       and now - self._last_cloud_time <= DYNAMIC_CLOUD_TIMEOUT)
        points = (self.dynamic_points if cloud_fresh
                  else np.empty((0, 2)))
        corridor = path_corridor_clearance(
            points, pose[:2], self.path, self.path_idx, DYNAMIC_FORWARD,
            corridor_half_width=LOCAL_TRIGGER_HALF_WIDTH)
        if corridor <= DYNAMIC_CORRIDOR:
            self._dynamic_until = now + DYNAMIC_RELEASE_SEC
        active = now < self._dynamic_until
        if not active:
            if self._dynamic_active:
                self.get_logger().info("实时障碍已离开路径走廊，恢复全局路径跟随")
                self._dynamic_active = False
                self.detour.reset()
                self._dynamic_resume_until = now + 4.0
                self._stuck_pos = None
                self._stuck_t0 = None
                self._set_status(S_FOLLOWING)
            return None

        if not self._dynamic_active:
            self._dynamic_active = True
            self.get_logger().warn(
                "新增障碍进入前方路径走廊，启用全向 DWA "
                "(clearance=%.2fm, dynamic_points=%d)"
                % (corridor, len(points)))
            self._set_status(S_DYNAMIC)

        if now - self._last_dwa_time < DWA_PERIOD:
            return self._dwa_command
        self._last_dwa_time = now
        # Compare whole passages around the obstacle before asking DWA for
        # the next velocity. Penalising deviation from the blocked global
        # line itself otherwise makes a safe wait beat the first sidestep.
        obstacles = self.local_obstacle_points if cloud_fresh else points
        if now - self._last_detour_time >= 0.6:
            self._last_detour_time = now
            goal = path_lookahead(pose[:2], self.path, self.path_idx, 3.5)
            self.detour.plan(pose[:2], goal, obstacles, self._static_clearance_at)
        route = self.detour.path
        if route:
            nearest = int(np.argmin(np.linalg.norm(
                np.asarray(route) - np.asarray(pose[:2]), axis=1)))
            route_index = min(nearest + 1, len(route) - 1)
        else:
            route, route_index = self.path, self.path_idx
        target = path_lookahead(pose[:2], route, route_index, 1.2)
        command = self.dwa.plan(
            pose, target, route, route_index, obstacles,
            self._static_clearance_at, self._last_command)
        self._dwa_command = command if command is not None else (0.0, 0.0, 0.0)
        if now - self._last_dwa_log >= 1.0:
            self._last_dwa_log = now
            self.get_logger().info(
                "DWA cmd=(%.2f, %.2f, %.2f), dynamic_points=%d"
                % (self._dwa_command + (len(points),)))
        return self._dwa_command

    def tick(self):
        if self.goal is None:
            return                          # 空闲不发指令，让巡视接管
        if self.map is None or self.cost is None:
            self._set_status(S_NO_MAP)
            return
        if not self.path:
            # _plan() reports NO_POSE or UNREACHABLE.  An empty path does not
            # mean that the already-loaded map disappeared.
            return
        pose = self._get_pose()
        if pose is None:
            self._set_status(S_NO_POSE)
            return
        if (self._last_cloud_time is None
                or self._now_sec() - self._last_cloud_time > DYNAMIC_CLOUD_TIMEOUT):
            self._stop()
            self.get_logger().warn(
                "实时点云不可用或超时，已停车等待传感器恢复",
                throttle_duration_sec=2.0)
            return

        # 到点判定
        gx, gy = self.path[-1]
        if math.hypot(gx - pose[0], gy - pose[1]) < GOAL_TOL:
            if self.goal_yaw is not None:
                yaw_error = _wrap(self.goal_yaw - pose[2])
                if abs(yaw_error) > GOAL_YAW_TOL:
                    tw = Twist()
                    tw.angular.z = max(
                        -W_MAX, min(W_MAX, 1.8 * yaw_error))
                    self.pub_cmd.publish(tw)
                    self._last_command = (0.0, 0.0, tw.angular.z)
                    self._set_status(S_ALIGNING)
                    return
            self._stop()
            self.get_logger().info(
                "到达目标 (%.2f, %.2f), yaw=%.1f deg"
                % (self.goal[0], self.goal[1], math.degrees(pose[2])))
            self.goal = None
            self.goal_yaw = None
            self._set_status(S_ARRIVED)
            return

        # 动态控制优先于静态路径的卡死/偏离规则。绕行本来就可能暂时
        # 偏离全局路径，因此不能在 DWA 接管期间触发 Lazy Theta* 重规划。
        self.path_idx = _advance_path_index(
            self.path, self.path_idx, pose[0], pose[1])
        dynamic_command = self._dynamic_command(pose)
        if dynamic_command is not None:
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = dynamic_command
            self.pub_cmd.publish(tw)
            self._last_command = dynamic_command
            self.last_pose = pose
            return

        # 被堵判定
        if self._stuck_pos is None:
            self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()
        else:
            moved = math.hypot(pose[0] - self._stuck_pos[0],
                               pose[1] - self._stuck_pos[1])
            dt = (self.get_clock().now() - self._stuck_t0).nanoseconds / 1e9
            if (self._now_sec() >= self._dynamic_resume_until
                    and dt > STUCK_TIMEOUT and moved < STUCK_DIST):
                self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()
                self.get_logger().warn("疑似被堵, 重规划")
                self._set_status(S_STUCK)
                self._plan()
                if self.status == S_UNREACH:
                    return
            if moved > STUCK_DIST:
                self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()

        # 偏离重规划
        if (self._now_sec() >= self._dynamic_resume_until
                and self._path_deviation(pose) > DEVIATE_REPLAN):
            self.get_logger().warn("偏离路径 %.2f m, 重规划" % self._path_deviation(pose))
            self._plan()
            if self.status == S_UNREACH:
                return

        tx, ty = self.path[self.path_idx]

        # 到目标还差多少（用于减速）
        rem = math.hypot(self.path[-1][0] - pose[0],
                         self.path[-1][1] - pose[1])

        # 全向底盘：直接把速度指向前瞻点（可侧移），航向单独回正
        dx, dy = tx - pose[0], ty - pose[1]
        dist = max(math.hypot(dx, dy), 1e-3)
        want = math.atan2(dy, dx)
        err = _wrap(want - pose[2])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        uxb = (c * dx + s * dy) / dist
        uyb = (-s * dx + c * dy) / dist
        scale = V_MAX * min(1.0, rem / 0.4)
        scale = min(scale, _corner_speed_limit(
            self.path, self.path_idx, pose[0], pose[1]))
        wz = max(-W_MAX, min(W_MAX, 1.6 * err))

        tw = Twist()
        tw.linear.x = uxb * scale
        tw.linear.y = uyb * scale
        tw.angular.z = wz
        self.pub_cmd.publish(tw)
        self._last_command = (tw.linear.x, tw.linear.y, tw.angular.z)
        self.last_pose = pose

    def _path_deviation(self, pose):
        if len(self.path) < 2:
            return 0.0
        best = 1e18
        for k in range(self.path_idx, min(self.path_idx + 5, len(self.path))):
            x1, y1 = self.path[k - 1]
            x2, y2 = self.path[k]
            vx, vy = x2 - x1, y2 - y1
            wx, wy = pose[0] - x1, pose[1] - y1
            t = (wx * vx + wy * vy) / max(vx * vx + vy * vy, 1e-9)
            t = max(0.0, min(1.0, t))
            px, py = x1 + t * vx, y1 + t * vy
            best = min(best, math.hypot(pose[0] - px, pose[1] - py))
        return best

    def _stop(self):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = 0.0, 0.0, 0.0
        self.pub_cmd.publish(tw)
        self._last_command = (0.0, 0.0, 0.0)

    # ---------------- 输出 ----------------
    def _set_status(self, s):
        if s != self.status:
            self.status = s
            msg = String()
            msg.data = s
            self.pub_status.publish(msg)
            self.get_logger().info("状态 -> %s" % s)

    def _publish_path(self):
        if self.map is None:
            return
        p = Path()
        p.header.frame_id = "map"
        for x, y in self.path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_path.publish(p)

    def _publish_planning_map(self):
        if self.map is not None:
            self.pub_planning_map.publish(self.map)

    def _publish_height_cost_map(self):
        """Publish height soft cost and hard-height cells for RViz inspection."""
        if self.map is None or self.height_cost is None:
            return
        msg = OccupancyGrid()
        msg.header = self.map.header
        msg.header.frame_id = "map"
        msg.info = self.map.info
        values = np.full(self.grid_shape, -1, dtype=np.int8)
        finite = np.isfinite(self.height_clearance)
        values[~finite] = 0
        values[finite] = np.clip(np.rint(self.height_cost[finite] * 100.0), 0, 100).astype(np.int8)
        hard = finite & (self.height_clearance <= self.hard_height)
        values[hard] = 100
        msg.data = [int(v) for v in values.ravel()]
        self.pub_height_cost.publish(msg)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--use-saved", action="store_true",
                    help="只用保存的地图，不订阅在线 /map")
    ap.add_argument("--scene", default="ariac",
                    choices=["ariac"],
                    help="选择场景地图 (默认: ariac)")
    ap.add_argument("--lambda-geo", type=float, default=LAMBDA_GEO,
                    help="软安全代价权重；0 为原始最短路")
    ap.add_argument("--lambda-height", type=float, default=LAMBDA_HEIGHT,
                    help="高度软代价权重；0 关闭高度软代价")
    ap.add_argument("--robot-height", type=float, default=ROBOT_HEIGHT)
    ap.add_argument("--height-safety-margin", type=float,
                    default=HEIGHT_SAFETY_MARGIN)
    ap.add_argument("--preferred-height", type=float,
                    default=PREFERRED_HEIGHT)
    args = ap.parse_args()

    # 根据场景设置地图路径
    global SAVED_MAP, SAVED_YAML, SAVED_CLOUD
    scene_dir = os.path.join(HERE, "..", "..", "maps", args.scene)
    SAVED_MAP = os.path.join(scene_dir, "%s_map_3d.pgm" % args.scene)
    SAVED_YAML = os.path.join(scene_dir, "%s_map_3d.yaml" % args.scene)
    SAVED_CLOUD = os.path.join(scene_dir, "%s_map_3d_cloud.ply" % args.scene)

    rclpy.init()
    node = NavP2P(use_saved=args.use_saved, scene=args.scene,
                  lambda_geo=args.lambda_geo, lambda_height=args.lambda_height,
                  robot_height=args.robot_height,
                  height_safety_margin=args.height_safety_margin,
                  preferred_height=args.preferred_height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
