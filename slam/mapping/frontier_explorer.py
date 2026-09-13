#!/usr/bin/env python3.8
"""Frontier Exploration + 全局路径规划 + 局部避障：自主探索建图。

订阅:
  /rtabmap/grid_map  nav_msgs/OccupancyGrid   2D栅格地图（已知/未知/占据）
  /pointcloud        sensor_msgs/PointCloud2   3D 点云（用于局部避障）
  /odom              nav_msgs/Odometry         机器人位姿（备用）
  TF                  map -> odom -> base_footprint（主定位，与地图帧一致）

发布:
  /cmd_vel           geometry_msgs/Twist       速度指令
  /mapping_complete  std_msgs/Bool              建图完成事件（仅发布一次）
  /frontier_markers  visualization_msgs/MarkerArray   Frontier 可视化
  /explore_path      nav_msgs/Path             规划路径可视化

原理:
  1. 从 occupancy grid 中检测 frontier（已知free与unknown的边界）
  2. 对地图障碍做 EDT 膨胀，从机器人位置做 Dijkstra wavefront，
     只把【可达】的 frontier 作为候选目标（墙另一侧的目标被过滤掉）
  3. 用 A*/回溯生成可达路径，沿 waypoints 纯追踪跟随
  4. 复用 bridge 的 LocalAvoidance（BACKUP->TURN->CLEAR->IDLE 状态机）
     做局部安全层：急停/减速/脱困
  5. 进度式卡死检测：目标距离不再下降才判卡住；多次脱困无效则拉黑目标重选

用法:
  source /opt/ros/foxy/setup.bash
  python3.8 slam/mapping/frontier_explorer.py
"""
import math
import os
import sys

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy.parameter

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE_DIR = os.path.join(HERE, "..", "bridge")
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

from local_avoidance import (  # noqa: E402
    AvoidanceConfig,
    LocalAvoidance,
    build_range_rays,
)
from explore_planner import ExplorePlanner  # noqa: E402
from frontier_policy import (  # noqa: E402
    build_frontier_mask,
    filter_visited_frontiers,
    goal_has_frontier,
    prune_visits,
)

# === Frontier 检测参数 ===
# 货架外沿通常形成数百格的连续 frontier，而过道入口只有几十格。
# 面积只作为信息增益的次要因素，否则机器人会反复追逐外沿、错过入口。
FRONTIER_MIN_SIZE = 8         # 最小 frontier 聚类大小（栅格数）
FRONTIER_GOAL_BIAS = 0.25     # 目标选择：0=纯距离优先，1=纯大小优先
FRONTIER_RECHECK_SEC = 3.0   # frontier 重检测间隔（秒）
FRONTIER_REACHED_DIST = 0.8  # 认为到达 frontier 的距离阈值（米）
REACHED_NEAR_DIST = 1.5      # 正前方被挡时的"到达"放宽距离（米）
FRONTIER_NO_GOAL_RETRY = 5   # 连续几次找不到 frontier 才判定完成
GOAL_REVALIDATE_SEC = 1.0    # 导航中检查目标是否已被新地图覆盖
GOAL_FRONTIER_RADIUS = 0.75  # 目标附近仍有 frontier 才继续前往（米）
VISITED_COOLDOWN_SEC = 30.0  # 已扫描区域在此期间不再选为目标
VISITED_COOLDOWN_RADIUS = 2.0

# === 全局路径规划参数 ===
PLAN_INFLATE = 0.45          # 障碍膨胀半径（米），> 狗碰撞盒半对角线 0.55 的常用裕量
PLAN_WAYPOINT_SPACING = 0.4  # 路径点间距（米）
PLAN_REPLAN_SEC = 1.0        # 导航中重规划最短间隔（秒）
PLAN_DEVIATE_REPLAN = 0.6    # 偏离路径超过此值(米)触发重规划
PLAN_NEAR_GOAL_SNAP = 10     # 目标格被膨胀占用时，就近吸附搜索半径（栅格）
PLAN_MAX_GOAL_DIST = 25.0    # wavefront 距离超过此值视为不可达

# === 目标选择与脱困 ===
GOAL_BLACKLIST_COUNT = 3     # 记住最近失败的 N 个目标
GOAL_BLACKLIST_RADIUS = 2.0  # 拉黑半径（米）
STUCK_NO_PROGRESS_SEC = 4.0  # 到目标距离无下降的判定窗口（秒）
PROGRESS_DIST = 0.25         # 目标距离下降多少米才算有进展
RECOVERY_ATTEMPTS_MAX = 2    # 连续几次恢复仍无进展则放弃该目标

# === 导航指令参数 ===
NAV_SPEED_MAX = 0.30         # 最大前进速度（m/s）
NAV_TURN_GAIN = 2.5          # 转向增益
NAV_TURN_MAX = 1.0           # 最大角速度
LOOKAHEAD = 0.45             # 纯追踪前瞻距离（米）

# 地图定位的安全边界。map->odom 是 RTAB-Map 图优化输出，过期时继续把
# odom 位姿当成 map 位姿会让规划器驶向错误坐标；宁可短暂停车等待更新。
MAP_TF_LOOKUP_TIMEOUT = 0.05
MAP_TF_MAX_AGE = 5.0
LOCALIZATION_WARN_INTERVAL = 2.0

# === 局部避障参数（复用 bridge 的 LocalAvoidance）===
AVOID_EMERGENCY_DIST = 0.75  # 紧急避障触发水平净距（米）
AVOID_SLOW_DIST = 1.50       # 减速区开始距离（米）
AVOID_FRONT_AZ_DEG = 42.0    # 前方扇区半角（度）

# === 探索状态 ===
STATE_BOOTSTRAP = 0          # 启动阶段：原地旋转，让 rtabmap 积累关键帧
STATE_WAIT_MAP = 1            # 等待第一张地图
STATE_FIND_FRONTIER = 2      # 检测 frontier 并选目标
STATE_NAVIGATE = 3           # 沿规划路径导航到目标
STATE_ROTATE_SCAN = 4        # 到达后等待地图，必要时短角度补扫
STATE_DONE = 5               # 无 frontier 可探索，建图完成

STATE_NAMES = ["BOOTSTRAP", "WAIT_MAP", "FIND_FRONTIER", "NAVIGATE", "ROTATE_SCAN", "DONE"]

# 启动阶段参数
BOOTSTRAP_DURATION = 6.0      # 原地旋转+前进时长（秒），sim time 较慢不宜太长
SCAN_SETTLE_SEC = 1.0         # 到达后先停车，等待 RTAB-Map 消化最新点云
SCAN_MAP_WAIT_MAX_SEC = 2.5   # 地图繁忙时允许多等一帧，但不再固定转 12.6 秒
SUPPLEMENT_SCAN_ANGLE = math.pi / 2.0
SUPPLEMENT_SCAN_SPEED = 0.6   # frontier 仍存在时仅补扫 90 度

# lidar 相关（与 bridge 一致）
LIDAR_HEIGHT = 1.00           # lidar 安装高度（世界系）
NAV_Z_MIN = -0.85            # 点云 z 下限（lidar 局部系）
NAV_Z_MAX = 0.50             # 点云 z 上限
RAY_NUM = 360                # 局部避障用的方位角数
RANGE_MAX = 8.0              # 与雷达 RANGE_MAX 一致


def quat_to_yaw(q):
    """四元数转 yaw。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    """两角度差，归一化到 [-pi, pi]。"""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer",
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 "use_sim_time",
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 True)
                         ])
        self.get_logger().info("Frontier Explorer 启动 (use_sim_time=True)")

        # 状态
        self.state = STATE_BOOTSTRAP
        self._bootstrap_start = None  # 用 ROS 时间，在第一次 tick 时初始化
        self.goal = None          # (x, y) 世界坐标（map 系）
        self.pose = None          # (x, y, yaw) odom 系
        self._last_odom_time = None
        self._last_localization_warn = -1e9
        self.grid = None          # OccupancyGrid 消息
        self._last_frontier_time = 0.0
        self._rotate_start = None
        self._scan_start_revision = -1
        self._supplement_scan_start = None
        self._no_frontier_count = 0
        self._completion_published = False
        self._map_revision = 0
        self._goal_checked_revision = -1
        self._last_goal_revalidate = 0.0
        self._visited_goals = []  # [(x, y, ROS timestamp), ...]

        # 全局规划（可无 ROS 单测的 ExplorePlanner）
        self.planner = ExplorePlanner(
            inflate=PLAN_INFLATE,
            waypoint_spacing=PLAN_WAYPOINT_SPACING,
            max_goal_dist=PLAN_MAX_GOAL_DIST,
            near_goal_snap=PLAN_NEAR_GOAL_SNAP,
            blacklist_radius=GOAL_BLACKLIST_RADIUS,
        )
        self.path = []            # 世界系 waypoints [(x,y), ...]
        self.path_idx = 0
        self._last_cost_rebuild = 0.0
        self._last_dijkstra_time = 0.0
        self._failed_goals = []   # [(x,y), ...]

        # 进度式卡死
        self._best_goal_dist = None
        self._last_goal_progress_t = None
        self._recovery_attempts = 0
        self._force_recovery = False

        # 局部避障（复用 bridge 的 LocalAvoidance）
        self._avoider = LocalAvoidance(AvoidanceConfig(
            range_min=0.12,
            range_max=RANGE_MAX,
            emergency_dist=AVOID_EMERGENCY_DIST,
            slow_dist=AVOID_SLOW_DIST,
            front_half_angle_deg=AVOID_FRONT_AZ_DEG,
            max_turn_rate=NAV_TURN_MAX,
        ))
        self.rays = np.full(RAY_NUM, RANGE_MAX, dtype=np.float64)
        az = np.linspace(-np.pi, np.pi, RAY_NUM, endpoint=False)
        self._ray_dirs = np.zeros((1, RAY_NUM, 3), dtype=np.float64)
        self._ray_dirs[0, :, 0] = np.cos(az)
        self._ray_dirs[0, :, 1] = np.sin(az)

        # TF（主定位，map 系）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅
        map_qos_transient = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        map_qos_volatile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE)
        for topic in ["/rtabmap/grid_map", "/rtabmap/map", "/map"]:
            self.create_subscription(OccupancyGrid, topic,
                                     self._on_map, map_qos_transient)
            self.create_subscription(OccupancyGrid, topic,
                                     self._on_map, map_qos_volatile)

        pc_qos = QoSProfile(depth=2,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, "/pointcloud",
                                 self._on_pointcloud, pc_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # 发布
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        completion_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_complete = self.create_publisher(
            Bool, "/mapping_complete", completion_qos)
        self.pub_markers = self.create_publisher(MarkerArray, "/frontier_markers", 5)
        self.pub_path = self.create_publisher(Path, "/explore_path", 5)

        # 主循环 10Hz
        self.create_timer(0.1, self._tick)

    # ==================== 回调 ====================

    def _now(self):
        """获取当前 ROS 时间（秒），兼容 use_sim_time。"""
        t = self.get_clock().now().nanoseconds
        return t / 1e9

    def _on_map(self, msg):
        self.grid = msg
        self._map_revision += 1
        now = self._now()
        if now - self._last_cost_rebuild >= 0.5:
            self._last_cost_rebuild = now
            self._rebuild_cost()
        if self.state == STATE_WAIT_MAP:
            self.state = STATE_FIND_FRONTIER
            self.get_logger().info("收到地图，开始探索")

    def _on_odom(self, msg):
        p = msg.pose.pose
        yaw = quat_to_yaw(p.orientation)
        self.pose = (p.position.x, p.position.y, yaw)
        self._last_odom_time = self._now()

    def _on_pointcloud(self, msg):
        """解析点云，构建每方位角的最小水平净距（供 LocalAvoidance 使用）。"""
        if msg.width == 0:
            self.rays[:] = RANGE_MAX
            return
        # 解析 PointCloud2 (假设 float32 xyzi, point_step=16)
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)[:, :3]
        self.rays = build_range_rays(
            pts, num_rays=RAY_NUM, range_max=RANGE_MAX,
            z_min=NAV_Z_MIN, z_max=NAV_Z_MAX)

    def _get_pose_map(self):
        """将最新 odom 位姿用 map->odom 修正转换到 map 系。

        不能在 TF 失败时直接返回裸 odom：地图、目标和路径都在 map 系，
        坐标系混用会在图优化或处理延迟时产生错误控制。
        """
        if self.pose is None or self._last_odom_time is None:
            return None
        if self._now() - self._last_odom_time > MAP_TF_MAX_AGE:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "odom", Time(),
                Duration(seconds=MAP_TF_LOOKUP_TIMEOUT))
            stamp = float(t.header.stamp.sec) + float(t.header.stamp.nanosec) / 1e9
            if stamp > 0.0 and self._now() - stamp > MAP_TF_MAX_AGE:
                return None

            # map_T_odom * odom_T_base, reduced to SE(2).
            tf_x = t.transform.translation.x
            tf_y = t.transform.translation.y
            tf_yaw = quat_to_yaw(t.transform.rotation)
            odom_x, odom_y, odom_yaw = self.pose
            c, s = math.cos(tf_yaw), math.sin(tf_yaw)
            x = tf_x + c * odom_x - s * odom_y
            y = tf_y + s * odom_x + c * odom_y
            yaw = math.atan2(math.sin(tf_yaw + odom_yaw),
                             math.cos(tf_yaw + odom_yaw))
            return (x, y, yaw)
        except Exception:
            return None

    def _warn_localization_unavailable(self):
        now = self._now()
        if now - self._last_localization_warn >= LOCALIZATION_WARN_INTERVAL:
            self._last_localization_warn = now
            self.get_logger().warn(
                "map->odom 定位暂不可用或已过期，已停车等待 RTAB-Map 更新")

    # ==================== 全局路径规划（委托给 ExplorePlanner）====================

    def _rebuild_cost(self):
        """占据栅格 -> EDT 距离场 -> 膨胀后代价栅格（unknown 视为不可走）。"""
        g = self.grid
        if g is None:
            return
        occ = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        self.planner.update_map(occ, g.info.resolution,
                                (g.info.origin.position.x, g.info.origin.position.y))

    def _dijkstra(self):
        """从机器人位置刷新可达距离。"""
        pose = self._get_pose_map()
        if pose is None:
            return
        self.planner.compute_dijkstra((pose[0], pose[1]))

    def _plan_goal(self, gx_world, gy_world):
        """生成到目标的可达路径（世界系 waypoints），不可达返回 None。"""
        return self.planner.plan_goal((gx_world, gy_world))

    def _publish_path(self):
        if not self.path:
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

    # ==================== Frontier 检测 ====================

    def _detect_frontiers(self):
        """检测地图中所有 frontier 并聚类。返回 [(centroid_x, centroid_y, size), ...]"""
        if self.grid is None:
            return []

        g = self.grid
        w, h = g.info.width, g.info.height
        res = g.info.resolution
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y

        data = np.array(g.data, dtype=np.int8).reshape(h, w)
        frontier_mask = build_frontier_mask(data)

        labeled, num_features = ndimage.label(frontier_mask)
        if num_features == 0:
            return []

        frontiers = []
        for i in range(1, num_features + 1):
            cells = np.argwhere(labeled == i)
            if len(cells) < FRONTIER_MIN_SIZE:
                continue
            cy = cells[:, 0].mean() * res + oy
            cx = cells[:, 1].mean() * res + ox
            frontiers.append((cx, cy, len(cells)))

        return frontiers

    def _select_frontier(self, frontiers):
        """选最佳 frontier：只选可达的，综合考虑距离和大小。"""
        # 离线 replay 用轻量对象直接借用本方法，因此显式调用类实现，
        # 不要求 replay 对象另外声明同名包装方法。
        frontiers = FrontierExplorer._filter_recently_visited(self, frontiers)
        if not frontiers:
            return None
        if self.planner.wavefront is None:
            self._dijkstra()
        if self.planner.wavefront is None:
            return None
        return self.planner.select_frontier(
            frontiers, goal_bias=FRONTIER_GOAL_BIAS,
            blacklist=self._failed_goals)

    def _filter_recently_visited(self, frontiers):
        """过滤近期已经扫描过的区域，离线回放对象没有记录时保持兼容。"""
        visits = getattr(self, "_visited_goals", [])
        if not visits:
            return list(frontiers)
        now = self._now()
        self._visited_goals = prune_visits(
            visits, now, VISITED_COOLDOWN_SEC)
        return filter_visited_frontiers(
            frontiers, self._visited_goals, VISITED_COOLDOWN_RADIUS)

    def _remember_visited(self, goal):
        now = self._now()
        self._visited_goals = prune_visits(
            self._visited_goals, now, VISITED_COOLDOWN_SEC)
        self._visited_goals.append((goal[0], goal[1], now))

    def _goal_still_frontier(self):
        if self.grid is None or self.goal is None:
            return False
        g = self.grid
        data = np.asarray(g.data, dtype=np.int8).reshape(
            g.info.height, g.info.width)
        return goal_has_frontier(
            data, g.info.resolution,
            (g.info.origin.position.x, g.info.origin.position.y),
            self.goal, GOAL_FRONTIER_RADIUS)

    def _blacklist_goal(self, x, y):
        self._failed_goals.append((x, y))
        if len(self._failed_goals) > GOAL_BLACKLIST_COUNT:
            self._failed_goals.pop(0)

    # ==================== 路径跟随 ====================

    def _lookahead_waypoint(self, pose):
        """沿路径选前瞻点，返回世界系 (x, y)；路径耗尽返回 None。"""
        if not self.path:
            return None
        # 推进已到达的路径点
        i = self.path_idx
        while (i < len(self.path) - 1
               and math.hypot(self.path[i][0] - pose[0],
                              self.path[i][1] - pose[1]) < 0.35):
            i += 1
        self.path_idx = i
        if i >= len(self.path):
            return None
        # 纯追踪：选 LOOKAHEAD 内最远的路径点
        best = i
        for k in range(i, len(self.path)):
            if math.hypot(self.path[k][0] - pose[0],
                          self.path[k][1] - pose[1]) <= LOOKAHEAD:
                best = k
            else:
                break
        return self.path[best]

    def _path_deviation(self, pose):
        """到路径最近线段的距离。"""
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

    # ==================== 卡住检测 ====================

    def _path_remaining(self, pose):
        """从机器人当前位置沿路径到终点的剩余长度（米）。"""
        if not self.path:
            return math.inf
        if len(self.path) < 2:
            return math.hypot(self.path[0][0] - pose[0],
                              self.path[0][1] - pose[1])
        i = min(max(self.path_idx, 0), len(self.path) - 1)
        rem = math.hypot(self.path[i][0] - pose[0],
                         self.path[i][1] - pose[1])
        for k in range(i, len(self.path) - 1):
            rem += math.hypot(self.path[k + 1][0] - self.path[k][0],
                              self.path[k + 1][1] - self.path[k][1])
        return rem

    def _goal_progress_stuck(self, value):
        """进度值（沿路径剩余长度）是否已持续 STUCK_NO_PROGRESS_SEC 无下降。

        用沿路径剩余长度而不是直线距离：绕障路径会让直线距离长期不动，
        用它判卡住会误报；沿路径推进才是真实进展。
        """
        now = self._now()
        if self._best_goal_dist is None:
            self._best_goal_dist = value
            self._last_goal_progress_t = now
            return False
        if value < self._best_goal_dist - PROGRESS_DIST:
            self._best_goal_dist = value
            self._last_goal_progress_t = now
            self._recovery_attempts = 0
            return False
        if now - self._last_goal_progress_t >= STUCK_NO_PROGRESS_SEC:
            self._last_goal_progress_t = now
            self._best_goal_dist = value
            return True
        return False

    def _reset_goal_progress(self, keep_attempts=False):
        self._best_goal_dist = None
        self._last_goal_progress_t = self._now()
        self._force_recovery = False
        if not keep_attempts:
            self._recovery_attempts = 0

    # ==================== 主循环 ====================

    def _finish_exploration(self):
        if self._completion_published:
            return
        self.state = STATE_DONE
        self._completion_published = True
        msg = Bool()
        msg.data = True
        self.pub_complete.publish(msg)
        self.get_logger().info("无可达 frontier，建图完成，已通知桥接冻结地图！")

    def _tick(self):
        cmd = Twist()

        if self.state == STATE_BOOTSTRAP:
            if self._bootstrap_start is None:
                self._bootstrap_start = self._now()
            elapsed = self._now() - self._bootstrap_start
            if elapsed < BOOTSTRAP_DURATION:
                cmd.angular.z = 0.45
                cmd.linear.x = 0.08
                if int(elapsed) % 5 == 0 and int(elapsed * 10) % 10 == 0:
                    self.get_logger().info(
                        f"[启动] 原地扫描中... {elapsed:.0f}/{BOOTSTRAP_DURATION:.0f}s")
            else:
                self.get_logger().info("启动扫描完成，等待 rtabmap 发布地图...")
                if self.grid is not None:
                    self.state = STATE_FIND_FRONTIER
                else:
                    self.state = STATE_WAIT_MAP

        elif self.state == STATE_WAIT_MAP:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.3

        elif self.state == STATE_FIND_FRONTIER:
            if self._get_pose_map() is None:
                self._warn_localization_unavailable()
                self.pub_cmd.publish(cmd)
                return
            now = self._now()
            if now - self._last_frontier_time < FRONTIER_RECHECK_SEC:
                return
            self._last_frontier_time = now

            self._dijkstra()  # 刷新可达距离
            frontiers = self._detect_frontiers()
            self._publish_frontier_markers(frontiers)

            if self.grid is not None:
                g = self.grid
                data = np.array(g.data, dtype=np.int8)
                n_free = int(np.sum((data >= 0) & (data < 50)))
                n_unknown = int(np.sum(data == -1))
                n_occupied = int(np.sum(data >= 50))
                total = len(data)
                self.get_logger().info(
                    f"[地图状态] {g.info.width}x{g.info.height} res={g.info.resolution:.2f}, "
                    f"free={n_free}({100*n_free//max(total,1)}%), "
                    f"unknown={n_unknown}({100*n_unknown//max(total,1)}%), "
                    f"occupied={n_occupied}, frontiers={len(frontiers)}")

            best = self._select_frontier(frontiers)
            if best is None:
                eligible = self._filter_recently_visited(frontiers)
                if frontiers and not eligible:
                    self._no_frontier_count = 0
                    self.get_logger().info(
                        "附近 frontier 均处于访问冷却期，继续缓慢观察地图...")
                    cmd.linear.x = 0.10
                    cmd.angular.z = 0.20
                    self.pub_cmd.publish(cmd)
                    return
                self._no_frontier_count += 1
                if self._no_frontier_count >= FRONTIER_NO_GOAL_RETRY:
                    self._finish_exploration()
                    self.pub_cmd.publish(cmd)
                    return
                self.get_logger().info(
                    f"暂无有效 frontier ({self._no_frontier_count}/"
                    f"{FRONTIER_NO_GOAL_RETRY})，随机移动...")
                cmd.linear.x = 0.2
                cmd.angular.z = 0.4
            else:
                self._no_frontier_count = 0
                self.goal = (best[0], best[1])
                self.path = []
                self.path_idx = 0
                self._reset_goal_progress()
                self._avoider.reset()
                self._goal_checked_revision = self._map_revision
                self._last_goal_revalidate = now
                self.state = STATE_NAVIGATE
                self.get_logger().info(
                    f"目标 frontier: ({self.goal[0]:.1f}, {self.goal[1]:.1f}), "
                    f"距离 {best[2]:.1f}m，共 {len(frontiers)} 个候选")

        elif self.state == STATE_NAVIGATE:
            pose = self._get_pose_map()
            if pose is None:
                self._warn_localization_unavailable()
                self.pub_cmd.publish(cmd)
                return
            if self.goal is None:
                self.pub_cmd.publish(cmd)
                return

            now = self._now()
            scan = self._avoider.analyze(
                self.rays[None, :], self._ray_dirs, LIDAR_HEIGHT)

            # 目标可能在行驶途中已被 360 度雷达覆盖。只在收到新地图后检查，
            # 避免对同一帧反复计算，也避免继续追逐已经消失的旧 frontier。
            if (now - self._last_goal_revalidate >= GOAL_REVALIDATE_SEC
                    and self._map_revision != self._goal_checked_revision):
                self._last_goal_revalidate = now
                self._goal_checked_revision = self._map_revision
                if not self._goal_still_frontier():
                    old_goal = self.goal
                    self.get_logger().info(
                        f"目标 frontier ({old_goal[0]:.1f}, {old_goal[1]:.1f}) "
                        "已被雷达覆盖，取消旧目标并重新选择")
                    self._remember_visited(old_goal)
                    self._clear_goal()
                    self.state = STATE_FIND_FRONTIER
                    self._last_frontier_time = 0.0
                    self.pub_cmd.publish(cmd)
                    return

            # 到达判定
            dist = math.hypot(self.goal[0] - pose[0],
                              self.goal[1] - pose[1])
            # frontier 贴着墙/未知边界，避障会让机器人停在安全净距外，
            # 到不了 0.8m 的阈值；此时若已经足够近且正前方被挡，视为已到达。
            reached = (dist < FRONTIER_REACHED_DIST
                       or (dist < REACHED_NEAR_DIST
                           and scan.front < AVOID_EMERGENCY_DIST))
            if reached:
                reached_goal = self.goal
                self.get_logger().info(
                    f"到达 frontier ({reached_goal[0]:.1f}, {reached_goal[1]:.1f}), "
                    f"距离 {dist:.2f}m")
                self._remember_visited(reached_goal)
                self.state = STATE_ROTATE_SCAN
                self._rotate_start = now
                self._scan_start_revision = self._map_revision
                self._supplement_scan_start = None
                self._avoider.reset()
                self.pub_cmd.publish(cmd)
                return

            # 进度式卡死：用沿路径剩余长度（绕障不会误报）
            prev_phase = self._avoider.phase
            if (not self._avoider.active
                    and self.path
                    and self._goal_progress_stuck(self._path_remaining(pose))):
                if self._recovery_attempts >= RECOVERY_ATTEMPTS_MAX:
                    self.get_logger().warn(
                        f"目标 ({self.goal[0]:.1f}, {self.goal[1]:.1f}) "
                        f"连续 {self._recovery_attempts} 次恢复仍无进展，拉黑重选")
                    self._blacklist_goal(self.goal[0], self.goal[1])
                    self._clear_goal()
                    self.state = STATE_FIND_FRONTIER
                    self._last_frontier_time = 0.0
                    self.pub_cmd.publish(cmd)
                    return
                self._recovery_attempts += 1
                self.get_logger().warn(
                    f"沿路径剩余 {self._path_remaining(pose):.2f}m 已有 "
                    f"{STUCK_NO_PROGRESS_SEC:.0f}s 未下降，强制脱困 "
                    f"{self._recovery_attempts}/{RECOVERY_ATTEMPTS_MAX} "
                    f"@({pose[0]:.1f},{pose[1]:.1f}) "
                    f"front={scan.front:.2f} path_pts={len(self.path)}")
                self._force_recovery = True
                self.path = []  # 恢复完成后基于新地图重新规划
            if self._avoider.phase != prev_phase:
                # 脱困结束回到正常控制，进度基线重置（路径可能已被清空）。
                # 不重置 recovery_attempts：只有真正推进才会清零，否则持续脱困后放弃。
                if self._avoider.phase == LocalAvoidance.IDLE:
                    self._reset_goal_progress(keep_attempts=True)

            # 规划/重规划路径
            if not self.path:
                if now - self._last_dijkstra_time >= PLAN_REPLAN_SEC:
                    self._last_dijkstra_time = now
                    self._dijkstra()
                path = self._plan_goal(self.goal[0], self.goal[1])
                if path is None:
                    self.get_logger().warn(
                        f"目标 ({self.goal[0]:.1f}, {self.goal[1]:.1f}) 规划失败，拉黑重选")
                    self._blacklist_goal(self.goal[0], self.goal[1])
                    self._clear_goal()
                    self.state = STATE_FIND_FRONTIER
                    self._last_frontier_time = 0.0
                    self.pub_cmd.publish(cmd)
                    return
                self.path = path
                self.path_idx = 0
                self._reset_goal_progress(keep_attempts=True)
                self._publish_path()

            look = self._lookahead_waypoint(pose)
            if look is None:
                self.path = []  # 路径耗尽，下 tick 重规划
                self.pub_cmd.publish(cmd)
                return

            # 期望指令（纯追踪）
            want = math.atan2(look[1] - pose[1], look[0] - pose[0])
            err = angle_diff(want, pose[2])
            wz = np.clip(NAV_TURN_GAIN * err, -NAV_TURN_MAX, NAV_TURN_MAX)
            vx = NAV_SPEED_MAX * max(0.25, 1.0 - abs(err) / math.pi)
            if abs(err) > 0.5:
                vx = 0.0

            # 局部避障（复用 bridge LocalAvoidance：急停/减速/脱困）
            preferred = 1.0 if err >= 0.0 else -1.0
            force = self._force_recovery
            self._force_recovery = False
            prev_phase = self._avoider.phase
            recovery = self._avoider.recovery_command(
                now, scan, preferred=preferred, force=force)
            if self._avoider.phase != prev_phase:
                self.get_logger().info(
                    f"[避障] {prev_phase}->{self._avoider.phase} "
                    f"前方净距 {scan.front:.2f}m")
            if recovery is not None:
                cmd.linear.x = recovery[0]
                cmd.angular.z = recovery[2]
                self.pub_cmd.publish(cmd)
                return

            caution = self._avoider.caution_command(
                now, scan, goal_error=err, cruise_speed=NAV_SPEED_MAX)
            if caution is not None:
                cmd.linear.x = caution[0]
                cmd.angular.z = caution[2]
                self.pub_cmd.publish(cmd)
                return

            cmd.linear.x = vx
            cmd.angular.z = wz

            # 偏离路径过远 -> 重规划
            if self._path_deviation(pose) > PLAN_DEVIATE_REPLAN:
                self.path = []

        elif self.state == STATE_ROTATE_SCAN:
            if self._get_pose_map() is None:
                self._warn_localization_unavailable()
                self.pub_cmd.publish(cmd)
                return
            if self._rotate_start is None:
                self._rotate_start = self._now()
                self._scan_start_revision = self._map_revision
            elapsed = self._now() - self._rotate_start
            map_not_updated = self._map_revision == self._scan_start_revision
            if (elapsed < SCAN_SETTLE_SEC
                    or (map_not_updated and elapsed < SCAN_MAP_WAIT_MAX_SEC)):
                # 雷达水平面本身是 360 度；先停车让 RTAB-Map 更新，不重复转整圈。
                pass
            elif self._supplement_scan_start is None:
                if not self._goal_still_frontier():
                    self.get_logger().info(
                        "停车等待后 frontier 已消失，跳过原地补扫")
                    self._finish_scan()
                else:
                    self._supplement_scan_start = self._now()
                    self.get_logger().info(
                        "frontier 仍存在，执行 90 度短补扫")
            elif (self._now() - self._supplement_scan_start
                  < SUPPLEMENT_SCAN_ANGLE / SUPPLEMENT_SCAN_SPEED):
                # 短补扫期间保留局部避障，避免贴墙转动时碰撞。
                now = self._now()
                scan = self._avoider.analyze(
                    self.rays[None, :], self._ray_dirs, LIDAR_HEIGHT)
                recovery = self._avoider.recovery_command(
                    now, scan, preferred=1.0)
                if recovery is not None:
                    cmd.linear.x = recovery[0]
                    cmd.angular.z = recovery[2]
                else:
                    cmd.angular.z = SUPPLEMENT_SCAN_SPEED
            else:
                self.get_logger().info("短补扫完成，寻找下一个 frontier")
                self._finish_scan()

        elif self.state == STATE_DONE:
            self.get_logger().info("探索完成，所有区域已覆盖", once=True)

        self.pub_cmd.publish(cmd)

    def _finish_scan(self):
        self._clear_goal()
        self.state = STATE_FIND_FRONTIER
        self._rotate_start = None
        self._scan_start_revision = -1
        self._supplement_scan_start = None
        self._last_frontier_time = 0.0

    def _clear_goal(self):
        self.goal = None
        self.path = []
        self.path_idx = 0
        self._best_goal_dist = None
        self._last_goal_progress_t = None
        self._recovery_attempts = 0
        self._avoider.reset()

    # ==================== 可视化 ====================

    def _publish_frontier_markers(self, frontiers):
        """发布 frontier 可视化 markers。"""
        ma = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, (fx, fy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "frontiers"
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            scale = min(0.8, 0.1 + size * 0.01)
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.5
            m.color.a = 0.7
            m.lifetime.sec = 5
            ma.markers.append(m)

        if self.goal is not None:
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "goal"
            m.id = 0
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = self.goal[0]
            m.pose.position.y = self.goal[1]
            m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.6
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime.sec = 5
            ma.markers.append(m)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.pub_cmd.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
