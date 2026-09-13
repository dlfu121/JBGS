#!/usr/bin/env bash
# 一键起 3D 激光 SLAM：MuJoCo 3D 雷达 + rtabmap_ros + rviz2 显示。
#
# 使用 rtabmap_ros 做 ICP-based 3D SLAM，输出 3D 点云地图。
#
# 用法:
#   ./slam/run_slam_3d.sh              # 自动巡视 + rtabmap + rviz
#   ./slam/run_slam_3d.sh --view       # 同时开 MuJoCo 查看器
#   ./slam/run_slam_3d.sh --explore    # Frontier 自主探索建图（无需航点）
#   ./slam/run_slam_3d.sh --fast       # 2 倍仿真速度，缩短实验墙钟时间
#   ./slam/run_slam_3d.sh --no-rviz    # 不开 rviz
#   ./slam/run_slam_3d.sh --fresh      # 清理遗留进程后启动
#   ./slam/run_slam_3d.sh --scene ariac      # 用 ARIAC 场景（默认）
#   ./slam/run_slam_3d.sh --scene warehouse  # 用仓库场景
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi

# 清除 ROS1 (noetic) 残留环境变量，避免干扰 ROS2 DDS 通信
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO

# 关键：从 PYTHONPATH / LD_LIBRARY_PATH / CMAKE_PREFIX_PATH 中移除 ROS1/noetic 路径，
# 否则 python3.8 会加载 ROS1 的 sensor_msgs（缺少 _TYPE_SUPPORT），
# 或链接到 noetic 的 libtf2 导致 ABI 符号冲突。
# 注意：grep -v 在无匹配行时返回 1，用 { grep ... || true; } 防止 pipefail 中断。
PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | { grep -v -E '/opt/ros/noetic|catkin|haption_ws' || true; } | tr '\n' ':' | sed 's/:$//')"
export PYTHONPATH
LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | { grep -v -E '/opt/ros/noetic|catkin|haption_ws' || true; } | tr '\n' ':' | sed 's/:$//')"
export LD_LIBRARY_PATH
CMAKE_PREFIX_PATH="$(echo "${CMAKE_PREFIX_PATH:-}" | tr ':' '\n' | { grep -v -E '/opt/ros/noetic|catkin|haption_ws' || true; } | tr '\n' ':' | sed 's/:$//')"
export CMAKE_PREFIX_PATH
PKG_CONFIG_PATH="$(echo "${PKG_CONFIG_PATH:-}" | tr ':' '\n' | { grep -v -E '/opt/ros/noetic|catkin|haption_ws' || true; } | tr '\n' ':' | sed 's/:$//')"
export PKG_CONFIG_PATH

# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

if ! ros2 pkg list 2>/dev/null | grep -qx rtabmap_slam; then
  cat >&2 <<'EOF'
没装 rtabmap_ros。需要（要 sudo 密码，本脚本不代劳）：
    sudo apt update && sudo apt install -y ros-foxy-rtabmap-ros
若 apt 源中无此包，需从源码编译：
    cd ~/colcon_ws/src
    git clone -b foxy-devel https://github.com/introlab/rtabmap_ros.git
    git clone -b foxy-devel https://github.com/introlab/rtabmap.git
    cd ~/colcon_ws && colcon build --symlink-install
装完再跑本脚本。
EOF
  exit 1
fi

if [ ! -f model/robot/robot_template_3d_py38.xml ]; then
  echo "缺 model/robot/robot_template_3d_py38.xml，请先准备 3D 机器人模型" >&2
  exit 1
fi

BRIDGE_ARGS=()
WANT_PATROL=1
WANT_RVIZ=1
WANT_VIEW=0
FRESH=0
SCENE="ariac"
EXPLORE=0
for a in "$@"; do
  case "$a" in
    --view)     WANT_VIEW=1; BRIDGE_ARGS+=(--view) ;;
    --no-rviz)  WANT_RVIZ=0 ;;
    --fresh)    FRESH=1 ;;
    --teleop)   WANT_PATROL=0 ;;
    --explore)  EXPLORE=1; WANT_PATROL=0 ;;
    --fast)     BRIDGE_ARGS+=(--sim-speed 2.0) ;;
    --scene)    : ;;  # 下面处理
    ariac|warehouse) SCENE="$a" ;;
    *)          BRIDGE_ARGS+=("$a") ;;
  esac
done
# 只有非 explore/teleop 模式才加 --patrol
if [ "$WANT_PATROL" = 1 ]; then
  BRIDGE_ARGS+=(--patrol)
fi
# 处理 --scene xxx 形式
for ((i=1; i<=$#; i++)); do
  if [[ "${!i}" == "--scene" ]]; then
    j=$((i+1)); SCENE="${!j}"
  fi
done

if [ "$FRESH" = 1 ]; then
  echo "[fresh] 清理遗留的 3D 桥接/rtabmap/RViz 进程 ..."
  # 使用 -9 强制终止，避免僵尸进程导致 pkill 挂起
  pkill -9 -u "$USER" -f "bridge_warehouse.py" 2>/dev/null || true
  pkill -9 -u "$USER" -f "bridge_ariac.py" 2>/dev/null || true
  pkill -9 -u "$USER" -f "frontier_explorer.py" 2>/dev/null || true
  pkill -9 -u "$USER" -f "cloud_map_height_filter.py" 2>/dev/null || true
  pkill -9 -u "$USER" -f "rtabmap_3d.launch.py" 2>/dev/null || true
  pkill -9 -u "$USER" -f "rtabmap --delete_db_on_start" 2>/dev/null || true
  pkill -9 -u "$USER" -f "rtabmap_slam/rtabmap" 2>/dev/null || true
  pkill -9 -u "$USER" -f "rviz2.*slam_3d.rviz" 2>/dev/null || true
  # 额外清理可能遗留的 point_cloud_assembler
  pkill -9 -u "$USER" -f "point_cloud_assembler" 2>/dev/null || true
  # 清理可能挂死的 ros2 daemon，避免后续 ros2 命令卡住
  pkill -9 -u "$USER" -f "_ros2_daemon" 2>/dev/null || true
  sleep 2
  # 重启 daemon 确保 ros2 CLI 能正常工作
  ros2 daemon start 2>/dev/null || true
  sleep 1
fi

EXISTING="$(ros2 node list 2>/dev/null | grep -E '^/mujoco_slam_bridge_3d$' || true)"
if [ -n "$EXISTING" ]; then
  cat >&2 <<EOF
检测到 3D 桥接节点还在运行：$EXISTING
请先 Ctrl-C 上次运行，或使用 --fresh 参数。
EOF
  exit 1
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 根据场景选择桥接脚本
if [ "$SCENE" = "ariac" ]; then
  BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"
  python3.8 model/scenes/build_ariac_compat_meshes.py --check >/dev/null ||
    python3.8 model/scenes/build_ariac_compat_meshes.py
  python3.8 model/robot/gen_ariac_robot.py \
    --start-x 4.0 --start-y 4.6 --start-z -0.08
elif [ "$SCENE" = "warehouse" ]; then
  BRIDGE_SCRIPT="slam/bridge/bridge_warehouse.py"
else
  echo "未知场景: $SCENE（可选: ariac, warehouse）" >&2; exit 1
fi

echo "[1/4] 起 MuJoCo 3D 点云桥接 (场景: $SCENE) ..."
python3.8 "$BRIDGE_SCRIPT" "${BRIDGE_ARGS[@]}" &
BRIDGE_PID="$!"
PIDS+=("$BRIDGE_PID")
sleep 5

# Viewer/GLFW can terminate the bridge immediately when DISPLAY or OpenGL is
# unavailable.  Never continue with an apparently healthy RTAB-Map that has no
# point-cloud producer.  For --view, retry the same bridge headlessly; without
# --view, fail fast because there is no recoverable GUI-specific cause.
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  wait "$BRIDGE_PID" 2>/dev/null || true
  if [ "$WANT_VIEW" = 1 ]; then
    echo "  ★ MuJoCo 查看器启动失败，自动回退到无界面建图模式" >&2
    HEADLESS_ARGS=()
    for arg in "${BRIDGE_ARGS[@]}"; do
      [ "$arg" = "--view" ] || HEADLESS_ARGS+=("$arg")
    done
    python3.8 "$BRIDGE_SCRIPT" "${HEADLESS_ARGS[@]}" &
    BRIDGE_PID="$!"
    PIDS+=("$BRIDGE_PID")
    sleep 5
  fi
fi
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "MuJoCo 3D 点云桥接启动失败；停止启动 RTAB-Map。" >&2
  exit 1
fi

echo "[2/4] 起 MapCloud 地面点过滤 ..."
python3.8 "$HERE/slam/cloud_map_height_filter.py" --ros-args \
  -p min_height:=0.18 &
PIDS+=("$!")
sleep 1

echo "[3/4] 起 rtabmap 3D SLAM ..."
ros2 launch "$HERE/slam/rtabmap_3d.launch.py" use_sim_time:=true &
PIDS+=("$!")
sleep 4

if [ "$WANT_RVIZ" = 1 ]; then
  echo "[4/4] 起 RViz (slam/slam_3d.rviz) ..."
  rviz2 -d "$HERE/slam/slam_3d.rviz" --ros-args -p use_sim_time:=true &
  PIDS+=("$!")
fi

if [ "$EXPLORE" = 1 ]; then
  echo "[4/4] 起 Frontier Explorer（自主探索建图）..."
  sleep 3  # 等待 rtabmap 开始发布地图
  python3.8 "$HERE/slam/frontier_explorer.py" &
  PIDS+=("$!")
fi

cat <<'EOF'

3D SLAM 已启动。常用命令（另开终端，先 source /opt/ros/foxy/setup.bash）：
  ros2 topic hz /pointcloud                    # 确认点云 10 Hz
  ros2 topic hz /cloud_map                     # 3D 地图更新频率

保存地图（建图完成后执行）：
  # 导出 3D 点云地图为 PCD 文件（可直接用于路径规划）
  ros2 service call /publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
  ros2 run pcl_ros pointcloud_to_pcd --ros-args -r input:=/cloud_map

  # 导出 2D 投影栅格（nav2 可用）
  ros2 run nav2_map_server map_saver_cli -f ariac_map_3d --ros-args -p use_sim_time:=true

  # 保存 rtabmap 数据库（包含完整位姿图，可离线重处理）
  # 数据库自动存于 ~/.ros/rtabmap.db，Ctrl-C 退出时自动保存

Ctrl-C 结束全部进程。
EOF
wait
