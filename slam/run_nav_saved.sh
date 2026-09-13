#!/usr/bin/env bash
# 方案 B：用保存的地图 + 仿真环境做路径规划和导航。
# 不需要实时 SLAM 建图，直接加载已保存的栅格地图。
#
# 启动内容：
#   1. slam_bridge.py  — MuJoCo 仿真 + TF + /clock + 实时点云
#   2. nav_p2p.py --use-saved  — 保存地图全局规划 + 条件式动态避障
#   3. RViz2  — 可视化地图和路径，点击目标点导航
#
# 用法:
#   ./slam/run_nav_saved.sh              # ARIAC, headless
#   ./slam/run_nav_saved.sh --view       # 同时开 MuJoCo 查看器
#   ./slam/run_nav_saved.sh --scene ariac --task inspect --view
#   ./slam/run_nav_saved.sh --scene ariac --task vla --view
#   ./slam/run_nav_saved.sh --dynamic-person  # 动态行人测试（ARIAC/warehouse）
#
# 在 RViz2 中：
#   - 工具栏点 "2D Goal Pose"，在地图上点击目标位置 → 自动规划并导航
#   - /nav_path 显示规划路径
#   - /nav_status 显示导航状态
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# 清除 ROS1 残留
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
source /opt/ros/foxy/setup.bash
# Keep ROS 2 logs writable on graphical machines whose home directory may be
# mounted read-only.  The VLA publisher uses the same per-task location.
export ROS_HOME="${ARIAC_ROS_HOME:-/tmp/ariac_vla_ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
mkdir -p "$ROS_LOG_DIR"

BRIDGE_ARGS=()
SCENE="ariac"
TASK_PROFILE="${ARIAC_TASK_PROFILE:-inspect}"
for a in "$@"; do
  case "$a" in
    --view) BRIDGE_ARGS+=(--view) ;;
    --scene|--task) : ;;
    ariac|warehouse) SCENE="$a" ;;
    inspect|vla|generic) : ;;
    *)      BRIDGE_ARGS+=("$a") ;;
  esac
done
if printf '%s\n' "$@" | grep -qx -- '--view'; then
  # MuJoCo viewer and the inspection OpenCV preview both require desktop GL.
  export MUJOCO_GL=glfw
fi
# 处理 --scene xxx 形式
for ((i=1; i<=$#; i++)); do
  if [[ "${!i}" == "--scene" ]]; then
    j=$((i+1)); SCENE="${!j}"
  elif [[ "${!i}" == "--task" ]]; then
    j=$((i+1)); TASK_PROFILE="${!j}"
  fi
done
if [[ "$TASK_PROFILE" != "inspect" && "$TASK_PROFILE" != "vla" && "$TASK_PROFILE" != "generic" ]]; then
  echo "未知任务: $TASK_PROFILE（可选: inspect, vla, generic）" >&2; exit 1
fi

# 清理遗留进程
pkill -u "$USER" -f "bridge_warehouse.py" 2>/dev/null || true
pkill -u "$USER" -f "bridge_ariac.py" 2>/dev/null || true
pkill -u "$USER" -f "nav_p2p.py" 2>/dev/null || true
pkill -u "$USER" -f "rviz2.*view_map" 2>/dev/null || true
sleep 2

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 根据场景选择桥接脚本
if [ "$SCENE" = "ariac" ]; then
  if [ "$TASK_PROFILE" = "generic" ]; then
    echo "ARIAC 请显式选择 --task inspect 或 --task vla" >&2; exit 1
  fi
  BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"
  export ARIAC_TASK_PROFILE="$TASK_PROFILE"
  python3.8 model/scenes/build_ariac_compat_meshes.py --check >/dev/null ||
    python3.8 model/scenes/build_ariac_compat_meshes.py
  # Keep the robot wheel mesh on the ARIAC floor when navigation regenerates
  # the composed model at startup.
  python3.8 model/robot/gen_ariac_robot.py --start-z -0.08
  if [ "$TASK_PROFILE" = "vla" ]; then
    # Randomize only the VLA scene.  Inspection loads the independent base
    # scene and can no longer inherit VLA object/camera task state.
    VLA_XML="model/robot/ariac_lab_with_robot_3d_vla.xml"
    if [ -n "${VLA_SEED:-}" ]; then
      python3.8 _ultimate_task/vla/randomize_ariac_grasp.py \
        --input model/robot/ariac_lab_with_robot_3d.xml \
        --output "$VLA_XML" --seed "$VLA_SEED"
    else
      python3.8 _ultimate_task/vla/randomize_ariac_grasp.py \
        --input model/robot/ariac_lab_with_robot_3d.xml \
        --output "$VLA_XML"
    fi
    export MUJOCO_SCENE_XML="$HERE/$VLA_XML"
  else
    if [ -n "${VLA_SEED:-}" ]; then
      echo "VLA_SEED 只能与 --task vla 一起使用" >&2; exit 1
    fi
    export MUJOCO_SCENE_XML="$HERE/model/robot/ariac_lab_with_robot_3d.xml"
  fi
elif [ "$SCENE" = "warehouse" ]; then
  BRIDGE_SCRIPT="slam/bridge/bridge_warehouse.py"
  export ARIAC_TASK_PROFILE="generic"
else
  echo "未知场景: $SCENE（可选: ariac, warehouse）" >&2; exit 1
fi

echo "[1/3] 起 MuJoCo 3D 仿真桥接 ($SCENE/$ARIAC_TASK_PROFILE, 实时雷达用于动态避障) ..."
python3.8 "$BRIDGE_SCRIPT" "${BRIDGE_ARGS[@]}" &
PIDS+=("$!")
sleep 5
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
  echo "错误：MuJoCo bridge 启动后立即退出；请检查上方错误日志。" >&2
  exit 1
fi

# 方案 B 没有 rtabmap，需要手动发布 map->odom 静态 TF（identity）
echo "      发布 map->odom 静态 TF ..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom --ros-args -p use_sim_time:=true &
PIDS+=("$!")

echo "[2/3] 起导航节点 (nav_p2p.py --use-saved --scene $SCENE) ..."
python3.8 nav_p2p.py --use-saved --scene "$SCENE" &
PIDS+=("$!")
sleep 3
if ! kill -0 "${PIDS[2]}" 2>/dev/null; then
  echo "错误：nav_p2p.py 启动后立即退出；请检查地图路径或 Python/ROS 环境。" >&2
  exit 1
fi

echo "[3/3] 起 RViz2 ..."
rviz2 -d "$HERE/slam/view_map.rviz" --ros-args -p use_sim_time:=true &
PIDS+=("$!")

cat <<'EOF'

=== 导航已启动（保存地图 + Lazy Theta* + 条件式全向 DWA）===

操作方法：
  1. 在 RViz2 工具栏点 "2D Goal Pose"（Nav Goal 按钮）
  2. 在地图上点击目标位置并拖动设置方向
  3. 机器人会自动规划路径并导航过去

或命令行发目标点：
  ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"

监控：
  ros2 topic echo /nav_status    # 查看状态: IDLE/PLANNING/FOLLOWING/ARRIVED/STUCK
  ros2 topic echo /nav_path      # 查看路径

Ctrl-C 退出全部。
EOF
wait
