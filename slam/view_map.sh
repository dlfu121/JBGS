#!/usr/bin/env bash
# 一键打开已保存的地图并在 RViz2 中显示。
# 用法:
#   ./slam/view_map.sh                          # 默认打开 ARIAC 3D 地图
#   ./slam/view_map.sh maps/ariac/ariac_map_3d.yaml # 指定地图文件
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# 清除 ROS1 残留
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
source /opt/ros/foxy/setup.bash

MAP_YAML="${1:-$HERE/maps/ariac/ariac_map_3d.yaml}"
if [ ! -f "$MAP_YAML" ]; then
  echo "地图文件不存在: $MAP_YAML" >&2; exit 1
fi
MAP_YAML="$(realpath "$MAP_YAML")"

echo "加载地图: $MAP_YAML"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1. 启动 map_server
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAP_YAML" \
  -p use_sim_time:=false &
PIDS+=("$!")
sleep 2

# 2. 启动 lifecycle_manager 自动激活 map_server
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:="['map_server']" &
PIDS+=("$!")
sleep 3

echo "地图已发布到 /map topic"

# 3. 启动 RViz2（使用预配置的 rviz 文件，已设好 Map QoS）
rviz2 -d "$HERE/slam/view_map.rviz" --ros-args -p use_sim_time:=false &
PIDS+=("$!")

cat <<'EOF'

地图已加载并发布。在 RViz2 中:
  1. Fixed Frame 设为 "map"
  2. Add -> By topic -> /map -> Map -> OK

Ctrl-C 退出。
EOF
wait
