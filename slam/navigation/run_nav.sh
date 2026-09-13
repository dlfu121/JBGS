#!/usr/bin/env bash
# 起点对点导航节点。前提：3D SLAM 桥接 + rtabmap 已在跑（./slam/mapping/run_slam_3d.sh）。
# 或使用 --use-saved 加载保存地图离线测试。
#
# 用法:
#   ./slam/navigation/run_nav.sh                          # 在线 /map（ariac）
#   ./slam/navigation/run_nav.sh --use-saved --scene ariac # 离线加载 ARIAC 地图
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi

# 清除 ROS1 残留
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

exec python3.8 slam/navigation/nav_p2p.py "$@"
