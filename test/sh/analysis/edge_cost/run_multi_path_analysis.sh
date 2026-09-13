#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
set +u
source /opt/ros/foxy/setup.bash
set -u
export MPLCONFIGDIR="${TMPDIR:-/tmp}/worker_scene_matplotlib"
mkdir -p "$MPLCONFIGDIR"
exec python3.8 "$ROOT/test/py/analysis/edge_cost/run_multi_path_analysis.py" "$@"
