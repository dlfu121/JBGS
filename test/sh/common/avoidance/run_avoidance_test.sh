#!/bin/bash
# Non-interactive entry point for avoidance and tabletop regression tests.
set -euo pipefail

cd "$(dirname "$0")"

usage() {
    cat <<'EOF'
Usage: ./run_avoidance_test.sh [regression|quick|nav|ros]

  regression  Run the full headless MuJoCo patrol/contact/tabletop test
              (default, about two minutes).
  quick       Compile the bridge modules and test clearance/state transitions.
  nav         Test saved-map inflation, LOS, and safe corner following.
  ros         Start ROS 2, RTAB-Map, and the MuJoCo viewer with a fresh map.
EOF
}

case "${1:-regression}" in
    regression)
        exec python3.8 ../../../py/warehouse/avoidance/test_avoidance_simple.py
        ;;
    quick)
        python3.8 ../../../py/common/avoidance/verify_avoidance_code.py
        exec python3.8 ../../../py/common/navigation/test_explore_planner.py
        ;;
    nav)
        unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT
        unset ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
        set +u
        source /opt/ros/foxy/setup.bash
        set -u
        exec python3.8 ../../../py/common/navigation/test_nav_safety.py
        ;;
    ros)
        exec ../../../../slam/run_slam_3d.sh --view --fresh
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
