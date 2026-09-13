#!/usr/bin/env bash
# 只重启导航节点 nav_p2p.py，MuJoCo 仿真/RViz 窗口保持不动。
# 用于"边改代码边看效果"：改完 nav_p2p.py 跑一次本脚本即可。
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
source /opt/ros/foxy/setup.bash

echo "[1/3] 停止旧 nav_p2p ..."
pkill -f "nav_p2p[.]py" 2>/dev/null || true
sleep 1

echo "[2/3] 兜底停车（防止桥接沿用最后一条指令）..."
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
sleep 1

echo "[3/3] 启动新 nav_p2p ..."
python3.8 nav_p2p.py --use-saved > /tmp/opencode/nav_p2p.log 2>&1 &
sleep 2

echo "done: nav_p2p 已重启，新代码已生效。日志 /tmp/opencode/nav_p2p.log"
