#!/usr/bin/env bash
# 保存 rtabmap 3D SLAM 建图结果。
#
# 用法:
#   ./slam/mapping/save_map_3d.sh                 # 默认保存 ARIAC 场景
#   ./slam/mapping/save_map_3d.sh --scene ariac   # 保存到 maps/ariac/
#   ./slam/mapping/save_map_3d.sh --scene ariac_lab  # ariac 的别名，仍保存到 maps/ariac/
#   ./slam/mapping/save_map_3d.sh --scene ariac --finalize  # 停止接收数据并保存最终地图
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

# 解析参数
SCENE="ariac"
FINALIZE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) SCENE="$2"; shift 2 ;;
    --finalize) FINALIZE=1; shift ;;
    *)       shift ;;
  esac
done

# 模型文件名是 ariac_lab.xml，但运行时场景名统一使用 ariac。接受文件名
# 形式的别名，避免同一个场景被误存成两套互不兼容的地图目录。
if [ "$SCENE" = "ariac_lab" ]; then
  SCENE="ariac"
fi

OUTDIR="$HERE/maps/$SCENE"
mkdir -p "$OUTDIR"

MAP_NAME="${SCENE}_map_3d"

echo "=== 保存 3D SLAM 地图到 $OUTDIR (场景: $SCENE) ==="

# 自动建图结束时，bridge 已停止 /pointcloud。再暂停 RTAB-Map，确保最终地图
# 发布、数据库复制和离线导出期间不会有新的关键帧写入。最终模式故意不 resume，
# 让 RViz 中的地图保持冻结；普通手动保存不改变 RTAB-Map 的运行状态。
if [ "$FINALIZE" -eq 1 ]; then
  echo "[finalize] 暂停 RTAB-Map，冻结最终地图 ..."
  if timeout 8s ros2 service call /pause std_srvs/srv/Empty "{}" \
      2>/dev/null; then
    echo "  RTAB-Map 已暂停（/pause）"
  elif timeout 8s ros2 service call /rtabmap/pause std_srvs/srv/Empty "{}" \
      2>/dev/null; then
    echo "  RTAB-Map 已暂停（/rtabmap/pause）"
  else
    echo "  ★ pause 服务不可用；点云已停止，继续保存静态结果" >&2
  fi
fi

# 1. 触发 rtabmap 发布完整地图
echo "[1/5] 请求 rtabmap 发布完整地图 ..."
if ! timeout 15s ros2 service call /publish_map \
    rtabmap_msgs/srv/PublishMap \
    "{global_map: true, optimized: true, graph_only: false}" \
    2>/dev/null; then
  echo "  (publish_map 服务不可用，跳过)"
fi

sleep 2

# 2. 保存 2D 栅格地图（nav2 格式）
echo "[2/5] 保存 2D 投影栅格 ..."
MAP_SAVED=0
# Foxy map_saver_cli uses a volatile subscription and can wait forever when it
# starts after RTAB-Map's last transient-local /map sample.  Match the publisher
# QoS directly and enforce both an internal wall-clock and process timeout.
if timeout 25s python3.8 "$HERE/slam/mapping/save_occupancy_grid.py" \
    --topic /map \
    --output "$OUTDIR/$MAP_NAME" \
    --timeout 20; then
  MAP_SAVED=1
  echo "  已保存 $OUTDIR/$MAP_NAME.pgm + .yaml"
else
  echo "  (map_saver 失败，可能 /map 尚未发布)"
fi

# 3. 复制 rtabmap 数据库
echo "[3/5] 复制 rtabmap 数据库 ..."
ROS_DB="${ROS_HOME:-$HOME/.ros}/rtabmap.db"
if [ -f "$ROS_DB" ]; then
  cp "$ROS_DB" "$OUTDIR/rtabmap.db"
  echo "  已保存 $OUTDIR/rtabmap.db"
else
  echo "  未找到 $ROS_DB"
fi

# 4. 导出 3D 点云
# 用 rtabmap-export 而不是 `ros2 run pcl_ros pointcloud_to_pcd`——本机没装
# pcl_ros，那条命令会直接失败。rtabmap-export 从数据库离线重建，好处是
# 用的是最终优化过的位姿图（比在线 /cloud_map 更准），而且不需要 SLAM 还在跑。
#   --scan  : 用激光点云（纯 lidar 建图必须加，否则它去找不存在的深度图）
#   --voxel : 0.03 m 体素下采样，保留足够密的障碍轮廓
# 导出的点云**包含地面**，与 RViz 里只显示障碍层的设置无关。
echo "[4/5] 导出 3D 点云 (PLY) ..."
if [ -f "$OUTDIR/rtabmap.db" ] && command -v rtabmap-export >/dev/null 2>&1; then
  rtabmap-export --scan --voxel 0.03 --max_range 0 \
    --output "$MAP_NAME" --output_dir "$OUTDIR" \
    "$OUTDIR/rtabmap.db" 2>&1 | tail -3
  PLY="$OUTDIR/${MAP_NAME}_cloud.ply"
  if [ -n "$PLY" ]; then
    echo "  已保存 $PLY ($(du -h "$PLY" | cut -f1))"
  else
    echo "  ★ 未生成 PLY，请手动检查：rtabmap-export --scan $OUTDIR/rtabmap.db"
  fi
else
  echo "  跳过（缺 rtabmap.db 或 rtabmap-export）"
fi

# 5. 用最终全局点云补全二维障碍轮廓。
# RTAB-Map 的 3D 概率栅格会在射线融合后稀释细长/重复观测的障碍；PLY
# 中仍保留这些命中点。保留原始 PGM 便于对比，增强后的文件仍使用原文件名，
# 因此现有 nav_p2p/RViz 无需改路径。
echo "[5/5] 用 3D 点云补全 PGM 障碍轮廓 ..."
RAW_PGM="$OUTDIR/${MAP_NAME}_raw.pgm"
PLY="${PLY:-}"
if [ "$MAP_SAVED" -ne 1 ] || [ ! -f "$OUTDIR/$MAP_NAME.pgm" ]; then
  echo "  ★ 缺少二维 PGM，无法补全轮廓" >&2
  exit 1
elif [ ! -f "$PLY" ]; then
  echo "  ★ 缺少 PLY，无法补全轮廓：保留原始 PGM" >&2
  exit 1
else
  cp "$OUTDIR/$MAP_NAME.pgm" "$RAW_PGM"
  python3.8 "$HERE/slam/mapping/project_cloud_to_pgm.py" \
    --pgm "$RAW_PGM" \
    --yaml "$OUTDIR/$MAP_NAME.yaml" \
    --ply "$PLY" \
    --output "$OUTDIR/$MAP_NAME.pgm" \
    --min-height 0.12 \
    --max-height 2.0 \
    --min-points 2 \
    --inflate-cells 1
fi

echo ""
echo "=== 完成 ==="
echo "后续路径规划用法："
echo "  2D 栅格(nav2): $OUTDIR/$MAP_NAME.yaml"
echo "  2D 原始栅格:   $RAW_PGM"
echo "  3D 点云(自定义): $OUTDIR/rtabmap.db"
