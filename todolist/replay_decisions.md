# 恢复笔记：replay_decisions.py 优化任务（2026-08-29 暂停，2026-09-03 续并收尾）

## 目标
离线决策回放：用已保存成品地图当 ground truth，模拟"逐步建图 + 探索决策"，
在真实 ariac 地图上几分钟内复现线上 frontier_explorer.py 的决策流，快速抓逻辑 bug。

## 已完成
- 新建 `slam/replay_decisions.py`（复用线上 `frontier_explorer.py` 的
  `_detect_frontiers`/`_select_frontier` + `explore_planner.py`，决策逻辑与真机一致）
- 跑通验证：加载 `maps/ariac/ariac_map_3d.yaml`（594×595 res=0.05，自由区 87.7%），
  起点 (4.0, 4.6) 检测到 frontier，能区分可达(✓)/不可达(✗)，决策打印清晰
- 输出文件 `/tmp/opencode/replay_run2.log` / `/tmp/opencode/replay_coverage.png`

## 2026-09-03 收尾内容
1. **性能节流（原待修 #1）**：`_rebuild_cost()`(EDT) 不再每 0.1s 步无条件重建，
   改成只在 frontier 重检分支（3s 一次 / 无目标时）重建，与线上 0.5s 节流等价；
   原地扫描期不再逐帧重检。300 步 ~1m30s 完成。
2. **修笔误**：`load_map(args.start and args.map)` → `load_map(args.map)`。
3. **新抓到的建模 bug（重要，线上无此问题）**：原"每步 360 条射线画点"建图太稀，
   7.5m 盘内残留 48.9% 针孔 unknown，把已知区打碎 → 可达性几乎全 ✗、只能选中
   脚边 0.4m 的假 frontier，机器人原地空转、cov 卡 12.4%。
   修法：FAN_RAYS=1200 条 + `binary_fill_holes` 补相邻射线间隙（等价 SLAM 逆
   传感器模型填充扇形），残留 unknown≈真实遮挡；揭示从每步改为每 0.5s/检测前。
4. **到达后扫描改短**：全向 360° 雷达 + 地图即时揭示时，原地转 12.5s 无信息增益，
   300 步只到 1 个目标。改成停车等"地图更新" 3s（对应线上 ROTATE_SCAN）。
   300 步验证：cov 21%→34.5%，完成跨区两次选点，决策/覆盖打印正常。

## 待办
- 方案二（原 #3，未做）：给线上 `frontier_explorer.py` 加结构化日志（打印全部候选
  分数/可达性/拉黑）+ `/frontier_markers` 可达/不可达颜色区分。
- 可选：回放目前行驶中每 3s 打印的"←选中"是"若有目标会选谁"，不会真切换目标
  （线上会在目标被覆盖时取消重选）；如需更贴近可加 goal 重校验。

## 运行方式
```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash   # 需要（rclpy 是 cpython-38）
python3.8 slam/replay_decisions.py --map maps/ariac/ariac_map_3d.yaml \
    --start 4.0,4.6 --max-steps 300
eog /tmp/opencode/replay_coverage.png
```

## 关键事实（背景，勿忘）
- 当前默认 `ODOM_NOISE_DEFAULT=False`（无噪里程计），rtabmap 无漂移可纠正，
  回放验证过：459 节点 0 处 map→odom 跳变（>3cm 阈值），loop closure 风险
  在此配置下不会触发，explore 可安全跑。
- 现有 patrol 会话 19:37 起已空转数小时但建图早已完成（sim 3..473s），
  地图已存 `maps/ariac/`，`--fresh` 杀掉不丢进度。
