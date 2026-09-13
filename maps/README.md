# Maps 目录说明

本目录按场景分开存放 SLAM 建图产生的地图文件。

## 运行方式

保存地图（`maps/ariac/` 是 `model/scenes/ariac_lab.xml` 静态场景和
`model/robot/ariac_lab_with_robot_3d.xml` SLAM 场景的地图产物目录）：

```bash
./slam/mapping/run_slam_3d.sh --scene ariac --fresh --explore
./slam/mapping/save_map_3d.sh                    # 默认保存 ARIAC 地图
./slam/mapping/save_map_3d.sh --scene ariac      # 保存到 maps/ariac/
```

查看地图：

```bash
./slam/navigation/view_map.sh maps/ariac/ariac_map_3d.yaml
```

路径规划（ARIAC 默认；需先完成并保存 ARIAC 地图）：

```bash
./slam/navigation/run_nav_saved.sh --view
./slam/navigation/run_nav_saved.sh --scene ariac --view
```

动态行人避障测试：使用 ARIAC 保存的静态地图进行 Lazy Theta* 全局规划，同时启用
实时点云和条件式全向 DWA。测试行人会在巡检路线上横穿并短暂停留；只有行人进入
当前路径走廊时，导航状态才会切换到 `DYNAMIC_AVOID`。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
./slam/navigation/run_nav_saved.sh --scene ariac --view --dynamic-person
```

在 RViz2 中沿巡检路线设置目标，或发送以下目标点：

```bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

观察导航状态：

```bash
ros2 topic echo /nav_status
```

正常情况下可以观察到：

```text
PLANNING -> FOLLOWING -> DYNAMIC_AVOID -> FOLLOWING -> ARRIVED
```

## 文件夹中文件介绍

```text
maps/
├── ariac/                  # 默认 ARIAC 仪表场景地图（首次保存时创建）
│   ├── ariac_map_3d.pgm
│   ├── ariac_map_3d.yaml
│   └── rtabmap.db
├── lab_map.yaml            # 实验室参考地图
└── README.md
```

## 可调节参数

| 参数 | 说明 |
|------|------|
| `--scene ariac` | 选择地图所属场景；省略时默认 `ariac` |
| `--dynamic-person` | 启用动态行人避障测试；不传时不影响普通静态地图导航 |

坐标系（世界系 → map 系：`(x - origin_x, y - origin_y)`）：

| 场景 | 狗起点(世界系) | map 原点 |
|------|---------------|----------|
| ariac | (4.0, 4.6) | 狗起点 |

## 备注

修改仪表、消防栓、罐体、柜体或其他碰撞几何后，应使用 `--fresh` 重新建图并覆盖
保存；旧的 `.pgm`、`.ply` 和 `rtabmap.db` 不会自动随 XML 更新。

动态行人默认停在场外。MuJoCo 查看器会隐藏黄色雷达射线，但实时雷达计算和
`/pointcloud` 仍保持启用。
