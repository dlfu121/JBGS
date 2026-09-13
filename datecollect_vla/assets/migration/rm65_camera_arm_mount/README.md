# RM65 + DexHand 相机—机械臂安装迁移包

这个目录记录当前 `rm65_dexhand_scene.xml` 中与迁移最相关的几何约定：

- 机械臂基座参考系：MuJoCo body `link_1`；
- 末端/手腕参考系：MuJoCo body `link_6`；
- 固定桌面相机：`table_camera`；
- 随腕运动相机：`wrist_overhead_camera`；
- 相机分辨率：640×480，控制/采集频率：30 Hz。

## 运行方式

迁移步骤：

1. 在新场景中建立与 `link_1` 等价的机械臂基座参考 body。
2. 将 `table_camera` 放到 `table_camera.pose.parent_frame` 指定的父坐标系，并复制
   `pos`、`xyaxes`、`fovy`。
3. 在新的末端 body（通常是 `link_6`）下添加 `wrist_overhead_camera`，复制其局部
   `pos`、`xyaxes`、`fovy`。
4. 保持两个相机名称不变，这样现有 ACT checkpoint 的视觉 feature key 不需要修改。
5. 如果新场景的机械臂基座整体移动，只需重新计算 `table_camera` 相对新基座的位姿；
   wrist 相机的局部位姿通常可以原样保留。
6. 用 `robot_migration.yaml` 的 `scene_path` 指向新 XML，然后先使用
   `--no-randomize --duration 10` 做推理 smoke test。

## 文件夹中文件介绍

| 文件 | 用途 |
|---|---|
| `camera_arm_relative.yaml` | 人和机器都容易读取的安装参数、相对位姿、相机参数和接口约定 |
| `camera_arm_relative.json` | 程序读取用的同一份参数，内容与 YAML 一致 |
| `camera_mounts.xml` | 可以直接拷贝到 MuJoCo XML 的相机片段和注释 |
| `robot_migration.yaml` | 迁移时使用的机器人配置模板，`scene_path` 需要替换 |

## 可调节参数

所有位置单位为米，角度单位为弧度（`fovy_deg` 除外）。MuJoCo 相机姿态保留为原始
`xyaxes` 六元组，这是最不容易在不同工具之间发生四元数顺序错误的表达方式。

`link_1` 是当前场景的机械臂基座 body。它在 world 中的原点为：

```text
(-0.700000, 0.000000, 1.919580) m
```

因此固定相机相对于 `link_1` 原点的平移为：

```text
(+0.200000, 0.000000, -0.040580) m
```

`wrist_overhead_camera` 是 `link_6` 的子相机，其局部平移为：

```text
(+0.020000, +0.001000, +0.017000) m
```

这个位姿会随 `link_6` 姿态变化，不能把它当成 world 固定相机。

| 参数 | 值 |
|---|---|
| 相机分辨率 | 640×480 |
| 控制/采集频率 | 30 Hz |
| 固定相机相对 `link_1` 平移 | `(+0.200000, 0.000000, -0.040580) m` |
| 腕部相机相对 `link_6` 平移 | `(+0.020000, +0.001000, +0.017000) m` |

## 备注

这份包只描述几何安装和接口，不包含 ACT 权重。ACT 仍要求状态、动作顺序、相机 key、
分辨率和归一化统计与训练 checkpoint 兼容。详细接口见 `camera_arm_relative.yaml`。
