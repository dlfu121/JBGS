# MuJoCo 模型目录

模型按职责分为三类：`scenes/` 存放纯场景和场景构建脚本，`robot/` 存放机器人
以及场景与机器人的组合模型，`assets/` 存放网格、纹理和转换元数据。

## ARIAC 主场景

| 路径 | 用途 |
| --- | --- |
| `scenes/ariac_lab.xml` | 不含移动机器人的 ARIAC 2025 仪表实验室主场景 |
| `assets/ariac/meshes/` | ARIAC OBJ、MTL 和纹理资源 |
| `assets/ariac/meshes/compat/` | MuJoCo 3.2.3 可加载的零厚度网格副本 |
| `assets/ariac/conversion_report.json` | 原场景转换统计 |
| `robot/ariac_lab_with_robot_3d.xml` | ARIAC + 四足机器人 + 3D 雷达，供 SLAM 使用 |

纯场景可直接用仓库中的 MuJoCo 3.10 `simulate` 查看：

```bash
../../bin/simulate model/scenes/ariac_lab.xml
```

项目的 ROS 2 SLAM 使用 Python 3.8 + MuJoCo 3.2.3。更新 ARIAC OBJ 或机器人
基础模型后，按顺序重新生成兼容资源和组合模型：

```bash
python3.8 model/scenes/build_ariac_compat_meshes.py
python3.8 model/robot/gen_ariac_robot.py
```

默认机器人起点是 `(4, 4.6)`。需要调整时可传入：

```bash
python3.8 model/robot/gen_ariac_robot.py --start-x 4 --start-y 4.6
```

ARIAC 组合模型沿用源机器人模型中 `lidar3d_frame` 的 `0.95 m` 安装高度。
仪表、消防栓、罐体或柜体几何发生变化后，需要重新生成组合模型，并重新构建
`maps/ariac/` 中的 SLAM 地图，不能继续把旧地图用于导航。

生成后运行模型回归：

```bash
./test/sh/ariac/model/run_ariac_scene_test.sh
```
