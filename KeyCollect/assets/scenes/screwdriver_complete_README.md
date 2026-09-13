# 完整螺丝刀 OBJ

文件：`screwdriver_complete.obj`  
材质：`screwdriver_complete.mtl`  
单位：米（m）

## 原点和坐标轴

该 OBJ 的原点与原仿真场景中 `screwdriver_red` body 的原点完全一致：

```text
原点 (0, 0, 0)：screwdriver_red body 根部
+X：从手柄指向金属杆/螺丝刀尖端（OBJ 文件坐标）
+Y、+Z：手柄横截面方向
```

因此它可以直接替代原来的“手柄 OBJ + MuJoCo cylinder”组合。迁移时如果保存的是 body 根部相对 `base_link` 的位置，数值不用改。

## 几何范围

```text
手柄中心：(-0.040, 0, 0) m
手柄范围：X = [-0.120, +0.040] m
手柄对边宽度：0.036 m（36 mm）
金属杆范围：X = [+0.040, +0.240] m
金属杆半径：0.006 m（6 mm）
总长度：0.360 m（360 mm）
```

## 与原场景的对应关系

原场景使用：

```text
body 根部：screwdriver_red
手柄局部中心：(-0.04, 0, 0) m
金属杆局部中心：(+0.14, 0, 0) m
```

新 OBJ 已经把这两部分合并，并沿 body 的局部 X 轴建模。由于 MuJoCo 的 OBJ 导入器会进行 up-axis 映射，使用 MuJoCo 时需给整个 mesh 一个与原圆柱相同的轴向转换四元数 `0.707105 0 0.707108 0`，将模型长度轴对齐到 MuJoCo body 的局部 X 轴。

## MuJoCo 引用示例

```xml
<asset>
  <mesh name="screwdriver_complete" file="screwdriver_complete.obj" />
</asset>

<body name="screwdriver_red"
      pos="-0.08 0.12 0.718"
      quat="0.819152 0 0 0.573576">
  <joint name="world_to_screwdriver_red" type="free" />
  <geom mesh="screwdriver_complete" pos="0 0 0"
        quat="0.707105 0 0.707108 0" rgba="1 1 1 1" />
</body>
```

注意：OBJ 和 MTL 必须放在同一个目录，或者同步修改 `mtllib` 路径。
