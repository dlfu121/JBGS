# Shelves corridor regression

四条路径均从两个方向通过货架区域；`gated` 是实际前方扫掠带门控后的净距。

| route | robot clearance | ungated lidar | gated lidar |
|---|---:|---:|---:|
| south_to_north | 0.729 | 0.954 | inf |
| north_to_south | 0.729 | 0.954 | inf |
| west_to_east | 0.728 | 0.547 | inf |
| east_to_west | 0.728 | 0.547 | inf |

合成回波验证：侧向 0.50 m 回波由门控忽略（旧中心线逻辑为 0.50 m），正前方阻塞回波触发 DWA，命令为 `(0.26, 0.12, -0.00)`。
