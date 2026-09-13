#!/usr/bin/env python3
"""起 rtabmap 做 3D 激光 SLAM，配合 slam_bridge_3d.py 发布的点云和里程计。

rtabmap 订阅 /pointcloud + /odom，用 ICP 做帧间匹配和回环检测，
输出 /cloud_map (3D 点云地图) 和 /map (2D 投影栅格)。

用法:
    source /opt/ros/foxy/setup.bash
    ros2 launch ./slam/rtabmap_3d.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "rtabmap_params.yaml")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # rtabmap SLAM 节点
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                PARAMS,
                {
                    "use_sim_time": use_sim_time,
                    "publish_tf": True,
                    "queue_size": 10,
                    # 注：不要在这里设 "Grid/FromDepth"。rtabmap 0.21 已用
                    # Grid/Sensor 取代它（见 rtabmap_params.yaml），而且这里的
                    # 覆盖会在参数文件之后生效，反而把 yaml 里的设置顶掉。
                }
            ],
            remappings=[
                ("scan_cloud", "/pointcloud"),
                ("odom", "/odom"),
            ],
            arguments=["--delete_db_on_start"],
        ),
    ])


if __name__ == "__main__":
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    raise SystemExit(ls.run())
