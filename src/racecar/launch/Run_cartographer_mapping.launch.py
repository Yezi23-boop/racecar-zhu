"""
Cartographer 纯建图启动文件。

只启动实车底层硬件、传感器和 Cartographer，不启动 RF2O、EKF、AMCL 或 Nav2。
Cartographer 根据 racecar_2d.lua 使用雷达和 IMU，并自行发布：
  map -> odom -> base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    racecar_dir = get_package_share_directory("racecar")
    racecar_launch_dir = os.path.join(racecar_dir, "launch")
    lidar_dir = get_package_share_directory("lslidar_driver")
    imu_dir = get_package_share_directory("imu_get")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            Node(
                package="racecar_driver",
                executable="racecar_driver_node",
                name="racecar_driver",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_dir, "launch", "lsn10_launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(imu_dir, "launch", "imu_get.launch.py")
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2laser_link",
                arguments=[
                    "0.07",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_footprint",
                    "laser_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2imu_link",
                arguments=[
                    "0.1653",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_footprint",
                    "IMU_link",
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                racecar_launch_dir, "cartographer.launch.py"
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "resolution": resolution,
                            "publish_period_sec": publish_period_sec,
                        }.items(),
                    )
                ],
            ),
        ]
    )
