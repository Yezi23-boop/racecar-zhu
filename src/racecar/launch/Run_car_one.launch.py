"""
静态地图导航的实车底层启动文件。

无编码器车辆使用 Cartographer 激光 + IMU 局部里程计：
  Cartographer: odom -> base_footprint
  AMCL: map -> odom（由 Run_nav.launch.py 启动）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_dir = get_package_share_directory("lslidar_driver")
    imu_dir = get_package_share_directory("imu_get")
    racecar_dir = get_package_share_directory("racecar")
    rviz_config = os.path.join(racecar_dir, "rviz", "nav2_default_view.rviz")
    use_rviz = LaunchConfiguration("use_rviz")
    driver_executable = LaunchConfiguration("driver_executable")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to start RViz2",
            ),
            DeclareLaunchArgument(
                "driver_executable",
                default_value="racecar_driver_node_one",
                description="Racecar driver executable",
            ),
            Node(
                package="racecar_driver",
                executable=driver_executable,
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
                name="base_footprint2base_link",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.15",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2laser_link",
                arguments=[
                    "--x",
                    "0.07",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "laser_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2imu_link",
                arguments=[
                    "--x",
                    "0.1653",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "IMU_link",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(racecar_dir, "launch", "cartographer_odometry.launch.py")
                ),
                launch_arguments={"use_sim_time": "false"}.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
