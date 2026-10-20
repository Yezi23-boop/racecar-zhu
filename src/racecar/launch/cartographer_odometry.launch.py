"""Cartographer laser/IMU local odometry for static-map navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    racecar_share = get_package_share_directory("racecar")
    config_dir = os.path.join(racecar_share, "config", "cartographer")

    return LaunchDescription(
        [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_odometry",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    "racecar_odometry_2d.lua",
                ],
                remappings=[("imu", "/IMU_data")],
            ),
            Node(
                package="racecar",
                executable="cartographer_odom_bridge.py",
                name="cartographer_odom_bridge",
                output="screen",
                parameters=[
                    {
                        "odom_frame": "odom",
                        "base_frame": "base_footprint",
                        "odom_topic": "/odom_cartographer",
                        "publish_rate": 20.0,
                    }
                ],
            ),
        ]
    )
