"""Static-map navigation vehicle bringup.

RF2O publishes odom -> base_footprint.  The legacy EKF is intentionally not
started because it publishes odom_combined -> base_footprint and conflicts
with the TF tree used by the current Nav2 configuration.
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
    rf2o_dir = get_package_share_directory("rf2o_laser_odometry")
    racecar_dir = get_package_share_directory("racecar")
    rviz_config = os.path.join(racecar_dir, "rviz", "nav2_default_view.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to start RViz2",
            ),
            # go.py relays /cmd_vel to /car_cmd_vel for multi-point navigation.
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        rf2o_dir,
                        "launch",
                        "rf2o_laser_odometry.launch.py",
                    )
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2base_link",
                arguments=[
                    "0.0", "0.0", "0.15",
                    "0.0", "0.0", "0.0",
                    "base_footprint", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2laser_link",
                arguments=[
                    "0.07", "0.0", "0.0",
                    "0.0", "0.0", "0.0",
                    "base_footprint", "laser_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint2imu_link",
                arguments=[
                    "0.1653", "0.0", "0.0",
                    "0.0", "0.0", "0.0",
                    "base_footprint", "IMU_link",
                ],
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
