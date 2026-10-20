#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    # 注意这里的缩进是4个空格
    imu_driver_node = LifecycleNode(
        package='imu_get',
        executable='publisher_imu_node',
        name='publisher_imu_node',              # 设置节点名称
        output='screen',
        emulate_tty=True,
        namespace='',
    )

    return LaunchDescription([
        imu_driver_node,
    ])
