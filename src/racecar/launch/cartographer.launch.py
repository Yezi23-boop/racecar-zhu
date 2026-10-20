"""
Cartographer SLAM launch for racecar (实车)
启动 cartographer_node + occupancy_grid_node
使用 racecar 自带的 lua 配置: racecar_2d.lua
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Cartographer 配置目录（racecar 包内）
    racecar_share = get_package_share_directory('racecar')
    cartographer_config_dir = os.path.join(racecar_share, 'config', 'cartographer')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Resolution of occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0',
                              description='OccupancyGrid publishing period'),

        # Cartographer SLAM 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'racecar_2d.lua'
            ],
            # remap IMU 话题：racecar 的 IMU_data (frame_id=IMU_link) → Cartographer 默认 /imu
            remappings=[
                ('imu', '/IMU_data'),
            ],
        ),

        # Occupancy Grid 节点（将 Cartographer submap 转为 /map）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution,
                       '-publish_period_sec', publish_period_sec],
        ),
    ])
