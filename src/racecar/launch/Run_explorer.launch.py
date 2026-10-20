"""
Run_explorer.launch.py - racecar Cartographer 自主探索模式
移植自仿真项目 turtlebot3_ws 的 explorer 方案

架构（与仿真完全一致）：
  Laser + IMU → Cartographer → map→odom→base_footprint TF + /map
  Cartographer /map → Nav2 (navigation_launch.py, 无 AMCL)

TF 链路：
  map → odom → base_footprint → laser_link / IMU_link
  (map→odom 和 odom→base_footprint 全部由 Cartographer 发布)

启动方式：
  ros2 launch racecar Run_explorer.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    racecar_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(racecar_dir, 'launch')

    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('imu_get')

    # Nav2 参数文件（Cartographer 专用，无 AMCL）
    nav_carto_param = os.path.join(racecar_dir, 'config', 'nav_carto.yaml')

    # RViz2 配置
    rviz_config_dir = os.path.join(racecar_dir, 'rviz', 'racecar_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),

        # ==========================================
        # 第一层：底盘驱动 + 传感器
        # ==========================================

        # 底盘驱动
        Node(
            package='racecar_driver',
            executable='racecar_driver_node',
            name='racecar_driver',
        ),

        # 雷达 LSN10
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_driver_share_dir, 'launch', 'lsn10_launch.py')
            )
        ),

        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_launch_share_dir, 'launch', 'imu_get.launch.py')
            )
        ),

        # 静态 TF：base_footprint → laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2laser_link',
            arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'laser_link']
        ),

        # 静态 TF：base_footprint → IMU_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2imu_link',
            arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'IMU_link']
        ),

        # ==========================================
        # 第二层：Cartographer SLAM
        # 和仿真方案完全一致：纯 Laser + IMU
        # Cartographer 自行提供 odom frame，发布：
        #   map → odom → base_footprint 的完整 TF
        # 延迟 3 秒启动，等传感器就绪
        # ==========================================
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'cartographer.launch.py')
                    ),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                ),
            ]
        ),

        # ==========================================
        # 第三层：Nav2 导航栈
        # 使用裁剪版 navigation_no_smoother.launch.py
        # 去掉 smoother_server，避免实车上生命周期切换异常
        # 延迟 4 秒启动，等 Cartographer 就绪
        # ==========================================
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation_no_smoother.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav_carto_param,
                        'use_composition': 'False',
                        'autostart': 'True',
                    }.items(),
                ),
            ]
        ),

        # ==========================================
        # RViz2 可视化（默认启用）
        # 使用 --no-rviz 禁用：bash explorer.sh --no-rviz
        # ==========================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
            condition=UnlessCondition(LaunchConfiguration('no_rviz', default='false')),
        ),
    ])
