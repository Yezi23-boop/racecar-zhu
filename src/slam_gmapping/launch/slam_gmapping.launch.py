from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('linear_update', default_value='0.20'),
        DeclareLaunchArgument('angular_update', default_value='0.10'),
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'linear_update': ParameterValue(LaunchConfiguration('linear_update'), value_type=float),
                'angular_update': ParameterValue(LaunchConfiguration('angular_update'), value_type=float),
            }],
        ),
    ])
