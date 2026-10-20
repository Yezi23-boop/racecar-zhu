-- Cartographer 2D local odometry for nav_one.
--
-- This trajectory deliberately uses "odom" as Cartographer's local map frame.
-- Cartographer is the only publisher of odom -> base_footprint.  AMCL remains
-- responsible for map -> odom so the existing PGM/YAML map can still be used.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "odom",
  tracking_frame = "IMU_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.08
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- Reject scan noise while the vehicle is stationary, but retain normal
-- low-speed motion updates.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.15)

-- Local odometry must not perform loop-closure jumps.  AMCL supplies the
-- global correction against the static map.
POSE_GRAPH.optimize_every_n_nodes = 0
-- Cartographer 会在采样率精确等于 0 时持续输出
-- "FixedRatioSampler is dropping all data"。使用极小的非零值可避免刷屏，
-- 同时配合 optimize_every_n_nodes = 0，仍然不会执行回环优化或造成位姿跳变。
POSE_GRAPH.global_sampling_ratio = 0.000001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.000001
POSE_GRAPH.constraint_builder.log_matches = false

return options
