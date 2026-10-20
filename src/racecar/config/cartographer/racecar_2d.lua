-- racecar Cartographer 2D SLAM 配置
-- 移植自仿真项目 turtlebot3_lds_2d.lua，适配实车硬件
-- 传感器：乐山雷达(lslidar) + IMU(imu_get)
-- 不使用轮速里程计，纯激光+IMU定位

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- tracking_frame 设为 IMU_link，启用 IMU 融合
  -- racecar 的 IMU link 名称是 "IMU_link"（区分大小写）
  tracking_frame = "IMU_link",
  -- published_frame 设为 base_footprint
  -- Cartographer 发布 map→odom→base_footprint 完整 TF 链
  published_frame = "base_footprint",
  odom_frame = "odom",
  -- Cartographer 自行创建 odom frame
  -- 真车没有 Gazebo 的里程计，由 Cartographer 提供 odom→base_footprint
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  -- 不使用轮速里程计
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 乐山雷达参数（根据 lsn10 实际规格调整）
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 启用 IMU 数据融合
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 纯激光扫描匹配（无里程计时必须开启）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动滤波器：小角度变化也要处理
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 回环检测与后端优化参数
-- 在 RISC-V 上长时间运行时，后端回环优化产生错误约束导致地图撕裂
-- 彻底禁用：关闭全局优化 + 禁用全局/非全局约束搜索
-- 前端扫描匹配已足够精确（静止漂移 <1cm/30s）
POSE_GRAPH.optimize_every_n_nodes = 0
-- 彻底禁用全局约束搜索（loop closure）
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
-- 减少全局约束搜索范围
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.log_matches = false

return options
