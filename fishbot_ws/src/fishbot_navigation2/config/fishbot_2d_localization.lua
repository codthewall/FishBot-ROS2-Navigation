-- fishbot_2d_localization.lua
-- 麦克纳姆轮机器人纯定位配置（Cartographer 2D）

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  
  -- 关键修改：禁用 Cartographer 提供 odom frame（由 EKF 提供）
  provide_odom_frame = false,
  
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  
  -- 使用外部里程计（来自 EKF 的编码器融合数据）
  use_odometry = true,
  
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 1.0,      -- 降低到1Hz，减少子图更新频率
  pose_publish_period_sec = 0.05,        -- 提高到20Hz，减少TF卡顿
  trajectory_publish_period_sec = 50e-3,
  
  -- 降低采样率以提高实时性
  rangefinder_sampling_ratio = 0.5,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 强制纯定位模式：只保留最近3个子图（防止内存增长）
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D轨迹构建器参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

-- 启用 IMU（关键：用于消除旋转累计误差，配合EKF融合编码器+IMU）
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 扫描匹配参数（针对麦克纳姆轮优化）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.0)

-- Ceres扫描匹配器权重（信任里程计还是信任扫描匹配）
-- 纯定位模式下应更信任扫描匹配，但保留一定里程计权重防止跳变
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50  -- From 30 to 50, enhanced rotation accuracy

-- 纯定位关键：降低全局优化频率，避免定位抖动
POSE_GRAPH.optimize_every_n_nodes = 100

-- 关键修改：提高旋转相关参数权重，提升重定位旋转精度
-- 约束构建器（提高阈值防止误匹配）
POSE_GRAPH.constraint_builder.min_score = 0.70
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Mecanum wheel specific optimization: increase rotation weight to compensate for slip
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- From 1e4 to 1e5, enhanced rotation constraint

return options