include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames
  map_frame = "map",                  -- global SLAM frame
  tracking_frame = "imu_link",        -- frame of your IMU
  published_frame = "base_link",      -- frame to be used for Nav2
  odom_frame = "odom",                -- continuous frame if using odometry (optional here)
  provide_odom_frame = true,          -- Cartographer will publish map->odom
  publish_frame_projected_to_2d = true,

  -- Sensors
  use_odometry = false,               -- no wheel odometry
  use_nav_sat = false,                 -- no GPS
  use_landmarks = false,               -- no landmarks

  -- Lidar setup
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Topic timeouts & publishing rates
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,

  -- Sampling ratios (all 1.0 for full data)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,   -- corrected name
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- IMU (6-DOF)
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- Laser parameters (tuned for a 2D LiDAR ~10Hz)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02  -- downsample for speed

-- Submap size & resolution
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03  -- 3cm cells

-- Scan matcher (online correlative helps robust initialization)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.6,
  angular_search_window = math.rad(20.0),
  translation_delta_cost_weight = 1e-1,
  rotation_delta_cost_weight = 1e-1,
}

-- Pose graph / loop closure
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
