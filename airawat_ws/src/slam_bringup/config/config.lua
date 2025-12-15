include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "imu_link",        -- IMU frame (must match your IMU message header.frame_id)
  published_frame = "base_link",      -- pose that cartographer publishes
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  -- We have a single 2D laser scan topic
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Use IMU (gyro + accel). Make sure /imu messages exist and header.frame_id == "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- Laser/scan parameters (tuned for X4 ~10Hz, ~0.097s scan_time)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- Keep single scan per update (X4 gives ~1 scan per 0.097s)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02  -- downsample points slightly

-- submap size & resolution
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90     -- ~90 scans per submap (tune as needed)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03 -- 3cm cell

-- scan matcher: use online correlative for robust init (slower but helps real robot)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.6,                       -- meters
  angular_search_window = math.rad(20.0),           -- radians
  translation_delta_cost_weight = 1e-1,
  rotation_delta_cost_weight = 1e-1,
}

-- Pose graph / loop closure
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
