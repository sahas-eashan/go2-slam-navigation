-- Derived from TurtleBot3 LDS 2D config, adapted for Go2 frames and /scan topic
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.05,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 4.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Accumulate two scans to help low-rate (~8 Hz) lidars during motion
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.05
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter = {
  max_length = 0.5,
  min_num_points = 150,
  max_range = 20.,
}

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

return options
