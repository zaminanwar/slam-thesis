-- Cartographer 2D configuration for SLAM evaluation
-- Reference: https://google-cartographer-ros.readthedocs.io/

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- === Frame Configuration ===
  -- Odometry publishes odom->base_footprint, so we track base_footprint
  -- TF chain: laser_frame -> base_link -> base_footprint -> odom
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",

  -- Use existing odometry from bag (odom -> base_footprint)
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  -- === Sensor Configuration ===
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- Number of LiDARs
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- === Timing ===
  lookup_transform_timeout_sec = 2.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- === Range Filter ===
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- === Map Builder Configuration ===
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- === Trajectory Builder 2D Configuration ===
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Motion filter: how much movement before inserting new scan
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.1

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- === Pose Graph Configuration ===
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 35

-- Constraint builder (loop closure)
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Loop closure search - increased weights for stronger correction (was 1.1e4/1e5)
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 5e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 5e5
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- Global optimization
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
