"""
Trajectory follower launch file.

Launches the trajectory_follower node to make the rover follow a predefined path.
Expects the simulation (rover_sim) to already be running.

Usage:
  # Basic usage with defaults:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv

  # With custom parameters:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv \
      lookahead_distance:=0.6 speed_scale:=0.8

  # With SLAM-corrected pose feedback:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv \
      use_slam_pose:=true pose_frame:=map

  # With ground truth pose (oracle mode):
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv \
      use_slam_pose:=true pose_frame:=map_gt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Default trajectory directory
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # ========== Basic Parameters ==========
    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.15',
        description='Waypoint reached threshold (m) - must be smaller than waypoint spacing'
    )

    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale',
        default_value='1.0',
        description='Speed multiplier applied to CSV speeds'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from /clock'
    )

    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Lookahead distance for Pure Pursuit (m)'
    )

    # ========== Velocity Control Parameters ==========
    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='0.8',
        description='Maximum angular velocity (rad/s)'
    )

    max_linear_accel_arg = DeclareLaunchArgument(
        'max_linear_accel',
        default_value='0.8',
        description='Maximum linear acceleration (m/s^2) for velocity smoothing'
    )

    max_angular_accel_arg = DeclareLaunchArgument(
        'max_angular_accel',
        default_value='0.8',
        description='Maximum angular acceleration (rad/s^2) for velocity smoothing'
    )

    max_centripetal_accel_arg = DeclareLaunchArgument(
        'max_centripetal_accel',
        default_value='0.5',
        description='Maximum centripetal acceleration (m/s^2) - limits speed in turns'
    )

    # ========== Smoothing Parameters ==========
    angular_filter_alpha_arg = DeclareLaunchArgument(
        'angular_filter_alpha',
        default_value='0.4',
        description='Low-pass filter coefficient (0-1, lower = more smoothing)'
    )

    use_velocity_smoothing_arg = DeclareLaunchArgument(
        'use_velocity_smoothing',
        default_value='true',
        description='Enable velocity rate limiting (prevents jerky motion)'
    )

    # ========== Rotation Behavior Parameters ==========
    rotate_in_place_threshold_arg = DeclareLaunchArgument(
        'rotate_in_place_threshold',
        default_value='0.785',
        description='Angle threshold for pure rotation (rad) - ~45 degrees'
    )

    pure_pursuit_threshold_arg = DeclareLaunchArgument(
        'pure_pursuit_threshold',
        default_value='0.524',
        description='Angle threshold for pure pursuit (rad) - ~30 degrees'
    )

    # ========== Pose Source Parameters ==========
    use_slam_pose_arg = DeclareLaunchArgument(
        'use_slam_pose',
        default_value='false',
        description='Use TF lookup for pose instead of raw /odom (requires SLAM or GT publisher)'
    )

    pose_frame_arg = DeclareLaunchArgument(
        'pose_frame',
        default_value='map',
        description='TF frame for pose lookup: "map" for SLAM-corrected, "map_gt" for ground truth'
    )

    # Build trajectory path: if it starts with '/', use as-is, otherwise prepend default dir
    trajectory_file = PythonExpression([
        "'", LaunchConfiguration('trajectory'), "' if '", LaunchConfiguration('trajectory'),
        "'.startswith('/') else '", default_traj_dir, "/' + '", LaunchConfiguration('trajectory'), "'"
    ])

    # Trajectory follower node
    trajectory_follower_node = Node(
        package='rover_control',
        executable='trajectory_follower',
        name='trajectory_follower',
        parameters=[{
            # Basic
            'trajectory_file': trajectory_file,
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'speed_scale': LaunchConfiguration('speed_scale'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            # Velocity control
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'max_linear_accel': LaunchConfiguration('max_linear_accel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
            'max_centripetal_accel': LaunchConfiguration('max_centripetal_accel'),
            # Smoothing
            'angular_filter_alpha': LaunchConfiguration('angular_filter_alpha'),
            'use_velocity_smoothing': LaunchConfiguration('use_velocity_smoothing'),
            # Rotation behavior
            'rotate_in_place_threshold': LaunchConfiguration('rotate_in_place_threshold'),
            'pure_pursuit_threshold': LaunchConfiguration('pure_pursuit_threshold'),
            # Pose source
            'use_slam_pose': LaunchConfiguration('use_slam_pose'),
            'pose_frame': LaunchConfiguration('pose_frame'),
        }],
        output='screen'
    )

    return LaunchDescription([
        # Basic
        trajectory_arg,
        goal_tolerance_arg,
        speed_scale_arg,
        use_sim_time_arg,
        lookahead_distance_arg,
        # Velocity control
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        max_linear_accel_arg,
        max_angular_accel_arg,
        max_centripetal_accel_arg,
        # Smoothing
        angular_filter_alpha_arg,
        use_velocity_smoothing_arg,
        # Rotation behavior
        rotate_in_place_threshold_arg,
        pure_pursuit_threshold_arg,
        # Pose source
        use_slam_pose_arg,
        pose_frame_arg,
        # Node
        trajectory_follower_node,
    ])
