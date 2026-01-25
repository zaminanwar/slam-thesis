"""
Cartographer 2D evaluation launch file.

Launches Cartographer with trajectory exporter for evaluation.
Requires a running Gazebo simulation with ground truth publisher.

Arguments:
    output_dir: Directory to write trajectory files (required)
    use_sim_time: Use simulation time (default: true)

Usage:
  # First, start the simulation with ground truth:
  ros2 launch rover_sim sim.launch.py
  ros2 launch gt_publisher gt_publisher.launch.py

  # Then, start SLAM evaluation:
  ros2 launch slam_launch cartographer_eval.launch.py output_dir:=/path/to/results

  # Optionally, follow a trajectory:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy

  # When done, Ctrl+C to save trajectory files
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')

    # === Launch arguments ===
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        description='Directory to write trajectory files (required)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')

    # === Config paths ===
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')
    cartographer_config_basename = 'cartographer_2d.lua'

    # === Cartographer node ===
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        output='screen',
    )

    # === Occupancy grid node (publishes /map) ===
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0},
        ],
        output='screen',
    )

    # === Trajectory exporter node ===
    # Note: GT uses base_footprint_gt to avoid TF conflict with main tree
    traj_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='traj_exporter',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'sample_rate': 20.0,
            'gt_parent_frame': 'map_gt',
            'gt_child_frame': 'base_footprint_gt',
            'est_parent_frame': 'map',
            'est_child_frame': 'base_footprint',
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        output_dir_arg,
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
        traj_exporter_node,
    ])
