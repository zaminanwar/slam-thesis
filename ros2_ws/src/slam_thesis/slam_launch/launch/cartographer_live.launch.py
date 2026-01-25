"""
Cartographer 2D launch file for live simulation SLAM evaluation.

This launch file works with a running Gazebo simulation. It does NOT
replay bags - the simulation provides all sensor data and TF in real-time.

Usage:
  # First, start the simulation:
  ros2 launch rover_sim sim.launch.py

  # Then, start SLAM:
  ros2 launch slam_launch cartographer_live.launch.py

  # Optionally, follow a trajectory:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy
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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

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

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
    ])
