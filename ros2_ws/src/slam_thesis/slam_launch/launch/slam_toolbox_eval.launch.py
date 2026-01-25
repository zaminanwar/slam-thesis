"""
slam_toolbox evaluation launch file.

Launches slam_toolbox with trajectory exporter for evaluation.
Requires a running Gazebo simulation with ground truth publisher.

Arguments:
    output_dir: Directory to write trajectory files (required)
    use_sim_time: Use simulation time (default: true)

Usage:
  # First, start the simulation with ground truth:
  ros2 launch rover_sim sim.launch.py
  ros2 launch gt_publisher gt_publisher.launch.py

  # Then, start SLAM evaluation:
  ros2 launch slam_launch slam_toolbox_eval.launch.py output_dir:=/path/to/results

  # Optionally, follow a trajectory:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy

  # When done, Ctrl+C to save trajectory files
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

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
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')

    # === slam_toolbox via its official launch file ===
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_toolbox_config,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # === Trajectory exporter node ===
    traj_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='traj_exporter',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'sample_rate': 20.0,
            'gt_parent_frame': 'map_gt',
            'gt_child_frame': 'base_footprint',
            'est_parent_frame': 'map',
            'est_child_frame': 'base_footprint',
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        output_dir_arg,
        use_sim_time_arg,
        slam_toolbox_launch,
        traj_exporter_node,
    ])
