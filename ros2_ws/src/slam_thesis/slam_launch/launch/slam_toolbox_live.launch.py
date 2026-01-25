"""
slam_toolbox launch file for live simulation SLAM evaluation.

This launch file works with a running Gazebo simulation. It does NOT
replay bags - the simulation provides all sensor data and TF in real-time.

Usage:
  # First, start the simulation:
  ros2 launch rover_sim sim.launch.py

  # Then, start SLAM:
  ros2 launch slam_launch slam_toolbox_live.launch.py

  # Optionally, follow a trajectory:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # === Launch arguments ===
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    # === Config paths ===
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')

    # === slam_toolbox via its official launch file ===
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_toolbox_config,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_launch,
    ])
