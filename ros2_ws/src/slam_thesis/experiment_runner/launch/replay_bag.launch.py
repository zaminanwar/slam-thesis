"""
Bag replay launch file for SLAM evaluation.

Replays a recorded rosbag with simulation time for SLAM algorithm testing.
This is used by the SLAM launch files (M5/M6) to provide sensor data.

Usage:
  # Basic replay
  ros2 launch experiment_runner replay_bag.launch.py bag:=traj_01_easy__baseline

  # Full path
  ros2 launch experiment_runner replay_bag.launch.py bag:=/path/to/bag_directory

  # With rate multiplier
  ros2 launch experiment_runner replay_bag.launch.py bag:=traj_01_easy__baseline rate:=0.5

Notes:
  - Uses --clock to publish /clock topic for use_sim_time
  - Publishes static transforms via robot_state_publisher
  - Does NOT start SLAM - that's done by slam_launch package
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_rover_description = get_package_share_directory('rover_description')

    # Default directories
    default_bag_dir = os.path.expanduser('~/thesis/ros2_ws/bags')

    # === Launch arguments ===
    bag_arg = DeclareLaunchArgument(
        'bag',
        description='Bag directory name or full path (e.g., traj_01_easy__baseline)'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (e.g., 0.5 for half speed)'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop playback continuously'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag'
    )

    # === Derived values ===
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    loop = LaunchConfiguration('loop')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build bag path: if it starts with '/', use as-is, otherwise prepend default dir
    bag_path = PythonExpression([
        "'", bag, "' if '", bag, "'.startswith('/') else '", default_bag_dir, "/' + '", bag, "'"
    ])

    # === Robot description ===
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # === Robot state publisher (for static TFs) ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # === Bag playback ===
    # Build ros2 bag play command
    bag_play_cmd = [
        'ros2', 'bag', 'play',
        bag_path,
        '--clock',  # Publish /clock for use_sim_time
        '--rate', rate,
    ]

    bag_play = ExecuteProcess(
        cmd=bag_play_cmd,
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        # Arguments
        bag_arg,
        rate_arg,
        loop_arg,
        use_sim_time_arg,

        # Nodes
        robot_state_publisher,
        bag_play,
    ])
