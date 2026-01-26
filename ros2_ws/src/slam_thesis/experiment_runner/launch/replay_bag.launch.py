"""
Bag replay launch file for SLAM evaluation.

Replays a recorded rosbag and optionally launches SLAM algorithms for offline evaluation.
This is used to run SLAM on pre-recorded datasets for reproducible experiments.

Usage:
  ros2 launch experiment_runner replay_bag.launch.py bag_path:=/path/to/bag
  ros2 launch experiment_runner replay_bag.launch.py bag_path:=/path/to/bag rate:=0.5
  ros2 launch experiment_runner replay_bag.launch.py bag_path:=/path/to/bag loop:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Default paths
    default_bags_dir = os.path.expanduser('~/thesis/ros2_ws/bags')

    # ========================
    # Launch Arguments
    # ========================

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Full path to the rosbag directory to replay'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (0.5 = half speed, 2.0 = double speed)'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop bag playback'
    )

    start_offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Start playback from this offset (seconds)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag /clock topic'
    )

    # Topics to remap (exclude certain topics from playback)
    exclude_topics_arg = DeclareLaunchArgument(
        'exclude_topics',
        default_value='',
        description='Comma-separated list of topics to exclude from playback'
    )

    # ========================
    # Bag Playback
    # ========================

    # Build rosbag play command
    # Base command
    rosbag_play_cmd = [
        'ros2', 'bag', 'play',
        LaunchConfiguration('bag_path'),
        '--rate', LaunchConfiguration('rate'),
        '--clock',  # Publish /clock from bag timestamps
    ]

    # Rosbag playback process
    rosbag_play = ExecuteProcess(
        cmd=rosbag_play_cmd,
        output='screen',
    )

    # Looped version (separate because ExecuteProcess doesn't support conditional args well)
    rosbag_play_loop = ExecuteProcess(
        cmd=rosbag_play_cmd + ['--loop'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('loop'))
    )

    rosbag_play_once = ExecuteProcess(
        cmd=rosbag_play_cmd,
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('loop'), "' == 'false'"])
        )
    )

    # ========================
    # Static Transform Publisher
    # ========================

    # Ensure required static transforms are available
    # (In case tf_static wasn't properly recorded or needs override)
    # This publishes identity transform from map to odom if needed
    # Note: Usually not needed if tf_static is in the bag

    # ========================
    # Info Message
    # ========================

    log_info = LogInfo(
        msg=['Replaying bag: ', LaunchConfiguration('bag_path'),
             ' at rate: ', LaunchConfiguration('rate')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_offset_arg,
        use_sim_time_arg,
        exclude_topics_arg,

        # Info
        log_info,

        # Bag playback
        rosbag_play_once,
        rosbag_play_loop,
    ])
