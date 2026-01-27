"""
Cartographer 2D launch file for offline SLAM evaluation with bag replay.

Replays a recorded rosbag and runs Cartographer for mapping/SLAM evaluation.
Publishes map and SLAM-corrected pose (map->odom transform) for evaluation.

Usage:
  ros2 launch slam_launch cartographer.launch.py bag_path:=/path/to/bag
  ros2 launch slam_launch cartographer.launch.py bag_path:=/path/to/bag rate:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')

    # Configuration files
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')

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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag /clock topic'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop bag playback (for debugging)'
    )

    config_basename_arg = DeclareLaunchArgument(
        'config_basename',
        default_value='cartographer_2d.lua',
        description='Cartographer configuration file basename'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution for occupancy grid (meters/pixel)'
    )

    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'
    )

    # ========================
    # Cartographer Nodes
    # ========================

    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', LaunchConfiguration('config_basename'),
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # Occupancy grid node (publishes /map)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-resolution', LaunchConfiguration('resolution'),
            '-publish_period_sec', LaunchConfiguration('publish_period_sec'),
        ],
    )

    # ========================
    # Bag Playback
    # ========================

    # Build rosbag play command
    rosbag_play_cmd = [
        'ros2', 'bag', 'play',
        LaunchConfiguration('bag_path'),
        '--rate', LaunchConfiguration('rate'),
        '--clock',  # Publish /clock from bag timestamps
    ]

    # Create bag playback process for non-loop mode
    rosbag_play_process = ExecuteProcess(
        cmd=rosbag_play_cmd,
        output='screen',
        name='rosbag_play',
    )

    # Event handler to shutdown launch when bag playback finishes
    shutdown_on_bag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play_process,
            on_exit=[
                LogInfo(msg='Bag playback finished, shutting down...'),
                EmitEvent(event=ShutdownEvent(reason='Bag playback completed')),
            ],
        )
    )

    # Delayed bag playback to let Cartographer initialize
    rosbag_play_once = TimerAction(
        period=3.0,  # Wait for Cartographer to initialize
        actions=[rosbag_play_process],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('loop'), "' == 'false'"])
        )
    )

    rosbag_play_loop = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=rosbag_play_cmd + ['--loop'],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('loop'))
    )

    # ========================
    # Info Message
    # ========================

    log_info = LogInfo(
        msg=['Running Cartographer with bag: ', LaunchConfiguration('bag_path'),
             ' at rate: ', LaunchConfiguration('rate')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        bag_path_arg,
        rate_arg,
        use_sim_time_arg,
        loop_arg,
        config_basename_arg,
        resolution_arg,
        publish_period_sec_arg,

        # Info
        log_info,

        # Cartographer nodes (start first)
        cartographer_node,
        occupancy_grid_node,

        # Event handler for shutdown on bag completion
        shutdown_on_bag_exit,

        # Bag playback (delayed)
        rosbag_play_once,
        rosbag_play_loop,
    ])
