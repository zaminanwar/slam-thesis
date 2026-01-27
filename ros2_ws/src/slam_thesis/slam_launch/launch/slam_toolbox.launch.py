"""
slam_toolbox launch file for offline SLAM evaluation with bag replay.

Replays a recorded rosbag and runs slam_toolbox for mapping/SLAM evaluation.
Publishes map and SLAM-corrected pose (map->odom transform) for evaluation.

Usage:
  ros2 launch slam_launch slam_toolbox.launch.py bag_path:=/path/to/bag
  ros2 launch slam_launch slam_toolbox.launch.py bag_path:=/path/to/bag rate:=0.5
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
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')

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

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml'),
        description='Path to slam_toolbox configuration file'
    )

    traj_output_arg = DeclareLaunchArgument(
        'traj_output',
        default_value='',
        description='Output file for SLAM trajectory (empty = no export)'
    )

    # ========================
    # slam_toolbox Node
    # ========================

    # slam_toolbox async SLAM node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
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

    # Delayed bag playback to let SLAM node initialize
    rosbag_play_once = TimerAction(
        period=2.0,  # Wait for slam_toolbox to initialize
        actions=[rosbag_play_process],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('loop'), "' == 'false'"])
        )
    )

    rosbag_play_loop = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=rosbag_play_cmd + ['--loop'],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('loop'))
    )

    # ========================
    # TF Recording (for offline trajectory extraction)
    # ========================

    # Record /tf during SLAM for later trajectory extraction
    tf_record_cmd = [
        'ros2', 'bag', 'record',
        '-o', LaunchConfiguration('traj_output'),
        '/tf', '/tf_static', '/clock',
        '--use-sim-time',
    ]

    tf_recorder = TimerAction(
        period=2.5,  # Start just before bag playback
        actions=[
            ExecuteProcess(
                cmd=tf_record_cmd,
                output='screen',
                name='tf_recorder',
            )
        ],
        condition=IfCondition(
            PythonExpression(["len('", LaunchConfiguration('traj_output'), "') > 0"])
        ),
    )

    # ========================
    # Info Message
    # ========================

    log_info = LogInfo(
        msg=['Running slam_toolbox with bag: ', LaunchConfiguration('bag_path'),
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
        slam_config_arg,
        traj_output_arg,

        # Info
        log_info,

        # SLAM node (starts first)
        slam_toolbox_node,

        # TF recorder (optional, for trajectory extraction)
        tf_recorder,

        # Event handler for shutdown on bag completion
        shutdown_on_bag_exit,

        # Bag playback (delayed)
        rosbag_play_once,
        rosbag_play_loop,
    ])
