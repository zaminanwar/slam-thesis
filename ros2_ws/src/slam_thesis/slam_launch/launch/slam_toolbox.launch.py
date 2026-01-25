"""
slam_toolbox launch file for SLAM evaluation on bag replay.

Launches slam_toolbox in mapping mode with bag playback for SLAM evaluation.

Usage:
  # Basic usage with bag name
  ros2 launch slam_launch slam_toolbox.launch.py bag:=traj_01_easy__baseline

  # With full bag path
  ros2 launch slam_launch slam_toolbox.launch.py bag:=/path/to/bag_directory

  # With custom rate
  ros2 launch slam_launch slam_toolbox.launch.py bag:=traj_01_easy__baseline rate:=0.5

Architecture:
  - robot_state_publisher continuously publishes static TFs (base_footprint->base_link->laser_frame)
  - Bag playback provides /clock, /tf (odom->base_footprint), /scan, /odom
  - Bag /tf_static is excluded (robot_state_publisher provides it)
  - slam_toolbox uses the combined TF tree for SLAM

Note: Static transforms from bag are only published once. Nodes starting later miss them.
      robot_state_publisher solves this by continuously republishing static TFs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
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
        description='Playback rate multiplier'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag'
    )

    # === Derived values ===
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build bag path: if it starts with '/', use as-is, otherwise prepend default dir
    bag_path = PythonExpression([
        "'", bag, "' if '", bag, "'.startswith('/') else '", default_bag_dir, "/' + '", bag, "'"
    ])

    # === Config paths ===
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')

    # === Robot description for static TFs ===
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # === Bag playback (delayed - provides /clock and dynamic TF) ===
    # Delayed to ensure robot_state_publisher has published static TFs
    # Exclude /tf_static since robot_state_publisher provides it
    bag_play = TimerAction(
        period=1.0,  # Wait for robot_state_publisher
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    bag_path,
                    '--clock',  # Publish /clock for use_sim_time
                    '--rate', rate,
                    '--topics', '/tf', '/odom', '/scan', '/gt_pose',  # Exclude /tf_static
                ],
                output='screen',
                shell=False,
            )
        ]
    )

    # === robot_state_publisher (provides static TFs) ===
    # Starts FIRST with wall time - static TFs are valid for all time in TF2
    # Using use_sim_time=false avoids TF timestamp synchronization issues
    # Static transforms have special handling in TF2 and work regardless of timestamp
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},  # Static TFs valid for all time
        ],
        output='screen',
    )

    # === slam_toolbox via its official launch file ===
    # Delayed to allow TF tree to establish (static + dynamic from bag)
    slam_toolbox_launch = TimerAction(
        period=4.0,  # Give bag time to publish /clock and dynamic TF
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_toolbox_config,
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        bag_arg,
        rate_arg,
        use_sim_time_arg,

        # Nodes - order matters!
        robot_state_publisher,  # First - provides static TFs (wall time, always valid)
        bag_play,               # Second - provides /clock and dynamic TF
        slam_toolbox_launch,    # Third - SLAM
    ])
