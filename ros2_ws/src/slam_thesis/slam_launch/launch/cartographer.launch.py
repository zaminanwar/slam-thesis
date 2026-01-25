"""
Cartographer 2D launch file for SLAM evaluation on bag replay.

Launches Cartographer in 2D mode with bag playback for SLAM evaluation.

Usage:
  # Basic usage with bag name
  ros2 launch slam_launch cartographer.launch.py bag:=traj_01_easy__baseline

  # With full bag path
  ros2 launch slam_launch cartographer.launch.py bag:=/path/to/bag_directory

  # With custom rate
  ros2 launch slam_launch cartographer.launch.py bag:=traj_01_easy__baseline rate:=0.5

Architecture:
  - robot_state_publisher provides static TFs (base_footprint->base_link->laser_frame)
  - Bag playback provides /clock, /tf (odom->base_footprint), /scan, /odom
  - Bag /tf_static is filtered out to avoid conflicts with robot_state_publisher
  - Cartographer uses the combined TF tree for SLAM
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_slam_launch = get_package_share_directory('slam_launch')
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
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')
    cartographer_config_basename = 'cartographer_2d.lua'

    # === Robot description for static TFs ===
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # === robot_state_publisher (starts first - provides static TFs) ===
    # This ensures laser_frame->base_link->base_footprint transforms are always available
    # with current timestamps, avoiding TF lookup failures
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # === Bag playback (delayed - provides /clock and dynamic TF) ===
    # Filter out /tf_static from bag since robot_state_publisher provides it
    bag_play = TimerAction(
        period=1.0,  # Wait for robot_state_publisher to be ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    bag_path,
                    '--clock',  # Publish /clock for use_sim_time
                    '--rate', rate,
                    '--topics', '/tf', '/odom', '/scan', '/clock', '/gt_pose',
                ],
                output='screen',
                shell=False,
            )
        ]
    )

    # === Cartographer node (delayed to allow TF from bag) ===
    cartographer_node = TimerAction(
        period=3.0,
        actions=[
            Node(
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
        ]
    )

    # === Occupancy grid node (publishes /map) ===
    occupancy_grid_node = TimerAction(
        period=4.0,
        actions=[
            Node(
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
        ]
    )

    return LaunchDescription([
        # Arguments
        bag_arg,
        rate_arg,
        use_sim_time_arg,

        # Nodes
        robot_state_publisher,  # First - provides static TFs
        bag_play,               # Second - provides /clock and dynamic data
        cartographer_node,      # Third - Cartographer with TF ready
        occupancy_grid_node,    # Fourth - map publisher
    ])
