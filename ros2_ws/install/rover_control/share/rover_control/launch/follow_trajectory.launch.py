"""
Trajectory follower launch file.

Launches the trajectory_follower node to make the rover follow a predefined path.
Expects the simulation (rover_sim) to already be running.

Usage:
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=/full/path/to/traj.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Default trajectory directory
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # Launch arguments
    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Pure pursuit lookahead distance (m)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.2',
        description='Waypoint reached threshold (m)'
    )

    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale',
        default_value='1.0',
        description='Speed multiplier'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from /clock'
    )

    # Build trajectory path: if it starts with '/', use as-is, otherwise prepend default dir
    trajectory_file = PythonExpression([
        "'", LaunchConfiguration('trajectory'), "' if '", LaunchConfiguration('trajectory'),
        "'.startswith('/') else '", default_traj_dir, "/' + '", LaunchConfiguration('trajectory'), "'"
    ])

    # Trajectory follower node
    trajectory_follower_node = Node(
        package='rover_control',
        executable='trajectory_follower',
        name='trajectory_follower',
        parameters=[{
            'trajectory_file': trajectory_file,
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'speed_scale': LaunchConfiguration('speed_scale'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    return LaunchDescription([
        trajectory_arg,
        lookahead_arg,
        goal_tolerance_arg,
        speed_scale_arg,
        use_sim_time_arg,
        trajectory_follower_node,
    ])
