"""
Trajectory export launch file.

Launches the trajectory exporter node to record TF-based poses to a TUM format file.
Can be used standalone or included in SLAM launch files for trajectory evaluation.

Usage:
  ros2 launch traj_exporter export_trajectory.launch.py output_file:=/path/to/traj.txt
  ros2 launch traj_exporter export_trajectory.launch.py parent_frame:=map child_frame:=base_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================
    # Launch Arguments
    # ========================

    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='/tmp/trajectory.txt',
        description='Output file path for TUM format trajectory'
    )

    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='map',
        description='Parent TF frame (typically "map" for SLAM output)'
    )

    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='base_footprint',
        description='Child TF frame (robot base frame)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='10.0',
        description='Transform sampling rate in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (required for bag replay)'
    )

    # ========================
    # Trajectory Exporter Node
    # ========================

    traj_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='traj_exporter',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'parent_frame': LaunchConfiguration('parent_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ========================
    # Info Message
    # ========================

    log_info = LogInfo(
        msg=['Exporting trajectory: ',
             LaunchConfiguration('parent_frame'), ' -> ',
             LaunchConfiguration('child_frame'), ' -> ',
             LaunchConfiguration('output_file')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        output_file_arg,
        parent_frame_arg,
        child_frame_arg,
        sample_rate_arg,
        use_sim_time_arg,

        # Info
        log_info,

        # Node
        traj_exporter_node,
    ])
