"""
Launch file for trajectory exporter node.

Exports TF transforms to TUM format files for evaluation with evo tool.

Arguments:
    output_dir: Directory to write gt.tum and est.tum (required)
    sample_rate: Sampling rate in Hz (default: 20.0)
    gt_parent_frame: Ground truth parent frame (default: map_gt)
    gt_child_frame: Ground truth child frame (default: base_footprint)
    est_parent_frame: Estimate parent frame (default: map)
    est_child_frame: Estimate child frame (default: base_footprint)
    use_sim_time: Use simulation time (default: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        description='Directory to write trajectory files (required)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='20.0',
        description='Sampling rate in Hz'
    )

    gt_parent_frame_arg = DeclareLaunchArgument(
        'gt_parent_frame',
        default_value='map_gt',
        description='Ground truth TF parent frame'
    )

    gt_child_frame_arg = DeclareLaunchArgument(
        'gt_child_frame',
        default_value='base_footprint',
        description='Ground truth TF child frame'
    )

    est_parent_frame_arg = DeclareLaunchArgument(
        'est_parent_frame',
        default_value='map',
        description='SLAM estimate TF parent frame'
    )

    est_child_frame_arg = DeclareLaunchArgument(
        'est_child_frame',
        default_value='base_footprint',
        description='SLAM estimate TF child frame'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Trajectory exporter node
    traj_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='traj_exporter',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'gt_parent_frame': LaunchConfiguration('gt_parent_frame'),
            'gt_child_frame': LaunchConfiguration('gt_child_frame'),
            'est_parent_frame': LaunchConfiguration('est_parent_frame'),
            'est_child_frame': LaunchConfiguration('est_child_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        output_dir_arg,
        sample_rate_arg,
        gt_parent_frame_arg,
        gt_child_frame_arg,
        est_parent_frame_arg,
        est_child_frame_arg,
        use_sim_time_arg,
        traj_exporter_node,
    ])
