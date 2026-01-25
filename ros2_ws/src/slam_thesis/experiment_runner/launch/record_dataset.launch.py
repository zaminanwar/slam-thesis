"""
Dataset recording launch file for SLAM thesis.

Launches the full simulation stack and records a rosbag with all necessary
topics for SLAM evaluation. Supports different experimental conditions.

Usage:
  # Basic recording (baseline condition)
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv

  # With explicit condition
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv condition:=baseline

  # High speed condition
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv condition:=high_speed

  # Degraded odometry condition
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv condition:=odom_degraded

Conditions:
  - baseline: Default settings, clean odometry
  - high_speed: Speed scale 1.5x
  - odom_degraded: Gaussian noise and drift added to odometry
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_rover_control = get_package_share_directory('rover_control')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_experiment_runner = get_package_share_directory('experiment_runner')

    # Default directories
    default_bag_dir = os.path.expanduser('~/thesis/ros2_ws/bags')
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # Ensure bags directory exists
    os.makedirs(default_bag_dir, exist_ok=True)

    # Load topics to record
    topics_file = os.path.join(pkg_experiment_runner, 'config', 'topics_to_record.yaml')
    with open(topics_file, 'r') as f:
        topics_config = yaml.safe_load(f)
    topics_to_record = topics_config.get('topics', [])

    # === Launch arguments ===
    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        description='Trajectory CSV filename (e.g., traj_01_easy.csv)'
    )

    condition_arg = DeclareLaunchArgument(
        'condition',
        default_value='baseline',
        description='Experimental condition: baseline, high_speed, odom_degraded'
    )

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=default_bag_dir,
        description='Directory to store rosbags'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # === Derived values ===
    trajectory = LaunchConfiguration('trajectory')
    condition = LaunchConfiguration('condition')
    bag_dir = LaunchConfiguration('bag_dir')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build trajectory basename (strip .csv extension if present)
    trajectory_basename = PythonExpression([
        "'", trajectory, "'.replace('.csv', '')"
    ])

    # Build bag output path: <bag_dir>/<trajectory_basename>__<condition>
    bag_output_path = PythonExpression([
        "'", bag_dir, "/' + '", trajectory_basename, "' + '__' + '", condition, "'"
    ])

    # Build trajectory file path
    trajectory_file = PythonExpression([
        "'", trajectory, "' if '", trajectory,
        "'.startswith('/') else '", default_traj_dir, "/' + '", trajectory, "'"
    ])

    # Speed scale based on condition
    speed_scale = PythonExpression([
        "'1.5' if '", condition, "' == 'high_speed' else '1.0'"
    ])

    # === Simulation launch ===
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_sim, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # === Ground truth publisher ===
    gt_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gt_publisher, 'launch', 'gt_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # === Trajectory follower (delayed start to allow sim to initialize) ===
    trajectory_launch = TimerAction(
        period=3.0,  # Wait for simulation to stabilize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_control, 'launch', 'follow_trajectory.launch.py')
                ),
                launch_arguments={
                    'trajectory': trajectory_file,
                    'speed_scale': speed_scale,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )

    # === Odometry noise node (only for odom_degraded condition) ===
    odom_noise_node = Node(
        package='experiment_runner',
        executable='odom_noise',
        name='odom_noise',
        parameters=[{
            'noise_std': 0.05,
            'drift_rate': 0.01,
            'use_sim_time': use_sim_time,
        }],
        # Remap so noise node intercepts /odom and republishes as /odom_noisy
        # For degraded condition, SLAM will use /odom_noisy
        remappings=[
            ('odom_in', '/odom'),
            ('odom_out', '/odom_noisy'),
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", condition, "' == 'odom_degraded'"])
        ),
    )

    # === Rosbag recording ===
    # Build rosbag2 record command
    record_args = ['ros2', 'bag', 'record', '-o', bag_output_path, '--use-sim-time']
    record_args.extend(topics_to_record)

    # Add /odom_noisy for degraded condition (record both original and noisy)
    record_args.append('/odom_noisy')

    rosbag_record = TimerAction(
        period=2.0,  # Start recording shortly after sim starts
        actions=[
            ExecuteProcess(
                cmd=record_args,
                output='screen',
                shell=False,
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        trajectory_arg,
        condition_arg,
        bag_dir_arg,
        use_sim_time_arg,

        # Launch components
        sim_launch,
        gt_publisher_launch,
        odom_noise_node,
        trajectory_launch,
        rosbag_record,
    ])
