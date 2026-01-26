"""
Dataset recording launch file for SLAM evaluation.

Launches the full simulation stack and records a rosbag with all topics
needed for offline SLAM evaluation.

Usage:
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv
  ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_02_loop.csv output_dir:=/path/to/bags
  ros2 launch experiment_runner record_dataset.launch.py condition:=degraded  # With odometry noise
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_rover_control = get_package_share_directory('rover_control')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_experiment_runner = get_package_share_directory('experiment_runner')

    # Default paths
    default_output_dir = os.path.expanduser('~/thesis/ros2_ws/bags')
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # ========================
    # Launch Arguments
    # ========================

    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory to save rosbag'
    )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='',
        description='Bag name (auto-generated if empty)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple.sdf',
        description='Gazebo world file'
    )

    condition_arg = DeclareLaunchArgument(
        'condition',
        default_value='baseline',
        description='Experiment condition: baseline or degraded (adds odometry noise)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Trajectory follower parameters
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Pure pursuit lookahead distance (m)'
    )

    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale',
        default_value='1.0',
        description='Speed multiplier for trajectory'
    )

    # ========================
    # Derived Values
    # ========================

    # Build trajectory path
    trajectory_file = PythonExpression([
        "'", LaunchConfiguration('trajectory'), "' if '", LaunchConfiguration('trajectory'),
        "'.startswith('/') else '", default_traj_dir, "/' + '", LaunchConfiguration('trajectory'), "'"
    ])

    # Auto-generate bag name if not provided
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    auto_bag_name = PythonExpression([
        "'", LaunchConfiguration('bag_name'), "' if '", LaunchConfiguration('bag_name'),
        "' else '", LaunchConfiguration('trajectory'),
        "'.replace('.csv', '') + '_' + '", LaunchConfiguration('condition'), "' + '_", timestamp, "'"
    ])

    bag_path = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        auto_bag_name
    ])

    # ========================
    # Included Launch Files
    # ========================

    # 1. Simulation (Gazebo + rover + bridges)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_sim, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 2. Ground truth publisher
    gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gt_publisher, 'launch', 'gt_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 3. Trajectory follower (delayed to let sim initialize)
    trajectory_launch = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_control, 'launch', 'follow_trajectory.launch.py')
                ),
                launch_arguments={
                    'trajectory': LaunchConfiguration('trajectory'),
                    'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                    'speed_scale': LaunchConfiguration('speed_scale'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    # ========================
    # Odometry Noise (Degraded Condition)
    # ========================

    # Odometry noise node for degraded condition
    odom_noise_node = Node(
        package='experiment_runner',
        executable='odom_noise_node',
        name='odom_noise',
        parameters=[{
            'noise_std_linear': 0.02,   # 2cm std dev
            'noise_std_angular': 0.01,  # 0.01 rad std dev
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/odom', '/odom_raw'),
            ('/odom_noisy', '/odom'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('condition'), "' == 'degraded'"])
        ),
        output='screen'
    )

    # ========================
    # Rosbag Recording
    # ========================

    # Topics to record
    topics_to_record = [
        '/scan',
        '/odom',
        '/tf',
        '/tf_static',
        '/gt_pose',
        '/clock',
    ]

    # Start recording (delayed to ensure all nodes are up)
    rosbag_record = TimerAction(
        period=3.0,  # Start recording after 3 seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', bag_path,
                    '--use-sim-time',
                ] + topics_to_record,
                output='screen'
            )
        ]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        trajectory_arg,
        output_dir_arg,
        bag_name_arg,
        world_arg,
        condition_arg,
        use_sim_time_arg,
        lookahead_arg,
        speed_scale_arg,

        # Simulation stack
        sim_launch,
        gt_launch,
        trajectory_launch,

        # Condition-specific nodes
        odom_noise_node,

        # Recording
        rosbag_record,
    ])
