"""
Real-time SLAM evaluation launch file.

Runs Gazebo simulation, SLAM algorithm, and trajectory export simultaneously.
No rosbag recording - everything runs in real-time and evaluates immediately.

Timing sequence:
  t=0s: Gazebo simulation + GT publisher
  t=2s: SLAM algorithm (slam_toolbox or cartographer)
  t=4s: Trajectory exporters (GT and EST)
  t=6s: Trajectory follower (starts robot movement)
  t=7s: Completion monitor (watches for trajectory_done)

Usage:
  ros2 launch experiment_runner run_realtime.launch.py \
    trajectory:=traj_01_easy.csv algorithm:=slam_toolbox

  ros2 launch experiment_runner run_realtime.launch.py \
    trajectory:=traj_02_loop.csv algorithm:=cartographer condition:=degraded
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_rover_control = get_package_share_directory('rover_control')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_slam_launch = get_package_share_directory('slam_launch')

    # Default paths
    default_output_dir = os.path.expanduser('~/thesis/ros2_ws/results/realtime')

    # ========================
    # Launch Arguments
    # ========================

    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='slam_toolbox',
        description='SLAM algorithm: slam_toolbox or cartographer'
    )

    condition_arg = DeclareLaunchArgument(
        'condition',
        default_value='baseline',
        description='Experiment condition: baseline or degraded (adds odometry noise)'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Output directory for trajectory files'
    )

    gt_output_file_arg = DeclareLaunchArgument(
        'gt_output_file',
        default_value='/tmp/gt_trajectory.tum',
        description='Output file for ground truth trajectory'
    )

    est_output_file_arg = DeclareLaunchArgument(
        'est_output_file',
        default_value='/tmp/est_trajectory.tum',
        description='Output file for estimated trajectory'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple.sdf',
        description='Gazebo world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    post_trajectory_wait_arg = DeclareLaunchArgument(
        'post_trajectory_wait',
        default_value='5.0',
        description='Seconds to wait after trajectory completion for SLAM to settle'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='600.0',
        description='Maximum experiment duration in seconds'
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
    # Stage 0: Simulation (t=0s)
    # ========================

    # Gazebo simulation with rover
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_sim, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Ground truth publisher
    gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gt_publisher, 'launch', 'gt_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ========================
    # Stage 1: SLAM Algorithm (t=2s)
    # ========================

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'slam_toolbox'"])
        ),
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-configuration_directory', os.path.join(pkg_slam_launch, 'config'),
            '-configuration_basename', 'cartographer_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'cartographer'"])
        ),
    )

    # Cartographer occupancy grid node
    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'cartographer'"])
        ),
    )

    # Delayed SLAM launch
    slam_launch = TimerAction(
        period=2.0,
        actions=[
            slam_toolbox_node,
            cartographer_node,
            cartographer_grid_node,
        ]
    )

    # ========================
    # Stage 2: Trajectory Exporters (t=4s)
    # ========================

    # Ground truth exporter (odom -> base_footprint)
    gt_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='gt_exporter',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('gt_output_file'),
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'sample_rate': 10.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Estimated trajectory exporter (map -> base_footprint)
    est_exporter_node = Node(
        package='traj_exporter',
        executable='traj_exporter',
        name='est_exporter',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('est_output_file'),
            'parent_frame': 'map',
            'child_frame': 'base_footprint',
            'sample_rate': 10.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    exporters_launch = TimerAction(
        period=4.0,
        actions=[
            gt_exporter_node,
            est_exporter_node,
        ]
    )

    # ========================
    # Stage 3: Trajectory Follower (t=6s)
    # ========================

    trajectory_launch = TimerAction(
        period=6.0,
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
    # Stage 4: Completion Monitor (t=7s)
    # ========================

    completion_monitor_node = Node(
        package='experiment_runner',
        executable='completion_monitor',
        name='completion_monitor',
        output='screen',
        parameters=[{
            'post_trajectory_wait': LaunchConfiguration('post_trajectory_wait'),
            'timeout': LaunchConfiguration('timeout'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    completion_launch = TimerAction(
        period=7.0,
        actions=[completion_monitor_node]
    )

    # ========================
    # Odometry Noise (Degraded Condition)
    # ========================

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
    # Info Messages
    # ========================

    log_info = LogInfo(
        msg=['Real-time SLAM evaluation: ',
             LaunchConfiguration('trajectory'), ' / ',
             LaunchConfiguration('algorithm'), ' / ',
             LaunchConfiguration('condition')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        trajectory_arg,
        algorithm_arg,
        condition_arg,
        output_dir_arg,
        gt_output_file_arg,
        est_output_file_arg,
        world_arg,
        use_sim_time_arg,
        post_trajectory_wait_arg,
        timeout_arg,
        lookahead_arg,
        speed_scale_arg,

        # Info
        log_info,

        # Stage 0: Simulation (immediate)
        sim_launch,
        gt_launch,

        # Odometry noise (conditional, immediate)
        odom_noise_node,

        # Stage 1: SLAM (t=2s)
        slam_launch,

        # Stage 2: Exporters (t=4s)
        exporters_launch,

        # Stage 3: Trajectory follower (t=6s)
        trajectory_launch,

        # Stage 4: Completion monitor (t=7s)
        completion_launch,
    ])
