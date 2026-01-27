"""
Live SLAM launch file - runs simulation + SLAM + trajectory export in real-time.

This is an alternative to the rosbag record/replay workflow. It runs everything
live without intermediate bag storage.

Architecture:
  Gazebo Sim -> /scan, /odom -> SLAM -> map->odom TF
       |                          |
       v                          v
  GT Publisher             Traj Exporter
       |                          |
       v                          v
  /gt_pose              gt_trajectory.tum
  map_gt->base_link     est_trajectory.tum

Usage:
  ros2 launch slam_launch live_slam.launch.py algorithm:=slam_toolbox trajectory:=traj_01_easy.csv
  ros2 launch slam_launch live_slam.launch.py algorithm:=cartographer trajectory:=traj_02_loop.csv
  ros2 launch slam_launch live_slam.launch.py algorithm:=slam_toolbox output_dir:=/path/to/results
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    RegisterEventHandler,
    EmitEvent,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_rover_control = get_package_share_directory('rover_control')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_rover_description = get_package_share_directory('rover_description')

    # Default paths
    default_output_dir = os.path.expanduser('~/thesis/ros2_ws/results')
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # SLAM configuration files
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')

    # URDF for robot_state_publisher (needed for Cartographer)
    urdf_file = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ========================
    # Launch Arguments
    # ========================

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='slam_toolbox',
        description='SLAM algorithm: slam_toolbox or cartographer',
        choices=['slam_toolbox', 'cartographer']
    )

    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory to save trajectory outputs'
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

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='10.0',
        description='Trajectory export sample rate (Hz)'
    )

    # ========================
    # Derived Values
    # ========================

    # Build trajectory path
    trajectory_file = PythonExpression([
        "'", LaunchConfiguration('trajectory'), "' if '", LaunchConfiguration('trajectory'),
        "'.startswith('/') else '", default_traj_dir, "/' + '", LaunchConfiguration('trajectory'), "'"
    ])

    # Output file paths
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_name = PythonExpression([
        "'", LaunchConfiguration('trajectory'),
        "'.replace('.csv', '') + '_' + '", LaunchConfiguration('algorithm'), "' + '_", timestamp, "'"
    ])

    gt_output_file = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        run_name,
        'gt_trajectory.tum'
    ])

    est_output_file = PathJoinSubstitution([
        LaunchConfiguration('output_dir'),
        run_name,
        'est_trajectory.tum'
    ])

    # ========================
    # 1. Simulation Launch
    # ========================

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_sim, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ========================
    # 2. Ground Truth Publisher
    # ========================

    gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gt_publisher, 'launch', 'gt_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ========================
    # 3. Trajectory Follower (delayed)
    # ========================

    trajectory_launch = TimerAction(
        period=5.0,  # Wait for Gazebo to initialize
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
    # 4. SLAM Nodes
    # ========================

    # --- slam_toolbox (lifecycle node) ---
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(slam_toolbox_config, allow_substs=False),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'slam_toolbox'"])
        ),
    )

    # Lifecycle management for slam_toolbox
    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'slam_toolbox'"])
        ),
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='slam_toolbox configured, activating...'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'slam_toolbox'"])
        ),
    )

    # --- Cartographer nodes ---
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

    cartographer_occupancy_grid = Node(
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

    # Robot state publisher for Cartographer (provides static TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_cartographer',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},  # Static TFs use timestamp 0
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'cartographer'"])
        ),
    )

    # ========================
    # 5. Trajectory Exporters (delayed to let SLAM initialize)
    # ========================

    trajectory_exporters = TimerAction(
        period=6.0,  # Start after SLAM has initialized
        actions=[
            # Ground truth exporter (odom -> base_footprint)
            Node(
                package='traj_exporter',
                executable='traj_exporter',
                name='gt_traj_exporter',
                output='screen',
                parameters=[{
                    'output_file': gt_output_file,
                    'parent_frame': 'odom',
                    'child_frame': 'base_footprint',
                    'sample_rate': LaunchConfiguration('sample_rate'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
            # SLAM estimated trajectory exporter (map -> base_footprint)
            Node(
                package='traj_exporter',
                executable='traj_exporter',
                name='est_traj_exporter',
                output='screen',
                parameters=[{
                    'output_file': est_output_file,
                    'parent_frame': 'map',
                    'child_frame': 'base_footprint',
                    'sample_rate': LaunchConfiguration('sample_rate'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
        ]
    )

    # ========================
    # 6. Trajectory Done Monitor
    # ========================

    # Node that monitors /trajectory_done and triggers shutdown
    trajectory_done_monitor = Node(
        package='experiment_runner',
        executable='trajectory_done_monitor',
        name='trajectory_done_monitor',
        output='screen',
        parameters=[{
            'shutdown_delay': 3.0,  # Wait 3s after trajectory done before shutdown
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ========================
    # Info Messages
    # ========================

    log_start = LogInfo(
        msg=['Starting live SLAM experiment with ', LaunchConfiguration('algorithm'),
             ' on trajectory: ', LaunchConfiguration('trajectory')]
    )

    log_output = LogInfo(
        msg=['Output directory: ', LaunchConfiguration('output_dir'), '/', run_name]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        algorithm_arg,
        trajectory_arg,
        output_dir_arg,
        world_arg,
        use_sim_time_arg,
        lookahead_arg,
        speed_scale_arg,
        sample_rate_arg,

        # Info
        log_start,
        log_output,

        # Simulation stack
        sim_launch,
        gt_launch,
        trajectory_launch,

        # SLAM (algorithm-dependent)
        slam_toolbox_node,
        configure_slam_toolbox,
        activate_slam_toolbox,
        cartographer_node,
        cartographer_occupancy_grid,
        robot_state_publisher_node,

        # Trajectory export
        trajectory_exporters,

        # Done monitor (triggers shutdown)
        trajectory_done_monitor,
    ])
