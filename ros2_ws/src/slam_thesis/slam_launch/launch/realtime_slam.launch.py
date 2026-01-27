"""
Real-time SLAM launch file - runs simulation + SLAM + bag recording.

Records all relevant topics for post-hoc trajectory extraction.
This avoids real-time TF lookup issues while still evaluating
SLAM in real-time operation.

Usage:
  ros2 launch slam_launch realtime_slam.launch.py algorithm:=slam_toolbox trajectory:=traj_01_easy.csv
  ros2 launch slam_launch realtime_slam.launch.py algorithm:=cartographer trajectory:=traj_02_loop.csv
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
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_rover_control = get_package_share_directory('rover_control')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_rover_description = get_package_share_directory('rover_description')

    # Default paths
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')

    # SLAM configuration files
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')

    # URDF for robot_state_publisher
    urdf_file = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ========================
    # Launch Arguments
    # ========================

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'cartographer']
    )

    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='traj_01_easy.csv',
        description='Trajectory CSV filename or full path'
    )

    bag_output_arg = DeclareLaunchArgument(
        'bag_output',
        default_value='/tmp/slam_recording',
        description='Output path for rosbag recording'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple.sdf'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    # ========================
    # Build trajectory path
    # ========================

    trajectory_file = PythonExpression([
        "'", LaunchConfiguration('trajectory'), "' if '", LaunchConfiguration('trajectory'),
        "'.startswith('/') else '", default_traj_dir, "/' + '", LaunchConfiguration('trajectory'), "'"
    ])

    # ========================
    # 1. Simulation
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
    # 3. SLAM Nodes
    # ========================

    # slam_toolbox (async mode for real-time)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            ParameterFile(slam_toolbox_config, allow_substs=False),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'slam_toolbox'"])
        ),
    )

    # Cartographer
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_2d.lua',
        ],
        remappings=[('scan', '/scan'), ('odom', '/odom')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'cartographer'"])
        ),
    )

    cartographer_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'cartographer'"])
        ),
    )

    # ========================
    # 4. Trajectory Follower (delayed)
    # ========================

    trajectory_follower = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_control, 'launch', 'follow_trajectory.launch.py')
                ),
                launch_arguments={
                    'trajectory': LaunchConfiguration('trajectory'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    # ========================
    # 5. Bag Recording (delayed to let everything start)
    # ========================

    bag_record_cmd = [
        'ros2', 'bag', 'record',
        '-o', LaunchConfiguration('bag_output'),
        '/tf', '/tf_static',
        '/gt_pose',
        '/odom',
        '/scan',
        '/clock',
        '--use-sim-time',
    ]

    bag_recorder = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=bag_record_cmd,
                output='screen',
                name='bag_recorder',
            )
        ]
    )

    # ========================
    # 6. Trajectory Done Monitor
    # ========================

    trajectory_done_monitor = Node(
        package='experiment_runner',
        executable='trajectory_done_monitor',
        name='trajectory_done_monitor',
        output='screen',
        parameters=[{
            'shutdown_delay': 5.0,  # Wait 5s after trajectory done for final TFs
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ========================
    # Info
    # ========================

    log_start = LogInfo(
        msg=['Real-time SLAM: ', LaunchConfiguration('algorithm'),
             ' on ', LaunchConfiguration('trajectory'),
             ' -> ', LaunchConfiguration('bag_output')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        algorithm_arg,
        trajectory_arg,
        bag_output_arg,
        world_arg,
        use_sim_time_arg,

        # Info
        log_start,

        # Simulation
        sim_launch,
        gt_launch,

        # SLAM
        slam_toolbox_node,
        cartographer_node,
        cartographer_occupancy_grid,

        # Trajectory follower
        trajectory_follower,

        # Bag recording
        bag_recorder,

        # Done monitor
        trajectory_done_monitor,
    ])
