"""
Nav2 SLAM-in-the-loop launch file.

Launches simulation + SLAM + Nav2 for autonomous navigation experiments.
Nav2 uses SLAM's map->odom transform directly (no AMCL).

Usage:
  ros2 launch nav_launch nav2_slam.launch.py algorithm:=slam_toolbox
  ros2 launch nav_launch nav2_slam.launch.py algorithm:=cartographer world:=obstacles.sdf
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
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # Package directories
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_gt_publisher = get_package_share_directory('gt_publisher')
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_nav_launch = get_package_share_directory('nav_launch')

    # Config files
    slam_toolbox_config = os.path.join(pkg_slam_launch, 'config', 'slam_toolbox.yaml')
    cartographer_config_dir = os.path.join(pkg_slam_launch, 'config')
    nav2_params = os.path.join(pkg_nav_launch, 'config', 'nav2_slam_params.yaml')

    # ========================
    # Launch Arguments
    # ========================

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'cartographer'],
        description='SLAM algorithm to use'
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

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

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

    # slam_toolbox (async mode)
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
    # 4. Nav2 Stack (delayed for SLAM init)
    # ========================

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),

            # Controller Server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params],
                remappings=[
                    ('cmd_vel', '/cmd_vel'),
                ],
            ),

            # Planner Server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params],
            ),

            # Behavior Server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params],
            ),

            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params],
            ),

            # Waypoint Follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params],
            ),

            # Velocity Smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params],
                remappings=[
                    ('cmd_vel', '/cmd_vel'),
                    ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
                ],
            ),

            # Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ],
                    'bond_timeout': 0.0,  # Disable bond for easier debugging
                }],
            ),
        ]
    )

    # Delay Nav2 start by 5 seconds to let SLAM initialize
    nav2_delayed = TimerAction(
        period=5.0,
        actions=[nav2_nodes]
    )

    # ========================
    # Info
    # ========================

    log_start = LogInfo(
        msg=['Nav2 SLAM-in-the-loop: algorithm=', LaunchConfiguration('algorithm'),
             ', world=', LaunchConfiguration('world')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        algorithm_arg,
        world_arg,
        use_sim_time_arg,
        autostart_arg,

        # Info
        log_start,

        # Simulation
        sim_launch,
        gt_launch,

        # SLAM
        slam_toolbox_node,
        cartographer_node,
        cartographer_occupancy_grid,

        # Nav2 (delayed)
        nav2_delayed,
    ])
