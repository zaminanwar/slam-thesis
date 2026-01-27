"""
Gazebo Classic simulation launch file for SLAM thesis rover.

Launches:
- Gazebo Classic with simple world
- Robot spawned at origin
- robot_state_publisher for URDF and static TFs

Note: Gazebo Classic plugins publish ROS topics directly (no bridge needed).

Usage:
  ros2 launch rover_sim sim.launch.py
  ros2 launch rover_sim sim.launch.py world:=simple.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_rover_sim = get_package_share_directory('rover_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple.world',
        description='World file name (in rover_sim/worlds/)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Paths
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    world_path = PathJoinSubstitution([
        pkg_rover_sim, 'worlds', LaunchConfiguration('world')
    ])

    # Robot description from xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Set Gazebo model path for custom models
    gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_rover_sim, 'worlds')
    )

    # Gazebo server (headless physics simulation)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
        }.items()
    )

    # Gazebo client (GUI)
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher (publishes URDF and static TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Spawn robot in Gazebo using spawn_entity.py
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,

        # Environment
        gz_model_path,

        # Gazebo
        gz_server,
        gz_client,

        # Robot
        robot_state_publisher,
        spawn_robot,
    ])
