"""
RViz display launch file for SLAM thesis rover.

Launches:
- robot_state_publisher with rover URDF
- joint_state_publisher_gui (optional, for standalone visualization)
- RViz2 with rover configuration

Usage:
  ros2 launch rover_description display.launch.py
  ros2 launch rover_description display.launch.py use_sim_time:=true  # When sim is running
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directory
    pkg_rover_description = get_package_share_directory('rover_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch RViz GUI'
    )

    jsp_gui_arg = DeclareLaunchArgument(
        'jsp_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui (for standalone URDF viewing)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_rover_description, 'rviz', 'rover.rviz'),
        description='Path to RViz config file'
    )

    # Paths
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')

    # Robot description from xacro (wrapped for Jazzy compatibility)
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Joint state publisher GUI (only when jsp_gui:=true, for standalone viewing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint state publisher (when no GUI, just publishes fixed joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        gui_arg,
        jsp_gui_arg,
        rviz_config_arg,

        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
