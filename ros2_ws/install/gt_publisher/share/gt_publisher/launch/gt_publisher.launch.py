"""
Ground truth publisher launch file.

Launches the gt_publisher node which subscribes to Gazebo model pose
and publishes /gt_pose + TF map_gt->base_link.

Usage:
  ros2 launch gt_publisher gt_publisher.launch.py
  ros2 launch gt_publisher gt_publisher.launch.py model_name:=rover publish_rate:=50.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='rover',
        description='Gazebo model name to track'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Ground truth publisher node
    gt_publisher_node = Node(
        package='gt_publisher',
        executable='gt_publisher',
        name='gt_publisher',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        model_name_arg,
        publish_rate_arg,
        use_sim_time_arg,
        gt_publisher_node,
    ])
