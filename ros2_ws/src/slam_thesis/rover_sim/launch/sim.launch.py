"""
Gazebo Harmonic simulation launch file for SLAM thesis rover.

Launches:
- Gazebo Harmonic (gz-sim) with simple world
- Robot spawned at origin
- ros_gz_bridge for topic bridging (cmd_vel, odom, scan, tf, clock)
- robot_state_publisher for URDF and static TFs

Usage:
  ros2 launch rover_sim sim.launch.py
  ros2 launch rover_sim sim.launch.py world:=simple.sdf
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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple.sdf',
        description='World file name (in rover_sim/worlds/)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Initial robot yaw (radians). 0=East, 1.5708=North, 3.1416=West, -1.5708=South'
    )

    # Paths
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    world_path = PathJoinSubstitution([
        pkg_rover_sim, 'worlds', LaunchConfiguration('world')
    ])

    # Robot description from xacro (wrapped for Jazzy compatibility)
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Set Gazebo resource path for models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_rover_sim, 'worlds')
    )

    # Gazebo Harmonic (gz-sim) launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot in Gazebo (default: facing east/+X, yaw=0)
    # Use spawn_yaw argument to set initial orientation
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen'
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

    # ros_gz_bridge for topic bridging between Gazebo and ROS 2
    # Bridge configuration:
    # - /clock: Gazebo -> ROS (for use_sim_time)
    # - /cmd_vel: ROS -> Gazebo (velocity commands)
    # - /odom: Gazebo -> ROS (odometry)
    # - /scan: Gazebo -> ROS (LiDAR)
    # - /tf: Gazebo -> ROS (transforms)
    # - /joint_states: Gazebo -> ROS (wheel positions)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/rover/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,
        spawn_yaw_arg,

        # Environment
        gz_resource_path,

        # Nodes
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
