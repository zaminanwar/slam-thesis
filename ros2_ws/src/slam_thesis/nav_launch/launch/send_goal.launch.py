"""
Utility launch file to send a single navigation goal.

Usage:
  ros2 launch nav_launch send_goal.launch.py x:=2.0 y:=1.0 yaw:=0.0
  ros2 launch nav_launch send_goal.launch.py x:=3.0 y:=-1.5 yaw:=1.57

The goal is sent to the /navigate_to_pose action server.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========================
    # Launch Arguments
    # ========================

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='2.0',
        description='Goal X position in meters'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Goal Y position in meters'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Goal yaw orientation in radians'
    )

    frame_arg = DeclareLaunchArgument(
        'frame',
        default_value='map',
        description='Reference frame for the goal'
    )

    # ========================
    # Send Goal via Python inline script
    # ========================

    # Inline Python script that takes args and sends goal
    python_script = '''
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
import sys

def main():
    rclpy.init()
    node = Node("send_goal_node")

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])
    frame = sys.argv[4]

    # Convert yaw to quaternion
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    action_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

    node.get_logger().info(f"Waiting for navigate_to_pose action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server not available!")
        rclpy.shutdown()
        return 1

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = frame
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.position.z = 0.0
    goal_msg.pose.pose.orientation.x = 0.0
    goal_msg.pose.pose.orientation.y = 0.0
    goal_msg.pose.pose.orientation.z = qz
    goal_msg.pose.pose.orientation.w = qw

    node.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw} rad in frame {frame}")

    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected!")
        rclpy.shutdown()
        return 1

    node.get_logger().info("Goal accepted, waiting for result...")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()
    node.get_logger().info(f"Navigation complete!")

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
'''

    send_goal_node = ExecuteProcess(
        cmd=[
            'python3', '-c', python_script,
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('frame'),
        ],
        output='screen',
    )

    # ========================
    # Info
    # ========================

    log_info = LogInfo(
        msg=['Sending navigation goal: x=', LaunchConfiguration('x'),
             ', y=', LaunchConfiguration('y'),
             ', yaw=', LaunchConfiguration('yaw'),
             ' in frame ', LaunchConfiguration('frame')]
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        x_arg,
        y_arg,
        yaw_arg,
        frame_arg,

        # Info
        log_info,

        # Send goal
        send_goal_node,
    ])
