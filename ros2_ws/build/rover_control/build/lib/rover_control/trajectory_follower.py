#!/usr/bin/env python3
"""
Trajectory follower node using Pure Pursuit algorithm.
Reads CSV waypoints and publishes /cmd_vel to follow the trajectory.
"""

import csv
import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import tf2_ros
from tf2_ros import TransformException


@dataclass
class Waypoint:
    """A single waypoint from the trajectory CSV."""
    x: float
    y: float
    yaw: float
    speed: float


class TrajectoryFollower(Node):
    """
    Pure Pursuit trajectory follower node.

    Subscribes to /odom for current pose.
    Publishes /cmd_vel for velocity commands.
    Publishes /trajectory_done when trajectory is complete.
    """

    def __init__(self):
        super().__init__('trajectory_follower')

        # Declare parameters
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('speed_scale', 1.0)
        # Note: use_sim_time is automatically handled by ROS 2 when passed via launch

        # Get parameters
        trajectory_file = self.get_parameter('trajectory_file').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.speed_scale = self.get_parameter('speed_scale').value

        # Load trajectory
        if not trajectory_file:
            self.get_logger().error('No trajectory_file parameter provided!')
            raise ValueError('trajectory_file parameter is required')

        self.waypoints = self._load_trajectory(trajectory_file)
        if not self.waypoints:
            self.get_logger().error(f'Failed to load trajectory from {trajectory_file}')
            raise ValueError(f'Could not load trajectory from {trajectory_file}')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {trajectory_file}')

        # State
        self.current_waypoint_idx = 0
        self.trajectory_done = False
        self.current_pose: Optional[Odometry] = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Transient local QoS for trajectory_done (latched)
        done_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.done_pub = self.create_publisher(Bool, '/trajectory_done', done_qos)

        # Publish initial done=False
        self._publish_done(False)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # TF buffer for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Control loop timer (20 Hz per INTERFACES.md)
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Trajectory follower initialized')

    def _load_trajectory(self, filepath: str) -> List[Waypoint]:
        """Load waypoints from a CSV file."""
        waypoints = []
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    wp = Waypoint(
                        x=float(row['x']),
                        y=float(row['y']),
                        yaw=float(row['yaw']),
                        speed=float(row['speed'])
                    )
                    waypoints.append(wp)
        except Exception as e:
            self.get_logger().error(f'Error loading trajectory: {e}')
            return []
        return waypoints

    def _odom_callback(self, msg: Odometry):
        """Store the latest odometry message."""
        self.current_pose = msg

    def _get_robot_pose(self):
        """
        Get the current robot pose (x, y, yaw) from odometry.
        Returns None if pose is not available.
        """
        if self.current_pose is None:
            return None

        pose = self.current_pose.pose.pose
        x = pose.position.x
        y = pose.position.y

        # Extract yaw from quaternion
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return x, y, yaw

    def _distance_to_waypoint(self, robot_x: float, robot_y: float, wp: Waypoint) -> float:
        """Calculate Euclidean distance from robot to waypoint."""
        return math.sqrt((wp.x - robot_x) ** 2 + (wp.y - robot_y) ** 2)

    def _find_lookahead_point(self, robot_x: float, robot_y: float):
        """
        Find the lookahead point on the trajectory.
        Returns the waypoint index and the lookahead point (x, y, speed).
        """
        # Start from current waypoint
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = self._distance_to_waypoint(robot_x, robot_y, wp)

            # If this waypoint is beyond lookahead distance, use it
            if dist >= self.lookahead_distance:
                return i, wp.x, wp.y, wp.speed

        # If no point beyond lookahead, use the last waypoint
        last_wp = self.waypoints[-1]
        return len(self.waypoints) - 1, last_wp.x, last_wp.y, last_wp.speed

    def _pure_pursuit(self, robot_x: float, robot_y: float, robot_yaw: float,
                      target_x: float, target_y: float, speed: float) -> Twist:
        """
        Calculate velocity command using Pure Pursuit algorithm.

        Pure Pursuit computes the curvature needed to reach the lookahead point
        and converts it to linear and angular velocity.
        """
        cmd = Twist()

        # Transform target to robot frame
        dx = target_x - robot_x
        dy = target_y - robot_y

        # Rotate to robot frame
        local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)

        # Distance to target
        L = math.sqrt(local_x ** 2 + local_y ** 2)

        if L < 0.01:
            # Too close, stop
            return cmd

        # Pure pursuit curvature: kappa = 2 * y / L^2
        curvature = 2.0 * local_y / (L ** 2)

        # Linear velocity (scaled)
        linear_vel = speed * self.speed_scale

        # Angular velocity = linear * curvature
        angular_vel = linear_vel * curvature

        # Clamp angular velocity
        max_angular = 1.5  # rad/s
        angular_vel = max(-max_angular, min(max_angular, angular_vel))

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        return cmd

    def _control_loop(self):
        """Main control loop, runs at 20 Hz."""
        if self.trajectory_done:
            # Ensure robot is stopped
            self.cmd_vel_pub.publish(Twist())
            return

        # Get current pose
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn('No odometry received yet', throttle_duration_sec=2.0)
            return

        robot_x, robot_y, robot_yaw = pose

        # Check if we've reached the current waypoint
        current_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_current = self._distance_to_waypoint(robot_x, robot_y, current_wp)

        if dist_to_current < self.goal_tolerance:
            # Move to next waypoint
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx}/{len(self.waypoints)}'
            )

            # Check if trajectory is complete
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('Trajectory complete!')
                self.trajectory_done = True
                self._publish_done(True)
                self.cmd_vel_pub.publish(Twist())  # Stop
                return

        # Find lookahead point
        _, target_x, target_y, speed = self._find_lookahead_point(robot_x, robot_y)

        # Calculate and publish velocity command
        cmd = self._pure_pursuit(robot_x, robot_y, robot_yaw, target_x, target_y, speed)
        self.cmd_vel_pub.publish(cmd)

    def _publish_done(self, done: bool):
        """Publish trajectory done status."""
        msg = Bool()
        msg.data = done
        self.done_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrajectoryFollower()
        rclpy.spin(node)
    except ValueError as e:
        print(f'Error: {e}')
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    main()
