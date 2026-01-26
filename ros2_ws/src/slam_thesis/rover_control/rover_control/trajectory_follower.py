#!/usr/bin/env python3
"""
Trajectory follower node using Pure Pursuit algorithm.
Reads CSV waypoints and publishes /cmd_vel to follow the trajectory.

Features:
- Pure Pursuit path tracking with configurable lookahead
- Low-pass filter on angular velocity to prevent oscillations
- Velocity rate limiting to respect acceleration constraints
- Curvature-based speed reduction for sharp corners
- Smooth transition between rotate-in-place and forward motion
- SLAM-corrected pose support via TF lookup
- Higher control loop frequency (50 Hz) to match odometry rate
"""

import csv
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

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

    Subscribes to /odom for current pose (or uses TF when use_slam_pose=True).
    Publishes /cmd_vel for velocity commands.
    Publishes /trajectory_done when trajectory is complete.
    """

    def __init__(self):
        super().__init__('trajectory_follower')

        # Declare parameters - basic
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('lookahead_distance', 0.1)

        # Declare parameters - velocity control
        self.declare_parameter('max_linear_velocity', 0.5)   # m/s
        self.declare_parameter('max_angular_velocity', 0.8)  # rad/s
        self.declare_parameter('max_linear_accel', 0.8)      # m/s^2
        self.declare_parameter('max_angular_accel', 0.8)     # rad/s^2
        self.declare_parameter('max_centripetal_accel', 0.5) # m/s^2 for curvature speed limiting

        # Declare parameters - smoothing
        self.declare_parameter('angular_filter_alpha', 0.4)  # Low-pass filter (0-1, lower = more smoothing)
        self.declare_parameter('use_velocity_smoothing', True)

        # Declare parameters - rotation behavior
        self.declare_parameter('rotate_in_place_threshold', 0.785)  # ~45 degrees in radians
        self.declare_parameter('pure_pursuit_threshold', 0.524)     # ~30 degrees - smooth transition zone

        # Declare parameters - pose source (SLAM-corrected vs raw odometry)
        self.declare_parameter('use_slam_pose', False)  # Use TF lookup instead of /odom
        self.declare_parameter('pose_frame', 'map')     # Frame to get pose from ('map' for SLAM, 'map_gt' for ground truth)

        # Get parameters
        trajectory_file = self.get_parameter('trajectory_file').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.speed_scale = self.get_parameter('speed_scale').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Velocity control parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_centripetal_accel = self.get_parameter('max_centripetal_accel').value

        # Smoothing parameters
        self.angular_filter_alpha = self.get_parameter('angular_filter_alpha').value
        self.use_velocity_smoothing = self.get_parameter('use_velocity_smoothing').value

        # Rotation thresholds
        self.rotate_threshold = self.get_parameter('rotate_in_place_threshold').value
        self.pursuit_threshold = self.get_parameter('pure_pursuit_threshold').value

        # Pose source parameters
        self.use_slam_pose = self.get_parameter('use_slam_pose').value
        self.pose_frame = self.get_parameter('pose_frame').value

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

        # Filtering and smoothing state
        self.prev_angular_vel = 0.0
        self.prev_cmd = Twist()

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

        # Control loop timer - 50 Hz to match odometry rate
        self.control_dt = 0.02  # 50 Hz
        self.control_timer = self.create_timer(self.control_dt, self._control_loop)

        pose_mode = 'SLAM/TF' if self.use_slam_pose else 'odometry'
        self.get_logger().info(
            f'Trajectory follower initialized: lookahead={self.lookahead_distance}m, '
            f'pose_mode={pose_mode}, frame={self.pose_frame}'
        )

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

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _odom_callback(self, msg: Odometry):
        """Store the latest odometry message."""
        self.current_pose = msg

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Get the current robot pose (x, y, yaw).

        When use_slam_pose=True, uses TF lookup (pose_frame -> base_footprint)
        to get SLAM-corrected or ground truth pose.

        When use_slam_pose=False, uses raw /odom messages.

        Returns None if pose is not available.
        """
        if self.use_slam_pose:
            # Use TF lookup for SLAM-corrected or ground truth pose
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.pose_frame,  # 'map' for SLAM, 'map_gt' for ground truth
                    'base_footprint',
                    rclpy.time.Time(),  # Get latest available
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                x = transform.transform.translation.x
                y = transform.transform.translation.y

                # Extract yaw from quaternion
                q = transform.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                return x, y, yaw

            except TransformException as e:
                self.get_logger().warn(
                    f'TF lookup {self.pose_frame}->base_footprint failed: {e}',
                    throttle_duration_sec=2.0
                )
                return None
        else:
            # Use raw odometry
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

    def _find_lookahead_point(self, robot_x: float, robot_y: float) -> Tuple[int, float, float, float]:
        """
        Find the lookahead point on the trajectory path.

        This implementation follows the PATH between waypoints rather than
        jumping directly to distant waypoints. This prevents the robot from
        taking shortcuts through obstacles.

        Returns the waypoint index and the lookahead point (x, y, speed).
        """
        # If we're at the last waypoint, just target it directly
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            last_wp = self.waypoints[-1]
            return len(self.waypoints) - 1, last_wp.x, last_wp.y, last_wp.speed

        # Get current and next waypoint to define the path segment
        current_wp = self.waypoints[self.current_waypoint_idx]
        next_wp = self.waypoints[self.current_waypoint_idx + 1]

        # Vector from current waypoint to next waypoint (path direction)
        path_dx = next_wp.x - current_wp.x
        path_dy = next_wp.y - current_wp.y
        path_length = math.sqrt(path_dx ** 2 + path_dy ** 2)

        if path_length < 0.001:
            # Waypoints are essentially the same, target the next one
            return self.current_waypoint_idx + 1, next_wp.x, next_wp.y, next_wp.speed

        # Normalize path direction
        path_ux = path_dx / path_length
        path_uy = path_dy / path_length

        # Vector from current waypoint to robot
        robot_dx = robot_x - current_wp.x
        robot_dy = robot_y - current_wp.y

        # Project robot position onto the path segment
        projection = robot_dx * path_ux + robot_dy * path_uy

        # Clamp projection to segment bounds [0, path_length]
        projection = max(0.0, min(path_length, projection))

        # Find the lookahead point along the path
        lookahead_along_path = projection + self.lookahead_distance

        # If lookahead extends beyond this segment, clamp to next waypoint
        if lookahead_along_path >= path_length:
            return self.current_waypoint_idx + 1, next_wp.x, next_wp.y, next_wp.speed

        # Interpolate the lookahead point along the path segment
        lookahead_x = current_wp.x + lookahead_along_path * path_ux
        lookahead_y = current_wp.y + lookahead_along_path * path_uy

        # Interpolate speed between waypoints
        t = lookahead_along_path / path_length
        lookahead_speed = current_wp.speed + t * (next_wp.speed - current_wp.speed)

        return self.current_waypoint_idx, lookahead_x, lookahead_y, lookahead_speed

    def _calculate_curvature_limited_speed(self, desired_speed: float, curvature: float) -> float:
        """
        Reduce speed based on path curvature to prevent excessive centripetal acceleration.
        v_max = sqrt(a_centripetal / curvature)
        """
        if abs(curvature) < 0.01:  # Essentially straight
            return desired_speed

        # Maximum speed for given curvature based on centripetal acceleration limit
        curvature_limited = math.sqrt(self.max_centripetal_accel / abs(curvature))

        return min(desired_speed, curvature_limited)

    def _apply_angular_filter(self, angular_vel: float) -> float:
        """Apply low-pass filter to angular velocity to prevent oscillations."""
        filtered = (self.angular_filter_alpha * angular_vel +
                   (1.0 - self.angular_filter_alpha) * self.prev_angular_vel)
        self.prev_angular_vel = filtered
        return filtered

    def _smooth_velocity_command(self, cmd: Twist) -> Twist:
        """
        Rate-limit velocity changes to respect acceleration constraints.
        Prevents sudden jumps in velocity that can cause jerky motion.
        """
        if not self.use_velocity_smoothing:
            return cmd

        smoothed = Twist()

        # Rate-limit linear velocity
        linear_diff = cmd.linear.x - self.prev_cmd.linear.x
        max_linear_change = self.max_linear_accel * self.control_dt
        linear_diff = max(-max_linear_change, min(max_linear_change, linear_diff))
        smoothed.linear.x = self.prev_cmd.linear.x + linear_diff

        # Rate-limit angular velocity
        angular_diff = cmd.angular.z - self.prev_cmd.angular.z
        max_angular_change = self.max_angular_accel * self.control_dt
        angular_diff = max(-max_angular_change, min(max_angular_change, angular_diff))
        smoothed.angular.z = self.prev_cmd.angular.z + angular_diff

        self.prev_cmd = smoothed
        return smoothed

    def _pure_pursuit(self, robot_x: float, robot_y: float, robot_yaw: float,
                      target_x: float, target_y: float, speed: float) -> Twist:
        """
        Calculate velocity command using Pure Pursuit algorithm.

        Features:
        - Smooth transition between rotate-in-place and forward motion
        - Curvature-based speed limiting
        - Angular velocity filtering
        - Proper differential drive constraints
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
            return cmd

        # Calculate angle to target in robot frame
        angle_to_target = math.atan2(local_y, local_x)
        abs_angle = abs(angle_to_target)

        # Calculate curvature for speed limiting
        curvature = 2.0 * local_y / (L ** 2) if L > 0.01 else 0.0

        # Apply curvature-based speed limiting
        limited_speed = self._calculate_curvature_limited_speed(speed * self.speed_scale, curvature)
        limited_speed = min(limited_speed, self.max_linear_vel)

        # Three-zone behavior with smooth transitions:
        # Zone 1: |angle| > rotate_threshold -> Pure rotation (no forward motion)
        # Zone 2: pursuit_threshold < |angle| < rotate_threshold -> Blended motion
        # Zone 3: |angle| < pursuit_threshold -> Pure pursuit

        if abs_angle > self.rotate_threshold:
            # Zone 1: Pure rotation - target is too far to the side
            cmd.linear.x = 0.0
            raw_angular = self.max_angular_vel if angle_to_target > 0 else -self.max_angular_vel
            cmd.angular.z = self._apply_angular_filter(raw_angular)

        elif abs_angle < self.pursuit_threshold:
            # Zone 3: Pure pursuit - target is ahead
            # Reduce speed slightly when turning
            turn_factor = 1.0 - (abs_angle / self.pursuit_threshold) * 0.3
            linear_vel = limited_speed * turn_factor

            angular_vel = linear_vel * curvature
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
            angular_vel = self._apply_angular_filter(angular_vel)

            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel

        else:
            # Zone 2: Blended region - smooth transition
            blend_range = self.rotate_threshold - self.pursuit_threshold
            blend_factor = (self.rotate_threshold - abs_angle) / blend_range

            # Pure pursuit component
            pp_angular = limited_speed * curvature
            pp_angular = max(-self.max_angular_vel, min(self.max_angular_vel, pp_angular))

            # Rotation component
            rotate_angular = self.max_angular_vel if angle_to_target > 0 else -self.max_angular_vel

            # Blend
            linear_vel = limited_speed * blend_factor
            angular_vel = blend_factor * pp_angular + (1.0 - blend_factor) * rotate_angular
            angular_vel = self._apply_angular_filter(angular_vel)

            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel

        return cmd

    def _control_loop(self):
        """Main control loop, runs at 50 Hz."""
        if self.trajectory_done:
            # Ensure robot is stopped
            self.cmd_vel_pub.publish(Twist())
            return

        # Get current pose
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn('No pose available yet', throttle_duration_sec=2.0)
            return

        robot_x, robot_y, robot_yaw = pose

        # Check if we've reached the NEXT waypoint (end of current segment)
        next_waypoint_idx = self.current_waypoint_idx + 1
        if next_waypoint_idx < len(self.waypoints):
            next_wp = self.waypoints[next_waypoint_idx]
            dist_to_next = self._distance_to_waypoint(robot_x, robot_y, next_wp)

            if dist_to_next < self.goal_tolerance:
                self.current_waypoint_idx += 1
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} '
                    f'at ({next_wp.x:.2f}, {next_wp.y:.2f})'
                )

                if self.current_waypoint_idx >= len(self.waypoints) - 1:
                    self.get_logger().info('Trajectory complete!')
                    self.trajectory_done = True
                    self._publish_done(True)
                    self.cmd_vel_pub.publish(Twist())
                    return
        else:
            last_wp = self.waypoints[-1]
            dist_to_last = self._distance_to_waypoint(robot_x, robot_y, last_wp)
            if dist_to_last < self.goal_tolerance:
                self.get_logger().info('Trajectory complete!')
                self.trajectory_done = True
                self._publish_done(True)
                self.cmd_vel_pub.publish(Twist())
                return

        # Find lookahead point
        _, target_x, target_y, speed = self._find_lookahead_point(robot_x, robot_y)

        # Debug logging (throttled)
        self.get_logger().info(
            f'Robot ({robot_x:.2f}, {robot_y:.2f}) -> Target ({target_x:.2f}, {target_y:.2f}), '
            f'LA={self.lookahead_distance:.2f}m',
            throttle_duration_sec=1.0
        )

        # Calculate velocity command
        cmd = self._pure_pursuit(robot_x, robot_y, robot_yaw, target_x, target_y, speed)

        # Apply velocity smoothing
        cmd = self._smooth_velocity_command(cmd)

        # Publish
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
