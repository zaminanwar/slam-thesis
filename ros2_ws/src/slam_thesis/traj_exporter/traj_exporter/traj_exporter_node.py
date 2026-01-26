#!/usr/bin/env python3
"""
Trajectory exporter node for SLAM thesis evaluation.

Samples TF transforms at a configurable rate and writes TUM format trajectory files:
- gt.tum: Ground truth trajectory (map_gt -> base_footprint_gt)
- est.tum: SLAM estimate trajectory (map -> base_footprint)
- odom_slam_delta.csv: Position/angle delta between raw odometry and SLAM estimate

Note: GT uses base_footprint_gt (NOT base_footprint) to avoid TF tree conflicts
with the main robot tree where Gazebo publishes odom -> base_footprint.

TUM format: timestamp tx ty tz qx qy qz qw
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from nav_msgs.msg import Odometry


class TrajectoryExporter(Node):
    """Exports TF-based trajectories to TUM format files."""

    def __init__(self):
        super().__init__('traj_exporter')

        # Declare parameters
        # Note: GT uses base_footprint_gt to avoid TF conflict with main tree
        self.declare_parameter('gt_parent_frame', 'map_gt')
        self.declare_parameter('gt_child_frame', 'base_footprint_gt')
        self.declare_parameter('est_parent_frame', 'map')
        self.declare_parameter('est_child_frame', 'base_footprint')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('sample_rate', 20.0)

        # Get parameter values
        self.gt_parent = self.get_parameter('gt_parent_frame').get_parameter_value().string_value
        self.gt_child = self.get_parameter('gt_child_frame').get_parameter_value().string_value
        self.est_parent = self.get_parameter('est_parent_frame').get_parameter_value().string_value
        self.est_child = self.get_parameter('est_child_frame').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value

        # Validate output_dir
        if not self.output_dir:
            self.get_logger().error('output_dir parameter is required')
            raise ValueError('output_dir parameter is required')

        # Create output directory if needed
        os.makedirs(self.output_dir, exist_ok=True)

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for trajectory points
        self.gt_poses = []
        self.est_poses = []

        # Storage for odom vs SLAM delta data
        # Format: (timestamp, odom_x, odom_y, odom_yaw, slam_x, slam_y, slam_yaw, pos_delta, angle_delta)
        self.odom_slam_delta = []
        self.latest_odom = None  # Store latest odometry message

        # Track availability
        self.gt_available = False
        self.est_available = False
        self.odom_available = False

        # Create timer for sampling
        timer_period = 1.0 / self.sample_rate
        self.timer = self.create_timer(timer_period, self.sample_transforms)

        # Create timer for periodic saving (every 5 seconds as backup)
        self.save_timer = self.create_timer(5.0, self.periodic_save)
        self.last_save_gt_count = 0
        self.last_save_est_count = 0
        self.last_save_delta_count = 0

        # Subscribe to odometry for odom vs SLAM delta computation
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Register shutdown callback
        self.context.on_shutdown(self.on_shutdown)

        self.get_logger().info(
            f'Trajectory exporter started:\n'
            f'  GT: {self.gt_parent} -> {self.gt_child}\n'
            f'  EST: {self.est_parent} -> {self.est_child}\n'
            f'  Odom vs SLAM delta: enabled\n'
            f'  Output: {self.output_dir}\n'
            f'  Rate: {self.sample_rate} Hz'
        )

    def sample_transforms(self):
        """Sample current TF transforms and store them."""
        # Use Time() (time 0) to get the LATEST available transform
        # instead of get_clock().now() which may be ahead of what's in the buffer.
        # The actual timestamp will be taken from the transform's header.stamp.
        latest = Time()

        # Sample ground truth
        self._sample_transform(
            self.gt_parent, self.gt_child, latest,
            self.gt_poses, 'GT', 'gt_available'
        )

        # Sample SLAM estimate
        self._sample_transform(
            self.est_parent, self.est_child, latest,
            self.est_poses, 'EST', 'est_available'
        )

        # Compute odom vs SLAM delta
        self._compute_odom_slam_delta()

    def _sample_transform(self, parent, child, time, storage, label, avail_attr):
        """Sample a single transform and store it."""
        try:
            # Look up transform with small timeout
            transform = self.tf_buffer.lookup_transform(
                parent, child, time,
                timeout=Duration(seconds=0.1)
            )

            # Extract timestamp (use transform stamp for accuracy)
            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            # Extract translation
            t = transform.transform.translation
            tx, ty, tz = t.x, t.y, t.z

            # Extract rotation (quaternion)
            r = transform.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w

            # Store pose tuple
            storage.append((timestamp, tx, ty, tz, qx, qy, qz, qw))

            # Log first successful lookup
            if not getattr(self, avail_attr):
                setattr(self, avail_attr, True)
                self.get_logger().info(f'{label} transform available: {parent} -> {child}')

        except (LookupException, ExtrapolationException) as e:
            # Log lookup failures periodically for debugging
            fail_count_attr = f'_{label.lower()}_fail_count'
            if not hasattr(self, fail_count_attr):
                setattr(self, fail_count_attr, 0)
            count = getattr(self, fail_count_attr)
            setattr(self, fail_count_attr, count + 1)

            # Log every 100 failures (every 5 seconds at 20Hz)
            if count % 100 == 0:
                self.get_logger().warn(
                    f'{label} TF lookup failed ({count} times): {parent} -> {child}: {e}'
                )

            if getattr(self, avail_attr):
                # Was available, now not - log warning
                self.get_logger().warn(f'{label} transform temporarily unavailable: {e}')
        except Exception as e:
            self.get_logger().error(f'{label} transform error: {e}')

    def odom_callback(self, msg: Odometry):
        """Store latest odometry message for delta computation."""
        self.latest_odom = msg
        if not self.odom_available:
            self.odom_available = True
            self.get_logger().info('Odometry available on /odom')

    def _quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle (rotation around Z axis)."""
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _compute_odom_slam_delta(self):
        """Compute delta between latest odometry and SLAM estimate."""
        if self.latest_odom is None:
            return

        # Get SLAM pose from the last estimated pose if available
        if not self.est_poses:
            return

        # Use the most recent SLAM estimate
        latest_est = self.est_poses[-1]
        slam_x, slam_y = latest_est[1], latest_est[2]
        slam_yaw = self._quaternion_to_yaw(latest_est[4], latest_est[5],
                                            latest_est[6], latest_est[7])

        # Extract odometry pose
        odom_pose = self.latest_odom.pose.pose
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        odom_yaw = self._quaternion_to_yaw(
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w
        )

        # Compute position delta (Euclidean distance)
        pos_delta = math.sqrt((slam_x - odom_x)**2 + (slam_y - odom_y)**2)

        # Compute angle delta (normalize to [-pi, pi])
        angle_delta = slam_yaw - odom_yaw
        # Normalize to [-pi, pi]
        while angle_delta > math.pi:
            angle_delta -= 2 * math.pi
        while angle_delta < -math.pi:
            angle_delta += 2 * math.pi
        angle_delta = abs(angle_delta)  # Store absolute value

        # Use SLAM timestamp for consistency
        timestamp = latest_est[0]

        # Store delta data
        self.odom_slam_delta.append((
            timestamp,
            odom_x, odom_y, odom_yaw,
            slam_x, slam_y, slam_yaw,
            pos_delta, angle_delta
        ))

    def periodic_save(self):
        """Periodically save trajectories as backup."""
        self.get_logger().info(
            f'Periodic save check: GT={len(self.gt_poses)}, EST={len(self.est_poses)}, '
            f'Delta={len(self.odom_slam_delta)}'
        )
        # Only save if we have new data
        if (len(self.gt_poses) > self.last_save_gt_count or
            len(self.est_poses) > self.last_save_est_count or
            len(self.odom_slam_delta) > self.last_save_delta_count):
            self.save_trajectories(quiet=False)  # Log saves for debugging
            self.last_save_gt_count = len(self.gt_poses)
            self.last_save_est_count = len(self.est_poses)
            self.last_save_delta_count = len(self.odom_slam_delta)

    def on_shutdown(self):
        """Callback when ROS context is shutting down."""
        self.get_logger().info('Shutdown requested, saving trajectories...')
        self.save_trajectories()

    def save_trajectories(self, quiet=False):
        """Save collected trajectories to TUM format files and odom/SLAM delta CSV."""
        gt_path = os.path.join(self.output_dir, 'gt.tum')
        est_path = os.path.join(self.output_dir, 'est.tum')
        delta_path = os.path.join(self.output_dir, 'odom_slam_delta.csv')

        # Save ground truth
        if self.gt_poses:
            self._write_tum_file(gt_path, self.gt_poses)
            if not quiet:
                self.get_logger().info(f'Saved {len(self.gt_poses)} GT poses to {gt_path}')
        elif not quiet:
            self.get_logger().warn('No GT poses collected')

        # Save estimate
        if self.est_poses:
            self._write_tum_file(est_path, self.est_poses)
            if not quiet:
                self.get_logger().info(f'Saved {len(self.est_poses)} EST poses to {est_path}')
        elif not quiet:
            self.get_logger().warn('No EST poses collected')

        # Save odom vs SLAM delta
        if self.odom_slam_delta:
            self._write_delta_csv(delta_path, self.odom_slam_delta)
            if not quiet:
                self.get_logger().info(
                    f'Saved {len(self.odom_slam_delta)} odom/SLAM delta samples to {delta_path}'
                )
        elif not quiet:
            self.get_logger().warn('No odom/SLAM delta data collected')

        return len(self.gt_poses), len(self.est_poses)

    def _write_tum_file(self, filepath, poses):
        """Write poses to TUM format file."""
        with open(filepath, 'w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
            for pose in poses:
                # Format: timestamp tx ty tz qx qy qz qw
                f.write(f'{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} '
                       f'{pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f} {pose[7]:.6f}\n')

    def _write_delta_csv(self, filepath, delta_data):
        """Write odom vs SLAM delta data to CSV file."""
        with open(filepath, 'w') as f:
            f.write('timestamp,odom_x,odom_y,odom_yaw,slam_x,slam_y,slam_yaw,pos_delta,angle_delta\n')
            for row in delta_data:
                # Format: timestamp, odom_x, odom_y, odom_yaw, slam_x, slam_y, slam_yaw, pos_delta, angle_delta
                f.write(f'{row[0]:.6f},{row[1]:.6f},{row[2]:.6f},{row[3]:.6f},'
                       f'{row[4]:.6f},{row[5]:.6f},{row[6]:.6f},{row[7]:.6f},{row[8]:.6f}\n')


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = TrajectoryExporter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as e:
        print(f'Parameter error: {e}')
        return 1
    finally:
        if node is not None:
            # Save trajectories on shutdown
            node.save_trajectories()
            node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
