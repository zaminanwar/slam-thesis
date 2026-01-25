#!/usr/bin/env python3
"""
Trajectory exporter node for SLAM thesis evaluation.

Samples TF transforms at a configurable rate and writes TUM format trajectory files:
- gt.tum: Ground truth trajectory (map_gt -> base_footprint_gt)
- est.tum: SLAM estimate trajectory (map -> base_footprint)

Note: GT uses base_footprint_gt (NOT base_footprint) to avoid TF tree conflicts
with the main robot tree where Gazebo publishes odom -> base_footprint.

TUM format: timestamp tx ty tz qx qy qz qw
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


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

        # Track availability
        self.gt_available = False
        self.est_available = False

        # Create timer for sampling
        timer_period = 1.0 / self.sample_rate
        self.timer = self.create_timer(timer_period, self.sample_transforms)

        # Create timer for periodic saving (every 5 seconds as backup)
        self.save_timer = self.create_timer(5.0, self.periodic_save)
        self.last_save_gt_count = 0
        self.last_save_est_count = 0

        # Register shutdown callback
        self.context.on_shutdown(self.on_shutdown)

        self.get_logger().info(
            f'Trajectory exporter started:\n'
            f'  GT: {self.gt_parent} -> {self.gt_child}\n'
            f'  EST: {self.est_parent} -> {self.est_child}\n'
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

    def periodic_save(self):
        """Periodically save trajectories as backup."""
        self.get_logger().info(f'Periodic save check: GT={len(self.gt_poses)}, EST={len(self.est_poses)}')
        # Only save if we have new data
        if (len(self.gt_poses) > self.last_save_gt_count or
            len(self.est_poses) > self.last_save_est_count):
            self.save_trajectories(quiet=False)  # Log saves for debugging
            self.last_save_gt_count = len(self.gt_poses)
            self.last_save_est_count = len(self.est_poses)

    def on_shutdown(self):
        """Callback when ROS context is shutting down."""
        self.get_logger().info('Shutdown requested, saving trajectories...')
        self.save_trajectories()

    def save_trajectories(self, quiet=False):
        """Save collected trajectories to TUM format files."""
        gt_path = os.path.join(self.output_dir, 'gt.tum')
        est_path = os.path.join(self.output_dir, 'est.tum')

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

        return len(self.gt_poses), len(self.est_poses)

    def _write_tum_file(self, filepath, poses):
        """Write poses to TUM format file."""
        with open(filepath, 'w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
            for pose in poses:
                # Format: timestamp tx ty tz qx qy qz qw
                f.write(f'{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} '
                       f'{pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f} {pose[7]:.6f}\n')


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
