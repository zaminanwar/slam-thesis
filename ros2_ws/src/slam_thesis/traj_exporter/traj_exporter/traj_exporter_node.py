#!/usr/bin/env python3
"""
Trajectory exporter node.
Samples TF transforms and writes TUM format trajectory files.

TUM format: timestamp tx ty tz qx qy qz qw
- timestamp: seconds with decimal (float)
- tx, ty, tz: position in meters
- qx, qy, qz, qw: orientation as quaternion

Usage:
  ros2 run traj_exporter traj_exporter --ros-args -p output_file:=/path/to/traj.txt
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


class TrajectoryExporter(Node):
    """
    Node that listens to TF transforms and exports trajectory to TUM format file.

    Samples the transform from parent_frame to child_frame at a configurable rate
    and writes each pose to a TUM format trajectory file.
    """

    def __init__(self):
        super().__init__('traj_exporter')

        # Declare parameters
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('output_file', '/tmp/trajectory.txt')
        self.declare_parameter('sample_rate', 10.0)  # Hz
        self.declare_parameter('transform_timeout', 0.1)  # seconds

        # Get parameters
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        self.transform_timeout = self.get_parameter('transform_timeout').get_parameter_value().double_value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Trajectory data storage
        self.trajectory_data = []
        self.last_timestamp: Optional[float] = None

        # Create output directory if needed
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        # Open file for writing (overwrite mode)
        self._file_handle = None
        try:
            self._file_handle = open(self.output_file, 'w')
            # Write TUM header comment
            self._file_handle.write('# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n')
            self._file_handle.flush()
        except IOError as e:
            self.get_logger().error(f'Failed to open output file {self.output_file}: {e}')
            raise

        # Timer for sampling
        sample_period = 1.0 / self.sample_rate
        self.sample_timer = self.create_timer(sample_period, self.sample_transform)

        self.get_logger().info(
            f'Trajectory exporter started: {self.parent_frame} -> {self.child_frame} '
            f'at {self.sample_rate} Hz -> {self.output_file}'
        )

        # Statistics
        self.samples_written = 0
        self.samples_failed = 0

    def sample_transform(self):
        """Sample the current TF transform and write to file."""
        try:
            # Use Time(seconds=0) to get the latest available transform
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                Time(seconds=0),
                timeout=Duration(seconds=self.transform_timeout)
            )

            # Extract timestamp
            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            # Skip duplicate timestamps
            if self.last_timestamp is not None and timestamp <= self.last_timestamp:
                return
            self.last_timestamp = timestamp

            # Extract translation
            t = transform.transform.translation
            tx, ty, tz = t.x, t.y, t.z

            # Extract rotation (quaternion)
            r = transform.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w

            # Write TUM format line
            line = f'{timestamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n'

            if self._file_handle:
                self._file_handle.write(line)
                self._file_handle.flush()

            self.samples_written += 1

            # Log progress periodically
            if self.samples_written % 100 == 0:
                self.get_logger().info(f'Samples written: {self.samples_written}')

        except (LookupException, ExtrapolationException) as e:
            self.samples_failed += 1
            # Only log occasionally to avoid spam
            if self.samples_failed <= 5 or self.samples_failed % 100 == 0:
                self.get_logger().debug(
                    f'Transform lookup failed ({self.samples_failed}x): {e}'
                )

    def destroy_node(self):
        """Clean up resources on shutdown."""
        # Close file handle
        if self._file_handle:
            self._file_handle.close()
            self._file_handle = None

        self.get_logger().info(
            f'Trajectory export complete: {self.samples_written} samples written, '
            f'{self.samples_failed} lookups failed -> {self.output_file}'
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryExporter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
