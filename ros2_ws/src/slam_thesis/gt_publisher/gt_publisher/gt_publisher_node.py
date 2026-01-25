#!/usr/bin/env python3
"""
Ground truth publisher node for SLAM thesis.

Subscribes to Gazebo model pose (bridged via ros_gz_bridge) and publishes:
- /gt_pose (nav_msgs/Odometry) at configurable rate
- TF: map_gt -> base_footprint_gt

IMPORTANT: Uses base_footprint_gt (NOT base_footprint) as the child frame to avoid
conflicting with the main TF tree where Gazebo publishes odom -> base_footprint.
This keeps the ground truth TF tree completely separate.

The Gazebo model pose is at base_footprint level (ground plane).
Uses use_sim_time for proper synchronization with simulation/bag replay.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GroundTruthPublisher(Node):
    """Publishes ground truth pose from Gazebo model state."""

    def __init__(self):
        super().__init__('gt_publisher')

        # Declare parameters
        self.declare_parameter('model_name', 'rover')
        self.declare_parameter('publish_rate', 50.0)

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Store latest pose from Gazebo
        self.latest_pose = None
        self.pose_received = False

        # Subscriber to bridged Gazebo pose
        pose_topic = f'/model/{self.model_name}/pose'
        self.pose_sub = self.create_subscription(
            Pose,
            pose_topic,
            self.pose_callback,
            10
        )

        # Publisher for ground truth odometry
        self.gt_pub = self.create_publisher(Odometry, '/gt_pose', 10)

        # TF broadcaster for map_gt -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_gt)

        self.get_logger().info(
            f'Ground truth publisher started: subscribing to {pose_topic}, '
            f'publishing at {self.publish_rate} Hz'
        )

    def pose_callback(self, msg: Pose):
        """Store the latest pose from Gazebo."""
        self.latest_pose = msg
        self.pose_received = True

    def publish_gt(self):
        """Publish ground truth pose and TF."""
        if not self.pose_received or self.latest_pose is None:
            return

        now = self.get_clock().now()

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map_gt'
        odom.child_frame_id = 'base_footprint_gt'

        # Copy pose
        odom.pose.pose = self.latest_pose

        # Covariance: set small values for ground truth (nearly perfect)
        # 6x6 matrix in row-major: [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[14] = 0.001  # z
        odom.pose.covariance[21] = 0.001  # roll
        odom.pose.covariance[28] = 0.001  # pitch
        odom.pose.covariance[35] = 0.001  # yaw

        # Twist is zero (we don't have velocity from pose alone)
        # Could be computed from pose differences if needed

        self.gt_pub.publish(odom)

        # Publish TF: map_gt -> base_footprint_gt (separate from main TF tree)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'map_gt'
        tf_msg.child_frame_id = 'base_footprint_gt'

        tf_msg.transform.translation.x = self.latest_pose.position.x
        tf_msg.transform.translation.y = self.latest_pose.position.y
        tf_msg.transform.translation.z = self.latest_pose.position.z

        tf_msg.transform.rotation.x = self.latest_pose.orientation.x
        tf_msg.transform.rotation.y = self.latest_pose.orientation.y
        tf_msg.transform.rotation.z = self.latest_pose.orientation.z
        tf_msg.transform.rotation.w = self.latest_pose.orientation.w

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
