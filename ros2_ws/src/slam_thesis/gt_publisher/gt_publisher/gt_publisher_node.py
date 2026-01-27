#!/usr/bin/env python3
"""
Ground truth publisher node for SLAM thesis.

Subscribes to Gazebo P3D plugin output (/gt_odom) and publishes:
- /gt_pose (nav_msgs/Odometry) at configurable rate
- TF: map_gt -> base_link

Uses use_sim_time for proper synchronization with simulation/bag replay.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GroundTruthPublisher(Node):
    """Publishes ground truth pose from Gazebo P3D plugin."""

    def __init__(self):
        super().__init__('gt_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)

        self.publish_rate = self.get_parameter('publish_rate').value

        # Store latest odometry from Gazebo P3D plugin
        self.latest_odom = None
        self.odom_received = False

        # Subscriber to Gazebo P3D plugin output
        # P3D plugin publishes Odometry messages directly
        gt_topic = '/gt_odom'
        self.odom_sub = self.create_subscription(
            Odometry,
            gt_topic,
            self.odom_callback,
            10
        )

        # Publisher for ground truth odometry (with proper frame IDs)
        self.gt_pub = self.create_publisher(Odometry, '/gt_pose', 10)

        # TF broadcaster for map_gt -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_gt)

        self.get_logger().info(
            f'Ground truth publisher started: subscribing to {gt_topic}, '
            f'publishing at {self.publish_rate} Hz'
        )

    def odom_callback(self, msg: Odometry):
        """Store the latest odometry from Gazebo P3D plugin."""
        self.latest_odom = msg
        self.odom_received = True

    def publish_gt(self):
        """Publish ground truth pose and TF."""
        if not self.odom_received or self.latest_odom is None:
            return

        now = self.get_clock().now()

        # Publish Odometry message with proper frame IDs
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map_gt'
        odom.child_frame_id = 'base_link'

        # Copy pose from P3D output
        odom.pose.pose = self.latest_odom.pose.pose

        # Covariance: set small values for ground truth (nearly perfect)
        # 6x6 matrix in row-major: [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[14] = 0.001  # z
        odom.pose.covariance[21] = 0.001  # roll
        odom.pose.covariance[28] = 0.001  # pitch
        odom.pose.covariance[35] = 0.001  # yaw

        # Copy twist if available from P3D
        odom.twist = self.latest_odom.twist

        self.gt_pub.publish(odom)

        # Publish TF: map_gt -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'map_gt'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = self.latest_odom.pose.pose.position.x
        tf_msg.transform.translation.y = self.latest_odom.pose.pose.position.y
        tf_msg.transform.translation.z = self.latest_odom.pose.pose.position.z

        tf_msg.transform.rotation.x = self.latest_odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = self.latest_odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = self.latest_odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = self.latest_odom.pose.pose.orientation.w

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
