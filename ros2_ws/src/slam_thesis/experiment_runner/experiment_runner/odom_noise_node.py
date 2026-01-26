#!/usr/bin/env python3
"""
Odometry noise injection node for degraded condition testing.

Subscribes to /odom_raw, adds Gaussian noise, and publishes to /odom_noisy.
Used to simulate degraded odometry conditions for SLAM robustness testing.

Parameters:
  - noise_std_linear: Standard deviation for linear velocity noise (m/s)
  - noise_std_angular: Standard deviation for angular velocity noise (rad/s)
  - drift_rate: Cumulative drift rate per meter traveled (m/m)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import copy


class OdomNoiseNode(Node):
    def __init__(self):
        super().__init__('odom_noise_node')

        # Declare parameters
        self.declare_parameter('noise_std_linear', 0.02)  # 2cm std dev
        self.declare_parameter('noise_std_angular', 0.01)  # ~0.6 deg std dev
        self.declare_parameter('drift_rate', 0.0)  # Cumulative drift (disabled by default)

        # Get parameters
        self.noise_std_linear = self.get_parameter('noise_std_linear').value
        self.noise_std_angular = self.get_parameter('noise_std_angular').value
        self.drift_rate = self.get_parameter('drift_rate').value

        # State for cumulative drift
        self.cumulative_drift_x = 0.0
        self.cumulative_drift_y = 0.0
        self.last_odom = None

        # Publisher and subscriber
        self.odom_pub = self.create_publisher(Odometry, '/odom_noisy', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Odom noise node started: linear_std={self.noise_std_linear:.3f}, '
            f'angular_std={self.noise_std_angular:.3f}, drift_rate={self.drift_rate:.4f}'
        )

    def odom_callback(self, msg: Odometry):
        # Deep copy to avoid modifying the original message
        noisy_odom = copy.deepcopy(msg)

        # Add Gaussian noise to position
        noisy_odom.pose.pose.position.x += np.random.normal(0, self.noise_std_linear)
        noisy_odom.pose.pose.position.y += np.random.normal(0, self.noise_std_linear)

        # Add noise to orientation (yaw via quaternion z component - simplified)
        # For small angles, we can approximate by adding noise to the z component
        noisy_odom.pose.pose.orientation.z += np.random.normal(0, self.noise_std_angular * 0.5)

        # Add noise to velocity
        noisy_odom.twist.twist.linear.x += np.random.normal(0, self.noise_std_linear)
        noisy_odom.twist.twist.angular.z += np.random.normal(0, self.noise_std_angular)

        # Apply cumulative drift if enabled
        if self.drift_rate > 0 and self.last_odom is not None:
            # Calculate distance traveled since last message
            dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)

            # Add drift proportional to distance traveled
            self.cumulative_drift_x += dist * self.drift_rate * np.random.normal(0, 1)
            self.cumulative_drift_y += dist * self.drift_rate * np.random.normal(0, 1)

            noisy_odom.pose.pose.position.x += self.cumulative_drift_x
            noisy_odom.pose.pose.position.y += self.cumulative_drift_y

        self.last_odom = msg

        # Increase covariance to reflect added noise
        # Position covariance indices: 0 (xx), 7 (yy), 14 (zz), 21 (roll), 28 (pitch), 35 (yaw)
        noisy_odom.pose.covariance[0] += self.noise_std_linear ** 2
        noisy_odom.pose.covariance[7] += self.noise_std_linear ** 2
        noisy_odom.pose.covariance[35] += self.noise_std_angular ** 2

        # Publish noisy odometry
        self.odom_pub.publish(noisy_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
