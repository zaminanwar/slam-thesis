"""
Odometry noise injection node for degraded condition experiments.

Subscribes to odometry, adds Gaussian noise and drift, then republishes.
This simulates degraded wheel encoder performance for robustness testing.

Parameters:
  - noise_std: Standard deviation of Gaussian noise (meters for position, radians for orientation)
  - drift_rate: Rate of cumulative drift per second (meters/s for position, radians/s for orientation)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to Euler angles [roll, pitch, yaw]."""
    x, y, z, w = q
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return [x, y, z, w]


class OdomNoiseNode(Node):
    """Node that injects noise and drift into odometry messages."""

    def __init__(self):
        super().__init__('odom_noise')

        # Declare parameters
        self.declare_parameter('noise_std', 0.05)
        self.declare_parameter('drift_rate', 0.01)

        # Get parameters
        self.noise_std = self.get_parameter('noise_std').value
        self.drift_rate = self.get_parameter('drift_rate').value

        self.get_logger().info(
            f'Odometry noise node started with noise_std={self.noise_std}, drift_rate={self.drift_rate}'
        )

        # Initialize drift accumulator
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0
        self.last_time = None

        # Create subscriber and publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Odometry,
            'odom_in',
            self.odom_callback,
            qos
        )

        self.publisher = self.create_publisher(
            Odometry,
            'odom_out',
            qos
        )

    def odom_callback(self, msg: Odometry):
        """Process incoming odometry and add noise."""
        current_time = self.get_clock().now()

        # Update drift accumulator
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            # Add random drift increment
            self.drift_x += np.random.normal(0, self.drift_rate * dt)
            self.drift_y += np.random.normal(0, self.drift_rate * dt)
            self.drift_yaw += np.random.normal(0, self.drift_rate * dt * 0.1)  # Less yaw drift

        self.last_time = current_time

        # Create output message (copy input)
        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.child_frame_id = msg.child_frame_id

        # Add noise to position
        noise_x = np.random.normal(0, self.noise_std)
        noise_y = np.random.normal(0, self.noise_std)

        out_msg.pose.pose.position.x = msg.pose.pose.position.x + noise_x + self.drift_x
        out_msg.pose.pose.position.y = msg.pose.pose.position.y + noise_y + self.drift_y
        out_msg.pose.pose.position.z = msg.pose.pose.position.z

        # Add noise to orientation (yaw only for 2D)
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        noise_yaw = np.random.normal(0, self.noise_std * 0.1)  # Less orientation noise
        noisy_yaw = yaw + noise_yaw + self.drift_yaw

        q_noisy = quaternion_from_euler(roll, pitch, noisy_yaw)
        out_msg.pose.pose.orientation = Quaternion(
            x=q_noisy[0], y=q_noisy[1], z=q_noisy[2], w=q_noisy[3]
        )

        # Copy covariance (could increase it to reflect added noise)
        out_msg.pose.covariance = list(msg.pose.covariance)

        # Copy twist (velocity) with some noise
        out_msg.twist.twist.linear.x = msg.twist.twist.linear.x + np.random.normal(0, self.noise_std * 0.1)
        out_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        out_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        out_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        out_msg.twist.twist.angular.y = msg.twist.twist.angular.y
        out_msg.twist.twist.angular.z = msg.twist.twist.angular.z + np.random.normal(0, self.noise_std * 0.1)
        out_msg.twist.covariance = list(msg.twist.covariance)

        self.publisher.publish(out_msg)


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
