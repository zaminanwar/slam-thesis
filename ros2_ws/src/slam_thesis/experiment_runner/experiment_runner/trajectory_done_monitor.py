#!/usr/bin/env python3
"""
Trajectory done monitor node.

Monitors the /trajectory_done topic and triggers a graceful shutdown
after a configurable delay when the trajectory is complete.

Used in live SLAM experiments to automatically end the experiment
when the rover finishes following its trajectory.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool


class TrajectoryDoneMonitor(Node):
    """
    Monitor for trajectory completion.

    Subscribes to /trajectory_done and initiates shutdown when True is received.
    """

    def __init__(self):
        super().__init__('trajectory_done_monitor')

        # Parameters
        self.declare_parameter('shutdown_delay', 3.0)
        self.shutdown_delay = self.get_parameter('shutdown_delay').value

        # Track state
        self.shutdown_initiated = False

        # Subscribe with transient local QoS to match publisher
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            Bool,
            '/trajectory_done',
            self.trajectory_done_callback,
            qos
        )

        self.get_logger().info(
            f'Trajectory done monitor started (shutdown_delay={self.shutdown_delay}s)'
        )

    def trajectory_done_callback(self, msg: Bool):
        """Handle trajectory_done messages."""
        if msg.data and not self.shutdown_initiated:
            self.shutdown_initiated = True
            self.get_logger().info(
                f'Trajectory complete! Shutting down in {self.shutdown_delay}s...'
            )

            # Schedule shutdown after delay
            self.create_timer(self.shutdown_delay, self.do_shutdown)

    def do_shutdown(self):
        """Perform shutdown."""
        self.get_logger().info('Initiating shutdown...')
        # Raise SystemExit to cleanly exit the node
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryDoneMonitor()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
