#!/usr/bin/env python3
"""
Completion monitor node for real-time SLAM evaluation.

Monitors the /trajectory_done topic and triggers graceful shutdown
after the trajectory is complete and SLAM has had time to settle.

Usage:
    Typically launched as part of run_realtime.launch.py, not standalone.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class CompletionMonitor(Node):
    """
    Monitors trajectory completion and triggers graceful shutdown.

    Subscribes to /trajectory_done (latched Bool) and waits for True.
    After receiving True, waits for post_trajectory_wait seconds to allow
    SLAM algorithms to finish processing, then triggers shutdown.
    """

    def __init__(self):
        super().__init__('completion_monitor')

        # Declare parameters
        self.declare_parameter('post_trajectory_wait', 5.0)
        self.declare_parameter('timeout', 600.0)

        # Get parameters
        self.post_wait = self.get_parameter('post_trajectory_wait').value
        self.timeout = self.get_parameter('timeout').value

        # State
        self.trajectory_complete = False
        self.shutdown_timer = None

        # QoS to match trajectory_follower's Transient Local (latched) publisher
        done_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribe to trajectory_done
        self.done_sub = self.create_subscription(
            Bool,
            '/trajectory_done',
            self._done_callback,
            done_qos
        )

        # Timeout timer
        self.timeout_timer = self.create_timer(self.timeout, self._timeout_callback)

        self.get_logger().info(
            f'Completion monitor started (post_wait={self.post_wait}s, timeout={self.timeout}s)'
        )

    def _done_callback(self, msg: Bool):
        """Handle trajectory_done messages."""
        if msg.data and not self.trajectory_complete:
            self.trajectory_complete = True
            self.get_logger().info(
                f'Trajectory complete! Waiting {self.post_wait}s for SLAM to settle...'
            )

            # Cancel timeout timer
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
                self.timeout_timer = None

            # Create one-shot timer for post-wait shutdown
            self.shutdown_timer = self.create_timer(
                self.post_wait,
                self._shutdown_callback
            )

    def _timeout_callback(self):
        """Handle experiment timeout."""
        self.get_logger().warn(f'Experiment timed out after {self.timeout}s')
        self._initiate_shutdown()

    def _shutdown_callback(self):
        """Initiate graceful shutdown after post-wait period."""
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None

        self.get_logger().info('Post-wait complete, initiating shutdown...')
        self._initiate_shutdown()

    def _initiate_shutdown(self):
        """Trigger graceful shutdown of the launch system."""
        self.get_logger().info('Shutting down...')
        # Raising SystemExit causes the launch system to terminate all nodes
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)

    node = CompletionMonitor()

    try:
        rclpy.spin(node)
    except SystemExit:
        # Expected exit from _initiate_shutdown
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
