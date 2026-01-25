#!/usr/bin/env python3
"""
Trajectory exporter node.
Samples TF transforms and writes TUM format trajectory files.
"""

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    # Node implementation will be added in T6.1
    rclpy.shutdown()


if __name__ == '__main__':
    main()
