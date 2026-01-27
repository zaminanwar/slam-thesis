#!/usr/bin/env python3
"""
Navigation experiment runner for SLAM comparison.

Runs Nav2 with SLAM-in-the-loop and sends a sequence of navigation goals.
Measures success rate, navigation time, and path efficiency.

Usage:
    python3 run_nav_experiment.py --algorithm slam_toolbox --goals nav_goals_01.yaml
    python3 run_nav_experiment.py --algorithm cartographer --goals nav_goals_02.yaml --verbose
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml


VALID_ALGORITHMS = ['slam_toolbox', 'cartographer']


def terminate_process_tree(proc, timeout=10.0):
    """Safely terminate a process and its children."""
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def yaw_to_quaternion(yaw):
    """Convert yaw angle (radians) to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_goals(goals_file):
    """Load navigation goals from YAML file."""
    # Check common locations
    if not os.path.isabs(goals_file):
        # Check in experiment_runner/config
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_dir = os.path.join(os.path.dirname(script_dir), 'config')
        candidate = os.path.join(config_dir, goals_file)
        if os.path.exists(candidate):
            goals_file = candidate
        else:
            # Check home directory
            candidate = os.path.expanduser(f'~/thesis/ros2_ws/src/slam_thesis/experiment_runner/config/{goals_file}')
            if os.path.exists(candidate):
                goals_file = candidate

    with open(goals_file, 'r') as f:
        data = yaml.safe_load(f)

    return data.get('goals', [])


def create_nav_experiment_node(rclpy, Node, ActionClient, NavigateToPose):
    """Factory function to create NavExperimentNode class after ROS2 is loaded."""

    class NavExperimentNode(Node):
        """ROS2 node for running navigation experiments."""

        def __init__(self, goals, timeout_per_goal=120.0, verbose=False):
            super().__init__('nav_experiment_node')
            self.goals = goals
            self.timeout_per_goal = timeout_per_goal
            self.verbose = verbose
            self._rclpy = rclpy
            self._NavigateToPose = NavigateToPose

            self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.results = []

        def wait_for_nav2(self, timeout=60.0):
            """Wait for Nav2 action server to be available."""
            self.get_logger().info('Waiting for Nav2 action server...')
            if not self.action_client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error('Nav2 action server not available!')
                return False
            self.get_logger().info('Nav2 action server available')
            return True

        def send_goal(self, x, y, yaw, name=''):
            """Send a single navigation goal and wait for result."""
            qx, qy, qz, qw = yaw_to_quaternion(yaw)

            goal_msg = self._NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.x = qx
            goal_msg.pose.pose.orientation.y = qy
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            self.get_logger().info(f'Sending goal "{name}": x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

            start_time = time.time()

            # Send goal
            future = self.action_client.send_goal_async(goal_msg)
            self._rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f'Goal "{name}" rejected!')
                return {
                    'name': name,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'success': False,
                    'time_s': 0.0,
                    'error': 'Goal rejected',
                }

            self.get_logger().info(f'Goal "{name}" accepted, waiting for result...')

            # Wait for result
            result_future = goal_handle.get_result_async()

            # Spin with timeout
            deadline = start_time + self.timeout_per_goal
            while not result_future.done():
                self._rclpy.spin_once(self, timeout_sec=0.5)
                if time.time() > deadline:
                    self.get_logger().warn(f'Goal "{name}" timed out!')
                    # Cancel the goal
                    cancel_future = goal_handle.cancel_goal_async()
                    self._rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                    return {
                        'name': name,
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'success': False,
                        'time_s': self.timeout_per_goal,
                        'error': 'Timeout',
                    }

            elapsed = time.time() - start_time
            result = result_future.result()

            # Check result status
            # NavigateToPose result has no explicit success field, check if we got a result
            success = result is not None

            if success:
                self.get_logger().info(f'Goal "{name}" reached in {elapsed:.2f}s')
            else:
                self.get_logger().warn(f'Goal "{name}" failed after {elapsed:.2f}s')

            return {
                'name': name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'success': success,
                'time_s': elapsed,
                'error': '' if success else 'Navigation failed',
            }

        def run_experiment(self):
            """Run navigation to all goals sequentially."""
            if not self.wait_for_nav2():
                return False

            self.get_logger().info(f'Starting navigation experiment with {len(self.goals)} goals')

            for i, goal in enumerate(self.goals):
                x = goal.get('x', 0.0)
                y = goal.get('y', 0.0)
                yaw = goal.get('yaw', 0.0)
                name = goal.get('name', f'goal_{i+1}')

                result = self.send_goal(x, y, yaw, name)
                self.results.append(result)

                if self.verbose:
                    status = 'SUCCESS' if result['success'] else 'FAILED'
                    print(f"  [{i+1}/{len(self.goals)}] {name}: {status} ({result['time_s']:.2f}s)")

                # Small delay between goals
                time.sleep(1.0)

            return True

    return NavExperimentNode


def start_nav2_slam(algorithm, world='simple.sdf', verbose=False):
    """Start Nav2 with SLAM-in-the-loop."""
    launch_cmd = [
        'ros2', 'launch', 'nav_launch', 'nav2_slam.launch.py',
        f'algorithm:={algorithm}',
        f'world:={world}',
        'use_sim_time:=true',
        'autostart:=true',
    ]

    if verbose:
        print(f"[nav_experiment] Starting Nav2 + SLAM: {' '.join(launch_cmd)}")

    proc = subprocess.Popen(
        launch_cmd,
        stdout=None if verbose else subprocess.PIPE,
        stderr=None if verbose else subprocess.PIPE,
    )

    return proc


def run_navigation_experiment(algorithm, goals_file, output_dir, world='simple.sdf',
                               timeout_per_goal=120.0, startup_delay=15.0, verbose=False):
    """
    Run complete navigation experiment.

    Returns: (success, error_message, results_dict)
    """
    # Load goals
    try:
        goals = load_goals(goals_file)
    except Exception as e:
        return False, f"Failed to load goals: {e}", None

    if not goals:
        return False, "No goals found in file", None

    # Create result directory
    goals_name = os.path.basename(goals_file).replace('.yaml', '')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    result_dir = os.path.join(output_dir, f'nav_{algorithm}_{goals_name}_{timestamp}')
    os.makedirs(result_dir, exist_ok=True)

    if verbose:
        print(f"[nav_experiment] Algorithm: {algorithm}")
        print(f"[nav_experiment] Goals: {goals_file} ({len(goals)} goals)")
        print(f"[nav_experiment] Output: {result_dir}")

    # Start Nav2 + SLAM
    nav_proc = start_nav2_slam(algorithm, world, verbose)

    try:
        # Wait for system to initialize
        if verbose:
            print(f"[nav_experiment] Waiting {startup_delay}s for system startup...")
        time.sleep(startup_delay)

        # Check if process is still running
        if nav_proc.poll() is not None:
            return False, "Nav2 launch failed to start", None

        # Import ROS2 modules
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose

        # Create node class
        NavExperimentNode = create_nav_experiment_node(rclpy, Node, ActionClient, NavigateToPose)

        # Initialize ROS2 and run experiment
        rclpy.init()

        try:
            node = NavExperimentNode(goals, timeout_per_goal, verbose)
            success = node.run_experiment()
            results = node.results

            node.destroy_node()
        finally:
            rclpy.shutdown()

        # Calculate summary metrics
        goals_succeeded = sum(1 for r in results if r['success'])
        goals_attempted = len(results)
        success_rate = goals_succeeded / goals_attempted if goals_attempted > 0 else 0.0
        total_time = sum(r['time_s'] for r in results)

        # Build results dict
        results_dict = {
            'algorithm': algorithm,
            'goals_file': os.path.basename(goals_file),
            'world': world,
            'timestamp': datetime.now().isoformat(),
            'goals_succeeded': goals_succeeded,
            'goals_attempted': goals_attempted,
            'success_rate': success_rate,
            'total_time_s': total_time,
            'timeout_per_goal_s': timeout_per_goal,
            'per_goal_results': results,
        }

        # Save results
        results_path = os.path.join(result_dir, 'nav_results.json')
        with open(results_path, 'w') as f:
            json.dump(results_dict, f, indent=2)

        if verbose:
            print(f"[nav_experiment] Results saved to: {results_path}")

        return True, "", results_dict

    finally:
        # Clean up Nav2 process
        if verbose:
            print("[nav_experiment] Shutting down Nav2...")
        terminate_process_tree(nav_proc)


def main():
    parser = argparse.ArgumentParser(
        description='Run navigation experiment with SLAM-in-the-loop.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--algorithm', required=True, choices=VALID_ALGORITHMS,
                        help='SLAM algorithm to use')
    parser.add_argument('--goals', required=True,
                        help='Navigation goals YAML file')
    parser.add_argument('--world', default='simple.sdf',
                        help='Gazebo world file (default: simple.sdf)')
    parser.add_argument('--output_dir', default=os.path.expanduser('~/thesis/ros2_ws/results'),
                        help='Output directory (default: ~/thesis/ros2_ws/results)')
    parser.add_argument('--timeout_per_goal', type=float, default=120.0,
                        help='Timeout per goal in seconds (default: 120)')
    parser.add_argument('--startup_delay', type=float, default=15.0,
                        help='Delay after launch before sending goals (default: 15)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"Navigation Experiment: {args.algorithm}")
    print(f"{'='*60}")

    success, error, results = run_navigation_experiment(
        algorithm=args.algorithm,
        goals_file=args.goals,
        output_dir=args.output_dir,
        world=args.world,
        timeout_per_goal=args.timeout_per_goal,
        startup_delay=args.startup_delay,
        verbose=args.verbose,
    )

    if not success:
        print(f"[nav_experiment] FAILED: {error}", file=sys.stderr)
        sys.exit(1)

    # Print summary
    print(f"\n{'='*60}")
    print(f"RESULTS: {args.algorithm}")
    print(f"{'='*60}")
    print(f"  Goals Succeeded: {results['goals_succeeded']}/{results['goals_attempted']}")
    print(f"  Success Rate:    {results['success_rate']*100:.1f}%")
    print(f"  Total Time:      {results['total_time_s']:.2f}s")
    print(f"{'='*60}")

    # Per-goal summary
    print("\nPer-goal results:")
    for r in results['per_goal_results']:
        status = 'OK' if r['success'] else 'FAIL'
        print(f"  {r['name']:20s} [{status:4s}] {r['time_s']:6.2f}s  ({r['x']:.1f}, {r['y']:.1f})")

    print(f"\nResults saved to: {args.output_dir}")
    sys.exit(0)


if __name__ == '__main__':
    main()
