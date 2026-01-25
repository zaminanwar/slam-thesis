#!/usr/bin/env python3
"""
run_one.py - Single experiment orchestrator.

Runs a complete SLAM evaluation experiment:
1. Start Gazebo simulation
2. Start ground truth publisher
3. Start SLAM (slam_toolbox or cartographer) with trajectory exporter
4. Execute trajectory
5. Wait for completion
6. Run evaluation (ATE/RPE metrics)

Usage:
    python3 run_one.py --algo slam_toolbox --trajectory traj_01_easy
    python3 run_one.py --algo cartographer --trajectory traj_02_loop --timeout 120
    python3 run_one.py --algo slam_toolbox --trajectory traj_01_easy --output_dir /custom/path

Output:
    results/{algo}_{trajectory}_{timestamp}/
    ├── gt.tum          # Ground truth trajectory
    ├── est.tum         # SLAM estimate trajectory
    └── metrics.json    # ATE/RPE evaluation results
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

# Constants
VALID_ALGORITHMS = ['slam_toolbox', 'cartographer']
DEFAULT_TIMEOUT = 180  # seconds
STARTUP_WAIT = 5  # seconds to wait for nodes to initialize
POST_TRAJECTORY_WAIT = 3  # seconds to wait after trajectory completes
TRAJECTORY_DONE_TOPIC = '/trajectory_done'


class ProcessManager:
    """Manages subprocess lifecycle with proper cleanup."""

    def __init__(self):
        self.processes: list[subprocess.Popen] = []
        self._setup_signal_handlers()

    def _setup_signal_handlers(self):
        """Setup handlers for graceful shutdown."""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle interrupt signals."""
        print(f"\n[run_one] Received signal {signum}, cleaning up...")
        self.cleanup()
        sys.exit(1)

    def start(self, cmd: list, name: str, env: Optional[dict] = None) -> subprocess.Popen:
        """Start a subprocess and track it."""
        print(f"[run_one] Starting: {name}")
        proc_env = os.environ.copy()
        if env:
            proc_env.update(env)

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=proc_env,
            preexec_fn=os.setsid  # Create new process group for clean termination
        )
        self.processes.append(proc)
        return proc

    def cleanup(self):
        """Terminate all managed processes."""
        print("[run_one] Cleaning up processes...")
        for proc in reversed(self.processes):
            if proc.poll() is None:  # Still running
                try:
                    # Send SIGINT first for graceful shutdown
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        # Force kill if still running
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                        proc.wait()
                except ProcessLookupError:
                    pass  # Already terminated
                except Exception as e:
                    print(f"[run_one] Warning: cleanup error: {e}")
        self.processes.clear()


def source_ros_env() -> dict:
    """Get environment with ROS workspace sourced."""
    # Source the workspace setup and capture environment
    workspace_setup = os.path.expanduser('~/thesis/ros2_ws/install/setup.bash')

    # Read ROS distro
    ros_distro = 'jazzy'  # Default
    distro_file = os.path.expanduser('~/thesis/.ros_distro')
    if os.path.exists(distro_file):
        with open(distro_file, 'r') as f:
            ros_distro = f.read().strip()

    cmd = f'source /opt/ros/{ros_distro}/setup.bash && source {workspace_setup} && env'
    result = subprocess.run(
        ['bash', '-c', cmd],
        capture_output=True,
        text=True
    )

    env = {}
    for line in result.stdout.splitlines():
        if '=' in line:
            key, _, value = line.partition('=')
            env[key] = value

    return env


def wait_for_topic(topic: str, timeout: float, env: dict) -> bool:
    """Wait for a topic to be available."""
    start = time.time()
    while time.time() - start < timeout:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            env=env
        )
        if topic in result.stdout.splitlines():
            return True
        time.sleep(0.5)
    return False


def wait_for_tf(parent: str, child: str, timeout: float, env: dict) -> bool:
    """Wait for a TF transform to be available."""
    start = time.time()
    attempts = 0
    while time.time() - start < timeout:
        attempts += 1
        try:
            # tf2_echo runs continuously, so we use timeout and check output
            proc = subprocess.Popen(
                ['ros2', 'run', 'tf2_ros', 'tf2_echo', parent, child],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # Merge stderr into stdout
                text=True,
                env=env
            )
            try:
                stdout, _ = proc.communicate(timeout=4)
                # Check for successful transform output
                if 'Translation' in stdout or 'At time' in stdout:
                    return True
            except subprocess.TimeoutExpired:
                # Read whatever output we got before killing
                proc.kill()
                stdout, _ = proc.communicate()
                # Check if we got valid transform output before timeout
                if 'Translation' in stdout or 'At time' in stdout:
                    return True
                # Debug: show waiting status periodically
                if attempts % 3 == 0:
                    print(f"[run_one]   Still waiting for {parent}->{child} TF...")
        except Exception as e:
            if attempts % 5 == 0:
                print(f"[run_one]   TF check error: {e}")
        time.sleep(1)
    return False


def wait_for_trajectory_done(timeout: float, env: dict) -> bool:
    """Wait for trajectory completion signal (data: true)."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            # Get a single message from the topic
            result = subprocess.run(
                ['ros2', 'topic', 'echo', TRAJECTORY_DONE_TOPIC,
                 'std_msgs/msg/Bool', '--once'],
                capture_output=True,
                text=True,
                env=env,
                timeout=5  # Short timeout per message check
            )
            if result.returncode == 0 and result.stdout:
                # Check if the message indicates completion (data: true)
                # The output format is like: "data: true" or "data: false"
                if 'data: true' in result.stdout.lower():
                    return True
                # Got a message but it's False, keep waiting
                time.sleep(0.5)
        except subprocess.TimeoutExpired:
            # No message received in 5s, keep trying
            continue
    return False


def run_experiment(
    algo: str,
    trajectory: str,
    output_dir: Path,
    timeout: float,
    speed_scale: float,
    verbose: bool
) -> dict:
    """
    Run a single SLAM evaluation experiment.

    Returns:
        dict with experiment results and status
    """
    results = {
        'algorithm': algo,
        'trajectory': trajectory,
        'output_dir': str(output_dir),
        'start_time': datetime.now().isoformat(),
        'status': 'unknown',
        'errors': []
    }

    pm = ProcessManager()
    env = source_ros_env()

    try:
        # Create output directory
        output_dir.mkdir(parents=True, exist_ok=True)

        # === Stage 1: Start Gazebo simulation ===
        print(f"\n[run_one] === Stage 1: Starting Gazebo simulation ===")
        sim_proc = pm.start(
            ['ros2', 'launch', 'rover_sim', 'sim.launch.py'],
            'Gazebo simulation',
            env
        )

        # Wait for /clock topic (indicates Gazebo is running)
        print("[run_one] Waiting for Gazebo to start...")
        if not wait_for_topic('/clock', 30, env):
            results['status'] = 'failed'
            results['errors'].append('Gazebo failed to start (no /clock topic)')
            return results

        # Wait for robot to spawn (pose topic available)
        print("[run_one] Waiting for robot to spawn...")
        if not wait_for_topic('/model/rover/pose', 30, env):
            results['status'] = 'failed'
            results['errors'].append('Robot failed to spawn (no /model/rover/pose topic)')
            return results

        # Extra wait for Gazebo to fully initialize
        time.sleep(STARTUP_WAIT)
        print("[run_one] Gazebo ready")

        # === Stage 2: Start ground truth publisher ===
        print(f"\n[run_one] === Stage 2: Starting ground truth publisher ===")
        gt_proc = pm.start(
            ['ros2', 'launch', 'gt_publisher', 'gt_publisher.launch.py'],
            'Ground truth publisher',
            env
        )

        # Wait for /gt_pose topic (indicates gt_publisher is working and receiving pose data)
        # The TF is published at the same time as /gt_pose
        print("[run_one] Waiting for ground truth publisher...")
        if not wait_for_topic('/gt_pose', 30, env):
            results['status'] = 'failed'
            results['errors'].append('Ground truth not available (no /gt_pose topic)')
            return results
        # Give a moment for TF to stabilize
        time.sleep(2)
        print("[run_one] Ground truth ready")

        # === Stage 3: Start SLAM with trajectory exporter ===
        print(f"\n[run_one] === Stage 3: Starting {algo} with trajectory exporter ===")
        slam_launch = f'{algo}_eval.launch.py'
        slam_proc = pm.start(
            ['ros2', 'launch', 'slam_launch', slam_launch,
             f'output_dir:={output_dir}'],
            f'{algo} + trajectory exporter',
            env
        )

        # Wait for SLAM to initialize - check for /map topic as indicator
        print("[run_one] Waiting for SLAM to initialize...")
        if not wait_for_topic('/map', 30, env):
            results['errors'].append('Warning: SLAM /map topic not available yet')
            # Continue anyway - SLAM might just need more data

        time.sleep(STARTUP_WAIT)
        print(f"[run_one] {algo} ready")

        # === Stage 4: Execute trajectory ===
        print(f"\n[run_one] === Stage 4: Executing trajectory: {trajectory} ===")
        traj_file = f'{trajectory}.csv' if not trajectory.endswith('.csv') else trajectory

        traj_proc = pm.start(
            ['ros2', 'launch', 'rover_control', 'follow_trajectory.launch.py',
             f'trajectory:={traj_file}',
             f'speed_scale:={speed_scale}'],
            'Trajectory follower',
            env
        )

        # Wait for trajectory completion
        print(f"[run_one] Waiting for trajectory to complete (timeout: {timeout}s)...")
        traj_start = time.time()

        if wait_for_trajectory_done(timeout, env):
            elapsed = time.time() - traj_start
            print(f"[run_one] Trajectory completed in {elapsed:.1f}s")
            results['trajectory_time'] = elapsed
        else:
            elapsed = time.time() - traj_start
            print(f"[run_one] Trajectory timeout after {elapsed:.1f}s")
            results['errors'].append(f'Trajectory timeout after {elapsed:.1f}s')

        # Post-trajectory wait for data to settle and periodic save to trigger
        # Periodic save happens every 5 seconds, so wait at least 6 seconds
        wait_time = max(POST_TRAJECTORY_WAIT, 6)
        print(f"[run_one] Waiting {wait_time}s for data to settle and save...")
        time.sleep(wait_time)

        # Check if files were created by periodic save
        gt_check = output_dir / 'gt.tum'
        est_check = output_dir / 'est.tum'
        print(f"[run_one] Checking for trajectory files before cleanup...")
        print(f"[run_one]   gt.tum exists: {gt_check.exists()}")
        print(f"[run_one]   est.tum exists: {est_check.exists()}")

        # === Stage 5: Stop SLAM and exporter (saves trajectory files) ===
        print(f"\n[run_one] === Stage 5: Stopping SLAM (saving trajectories) ===")
        results['end_time'] = datetime.now().isoformat()

    except Exception as e:
        results['status'] = 'failed'
        results['errors'].append(f'Exception during experiment: {str(e)}')
        return results
    finally:
        # Clean up all processes
        pm.cleanup()

    # === Stage 6: Run evaluation ===
    print(f"\n[run_one] === Stage 6: Running evaluation ===")
    gt_file = output_dir / 'gt.tum'
    est_file = output_dir / 'est.tum'

    # Check if trajectory files exist
    if not gt_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Ground truth file not found: {gt_file}')
        return results

    if not est_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Estimate file not found: {est_file}')
        return results

    # Count poses
    def count_poses(f):
        count = 0
        with open(f, 'r') as fp:
            for line in fp:
                if line.strip() and not line.startswith('#'):
                    count += 1
        return count

    results['gt_poses'] = count_poses(gt_file)
    results['est_poses'] = count_poses(est_file)
    print(f"[run_one] GT poses: {results['gt_poses']}, EST poses: {results['est_poses']}")

    # Run evaluate_run.py
    eval_script = Path(__file__).parent / 'evaluate_run.py'
    eval_result = subprocess.run(
        ['python3', str(eval_script), '--run_dir', str(output_dir), '--verbose'],
        capture_output=True,
        text=True
    )

    if eval_result.returncode != 0:
        results['errors'].append(f'Evaluation failed: {eval_result.stderr}')

    # Load metrics if available
    metrics_file = output_dir / 'metrics.json'
    if metrics_file.exists():
        with open(metrics_file, 'r') as f:
            metrics = json.load(f)
            results['metrics'] = metrics
            results['status'] = metrics.get('status', 'unknown')
            if verbose:
                if metrics.get('ate'):
                    print(f"[run_one] ATE RMSE: {metrics['ate']['rmse']:.4f} m")
                if metrics.get('rpe'):
                    print(f"[run_one] RPE RMSE: {metrics['rpe']['rmse']:.4f} m")
    else:
        results['status'] = 'failed'
        results['errors'].append('Metrics file not generated')

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Run a single SLAM evaluation experiment',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run slam_toolbox on traj_01_easy
    python3 run_one.py --algo slam_toolbox --trajectory traj_01_easy

    # Run cartographer on traj_02_loop with custom timeout
    python3 run_one.py --algo cartographer --trajectory traj_02_loop --timeout 120

    # Run with custom output directory
    python3 run_one.py --algo slam_toolbox --trajectory traj_01_easy \\
        --output_dir ~/thesis/ros2_ws/results/my_test

Output structure:
    results/{algo}_{trajectory}_{timestamp}/
    ├── gt.tum          # Ground truth trajectory
    ├── est.tum         # SLAM estimate trajectory
    └── metrics.json    # ATE/RPE evaluation results
"""
    )

    parser.add_argument(
        '--algo', '-a',
        choices=VALID_ALGORITHMS,
        required=True,
        help='SLAM algorithm to evaluate'
    )

    parser.add_argument(
        '--trajectory', '-t',
        required=True,
        help='Trajectory name (e.g., traj_01_easy) or full path'
    )

    parser.add_argument(
        '--output_dir', '-o',
        type=Path,
        help='Output directory (default: results/{algo}_{trajectory}_{timestamp})'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f'Maximum time to wait for trajectory completion (default: {DEFAULT_TIMEOUT}s)'
    )

    parser.add_argument(
        '--speed_scale',
        type=float,
        default=1.0,
        help='Trajectory speed multiplier (default: 1.0)'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed output'
    )

    args = parser.parse_args()

    # Determine output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        traj_name = Path(args.trajectory).stem
        results_base = Path.home() / 'thesis' / 'ros2_ws' / 'results'
        output_dir = results_base / f'{args.algo}_{traj_name}_{timestamp}'

    print(f"[run_one] SLAM Evaluation Experiment")
    print(f"[run_one] Algorithm: {args.algo}")
    print(f"[run_one] Trajectory: {args.trajectory}")
    print(f"[run_one] Output: {output_dir}")
    print(f"[run_one] Timeout: {args.timeout}s")

    # Run experiment
    results = run_experiment(
        algo=args.algo,
        trajectory=args.trajectory,
        output_dir=output_dir,
        timeout=args.timeout,
        speed_scale=args.speed_scale,
        verbose=args.verbose
    )

    # Save run metadata
    run_info_file = output_dir / 'run_info.json'
    with open(run_info_file, 'w') as f:
        json.dump(results, f, indent=2)

    # Print summary
    print(f"\n[run_one] === Experiment Complete ===")
    print(f"[run_one] Status: {results['status']}")
    print(f"[run_one] Output: {output_dir}")

    if results['errors']:
        print("[run_one] Errors:")
        for err in results['errors']:
            print(f"  - {err}")

    # Exit code based on status
    if results['status'] in ['success', 'partial']:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
