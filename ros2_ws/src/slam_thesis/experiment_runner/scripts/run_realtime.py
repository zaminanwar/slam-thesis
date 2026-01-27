#!/usr/bin/env python3
"""
Real-time SLAM experiment orchestrator.

Runs a single SLAM algorithm with live Gazebo simulation (no rosbag),
exports trajectories, and computes evaluation metrics.

Usage:
    python3 run_realtime.py --trajectory traj_01_easy.csv --algorithm slam_toolbox
    python3 run_realtime.py --trajectory traj_02_loop.csv --algorithm cartographer --condition degraded

Output:
    Creates in the output directory:
    - gt_trajectory.tum: Ground truth trajectory (odom -> base_footprint)
    - est_trajectory.tum: SLAM estimated trajectory (map -> base_footprint)
    - metrics.json: ATE/RPE evaluation metrics
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
from typing import Optional, Tuple

# Valid SLAM algorithms
VALID_ALGORITHMS = ['slam_toolbox', 'cartographer']

# Valid conditions
VALID_CONDITIONS = ['baseline', 'degraded']


def find_script_path(script_name: str) -> str:
    """Find the path to a script in the experiment_runner package."""
    # Try installed location first
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory('experiment_runner')
        script_path = os.path.join(pkg_dir, 'scripts', script_name)
        if os.path.exists(script_path):
            return script_path
    except Exception:
        pass

    # Fallback to source location (same directory as this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(script_dir, script_name)
    if os.path.exists(script_path):
        return script_path

    raise FileNotFoundError(f"Could not find script: {script_name}")


def find_trajectory_file(trajectory: str) -> str:
    """Find and validate trajectory file."""
    # If it's already an absolute path
    if trajectory.startswith('/'):
        if os.path.exists(trajectory):
            return trajectory
        raise ValueError(f"Trajectory file not found: {trajectory}")

    # Check default trajectory directory
    default_traj_dir = os.path.expanduser('~/thesis/trajectories')
    traj_path = os.path.join(default_traj_dir, trajectory)

    if os.path.exists(traj_path):
        return traj_path

    raise ValueError(f"Trajectory file not found: {trajectory} (checked {traj_path})")


def extract_trajectory_name(trajectory: str) -> str:
    """Extract trajectory name without extension."""
    return Path(trajectory).stem


def run_realtime_experiment(
    trajectory: str,
    algorithm: str,
    condition: str,
    gt_file: str,
    est_file: str,
    post_trajectory_wait: float = 5.0,
    timeout: float = 600.0,
    verbose: bool = False
) -> Tuple[bool, str]:
    """
    Run real-time SLAM experiment.

    Args:
        trajectory: Trajectory CSV filename or path
        algorithm: SLAM algorithm (slam_toolbox or cartographer)
        condition: Experiment condition (baseline or degraded)
        gt_file: Output path for ground truth trajectory
        est_file: Output path for estimated trajectory
        post_trajectory_wait: Seconds to wait after trajectory completion
        timeout: Maximum experiment duration
        verbose: Print verbose output

    Returns:
        Tuple of (success, error_message)
    """
    # Build launch command
    launch_cmd = [
        'ros2', 'launch', 'experiment_runner', 'run_realtime.launch.py',
        f'trajectory:={trajectory}',
        f'algorithm:={algorithm}',
        f'condition:={condition}',
        f'gt_output_file:={gt_file}',
        f'est_output_file:={est_file}',
        f'post_trajectory_wait:={post_trajectory_wait}',
        f'timeout:={timeout}',
    ]

    if verbose:
        print(f"[run_realtime] Starting real-time experiment")
        print(f"[run_realtime] Trajectory: {trajectory}")
        print(f"[run_realtime] Algorithm: {algorithm}")
        print(f"[run_realtime] Condition: {condition}")
        print(f"[run_realtime] Command: {' '.join(launch_cmd)}")

    try:
        # Start launch process
        proc = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE if not verbose else None,
            stderr=subprocess.PIPE if not verbose else None,
            preexec_fn=os.setsid  # Create new process group for cleanup
        )

        # Wait for completion with timeout
        # Add extra buffer time beyond the experiment timeout
        effective_timeout = timeout + 60
        start_time = time.time()

        while proc.poll() is None:
            elapsed = time.time() - start_time
            if elapsed > effective_timeout:
                # Kill the process group
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                time.sleep(2)
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                return False, f"Experiment timed out after {elapsed:.1f}s"
            time.sleep(1.0)

        if verbose:
            print(f"[run_realtime] Launch completed with code: {proc.returncode}")

        # Check if it exited cleanly (0 is success from completion_monitor)
        if proc.returncode not in [0, -2, -15]:  # 0, SIGINT, SIGTERM are OK
            return False, f"Launch exited with code {proc.returncode}"

        return True, ""

    except Exception as e:
        return False, f"Error running experiment: {e}"


def verify_trajectory_files(gt_file: str, est_file: str) -> Tuple[bool, str]:
    """
    Verify that trajectory files exist and have content.

    Returns:
        Tuple of (success, error_message)
    """
    for name, filepath in [('Ground truth', gt_file), ('Estimated', est_file)]:
        if not os.path.exists(filepath):
            return False, f"{name} trajectory file not created: {filepath}"

        # Check file has content (more than just header)
        with open(filepath, 'r') as f:
            lines = [l for l in f.readlines() if l.strip() and not l.startswith('#')]
            if len(lines) < 2:
                return False, f"{name} trajectory file has too few poses: {len(lines)}"

    return True, ""


def run_evaluation(
    gt_file: str,
    est_file: str,
    output_dir: str,
    dataset: str,
    algorithm: str,
    save_plots: bool = True,
    verbose: bool = False
) -> Tuple[bool, str, Optional[dict]]:
    """
    Run the evaluation script.

    Returns:
        Tuple of (success, error_message, metrics_dict)
    """
    try:
        evaluate_script = find_script_path('evaluate_run.py')
    except FileNotFoundError as e:
        return False, str(e), None

    # Build evaluation command
    eval_cmd = [
        sys.executable, evaluate_script,
        '--gt_file', gt_file,
        '--est_file', est_file,
        '--output_dir', output_dir,
        '--dataset', dataset,
        '--algorithm', algorithm,
    ]

    if save_plots:
        eval_cmd.append('--save_plots')

    if verbose:
        eval_cmd.append('--verbose')
        print(f"[run_realtime] Running evaluation")
        print(f"[run_realtime] Command: {' '.join(eval_cmd)}")

    try:
        result = subprocess.run(
            eval_cmd,
            capture_output=True,
            text=True,
            timeout=120
        )

        if verbose:
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(result.stderr, file=sys.stderr)

        # Load metrics.json
        metrics_path = os.path.join(output_dir, 'metrics.json')
        if os.path.exists(metrics_path):
            with open(metrics_path, 'r') as f:
                metrics = json.load(f)
            return metrics.get('success', False), metrics.get('error', ''), metrics
        else:
            return False, "metrics.json not created", None

    except subprocess.TimeoutExpired:
        return False, "Evaluation script timed out", None
    except Exception as e:
        return False, f"Evaluation error: {e}", None


def main():
    parser = argparse.ArgumentParser(
        description='Run a single real-time SLAM experiment with evaluation.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --trajectory traj_01_easy.csv --algorithm slam_toolbox
  %(prog)s --trajectory traj_02_loop.csv --algorithm cartographer --condition degraded
  %(prog)s --trajectory /full/path/to/traj.csv --algorithm slam_toolbox --verbose
        """
    )

    parser.add_argument(
        '--trajectory',
        type=str,
        required=True,
        help='Trajectory CSV filename or full path'
    )

    parser.add_argument(
        '--algorithm',
        type=str,
        required=True,
        choices=VALID_ALGORITHMS,
        help='SLAM algorithm to use'
    )

    parser.add_argument(
        '--condition',
        type=str,
        default='baseline',
        choices=VALID_CONDITIONS,
        help='Experiment condition (default: baseline)'
    )

    parser.add_argument(
        '--output_dir',
        type=str,
        default=None,
        help='Output directory for results (default: auto-generated)'
    )

    parser.add_argument(
        '--post_trajectory_wait',
        type=float,
        default=5.0,
        help='Seconds to wait after trajectory completion (default: 5.0)'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=600.0,
        help='Maximum experiment duration in seconds (default: 600)'
    )

    parser.add_argument(
        '--save_plots',
        action='store_true',
        default=True,
        help='Save visualization plots (default: True)'
    )

    parser.add_argument(
        '--no_plots',
        action='store_true',
        help='Disable plot generation'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Validate trajectory
    try:
        trajectory_path = find_trajectory_file(args.trajectory)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Extract trajectory name for output directory
    traj_name = extract_trajectory_name(args.trajectory)

    # Generate output directory with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    if args.output_dir:
        output_dir = os.path.expanduser(args.output_dir)
    else:
        output_dir = os.path.expanduser(
            f'~/thesis/ros2_ws/results/realtime/{traj_name}__{args.condition}__{args.algorithm}__{timestamp}'
        )

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Set up file paths
    gt_file = os.path.join(output_dir, 'gt_trajectory.tum')
    est_file = os.path.join(output_dir, 'est_trajectory.tum')
    save_plots = args.save_plots and not args.no_plots

    # Print configuration
    print(f"\n{'='*60}")
    print(f"Real-Time SLAM Experiment")
    print(f"{'='*60}")
    print(f"Trajectory: {traj_name}")
    print(f"Algorithm:  {args.algorithm}")
    print(f"Condition:  {args.condition}")
    print(f"Output:     {output_dir}")
    print(f"{'='*60}\n")

    # Run the experiment
    print(f"[run_realtime] Starting experiment...")
    success, error = run_realtime_experiment(
        trajectory=args.trajectory,
        algorithm=args.algorithm,
        condition=args.condition,
        gt_file=gt_file,
        est_file=est_file,
        post_trajectory_wait=args.post_trajectory_wait,
        timeout=args.timeout,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_realtime] Experiment failed: {error}", file=sys.stderr)
        # Create failure metrics.json
        failure_result = {
            'success': False,
            'dataset': f'{traj_name}__{args.condition}',
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"Experiment failed: {error}",
            'ate': None,
            'rpe': None,
        }
        with open(os.path.join(output_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    # Verify trajectory files
    success, error = verify_trajectory_files(gt_file, est_file)
    if not success:
        print(f"[run_realtime] Trajectory verification failed: {error}", file=sys.stderr)
        failure_result = {
            'success': False,
            'dataset': f'{traj_name}__{args.condition}',
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"Trajectory export failed: {error}",
            'ate': None,
            'rpe': None,
        }
        with open(os.path.join(output_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"[run_realtime] Trajectory files verified")
        print(f"[run_realtime]   GT: {gt_file}")
        print(f"[run_realtime]   EST: {est_file}")

    # Run evaluation
    print(f"[run_realtime] Running evaluation...")
    success, error, metrics = run_evaluation(
        gt_file=gt_file,
        est_file=est_file,
        output_dir=output_dir,
        dataset=f'{traj_name}__{args.condition}',
        algorithm=args.algorithm,
        save_plots=save_plots,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_realtime] Evaluation failed: {error}", file=sys.stderr)
        sys.exit(1)

    # Print summary
    print(f"\n{'='*60}")
    print(f"RESULTS: {traj_name} / {args.algorithm} / {args.condition}")
    print(f"{'='*60}")
    print(f"  ATE RMSE: {metrics['ate']['rmse']:.4f} m")
    print(f"  ATE Mean: {metrics['ate']['mean']:.4f} m")
    print(f"  RPE RMSE: {metrics['rpe']['rmse']:.4f} m")
    print(f"  RPE Mean: {metrics['rpe']['mean']:.4f} m")
    print(f"  Trajectory: {metrics['trajectory_length_m']:.2f} m over {metrics['duration_s']:.2f} s")
    print(f"  Poses: {metrics['num_poses']}")
    print(f"{'='*60}")
    print(f"Results saved to: {output_dir}")

    sys.exit(0)


if __name__ == '__main__':
    main()
