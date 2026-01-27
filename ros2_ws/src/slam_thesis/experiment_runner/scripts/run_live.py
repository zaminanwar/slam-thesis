#!/usr/bin/env python3
"""
Live SLAM experiment runner.

Runs a single live SLAM experiment without rosbag recording/replay.
Launches simulation + SLAM + trajectory export, waits for completion,
then runs evaluation.

This is an alternative to the record_dataset.launch.py + run_one.py workflow.

Usage:
    python3 run_live.py --trajectory traj_01_easy.csv --algorithm slam_toolbox
    python3 run_live.py --trajectory traj_02_loop.csv --algorithm cartographer --verbose
    python3 run_live.py --trajectory /path/to/traj.csv --algorithm slam_toolbox --output_dir /path/to/results

Output:
    Creates in output_dir/<trajectory>_<algorithm>_<timestamp>/:
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


def terminate_process_tree(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    """
    Safely terminate a process and its children.
    """
    if proc.poll() is not None:
        return  # Already terminated

    try:
        # First try graceful termination with SIGINT
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Try SIGTERM
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # Force kill
                proc.kill()
                proc.wait(timeout=2.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def run_live_slam(
    trajectory: str,
    algorithm: str,
    output_dir: str,
    timeout: float = 600.0,
    verbose: bool = False
) -> Tuple[bool, str, str]:
    """
    Run live SLAM experiment.

    Args:
        trajectory: Trajectory CSV filename or path
        algorithm: SLAM algorithm (slam_toolbox or cartographer)
        output_dir: Base output directory
        timeout: Maximum time in seconds
        verbose: Print verbose output

    Returns:
        Tuple of (success, error_message, result_dir)
    """
    # Build launch command
    launch_cmd = [
        'ros2', 'launch', 'slam_launch', 'live_slam.launch.py',
        f'algorithm:={algorithm}',
        f'trajectory:={trajectory}',
        f'output_dir:={output_dir}',
        'use_sim_time:=true',
    ]

    if verbose:
        print(f"[run_live] Starting live SLAM experiment")
        print(f"[run_live] Algorithm: {algorithm}")
        print(f"[run_live] Trajectory: {trajectory}")
        print(f"[run_live] Output dir: {output_dir}")
        print(f"[run_live] Command: {' '.join(launch_cmd)}")

    # Start the launch process
    proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE if not verbose else None,
        stderr=subprocess.PIPE if not verbose else None,
    )

    try:
        # Wait for completion
        start_time = time.time()
        while proc.poll() is None:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                terminate_process_tree(proc)
                return False, f"Experiment timed out after {timeout}s", ""
            time.sleep(1.0)

        if verbose:
            print(f"[run_live] Launch process exited with code: {proc.returncode}")

        # Find the result directory (most recent in output_dir)
        traj_name = os.path.basename(trajectory).replace('.csv', '')
        result_dirs = sorted(
            [d for d in Path(output_dir).iterdir()
             if d.is_dir() and d.name.startswith(f"{traj_name}_{algorithm}_")],
            key=lambda x: x.stat().st_mtime,
            reverse=True
        )

        if not result_dirs:
            return False, "No result directory found", ""

        result_dir = str(result_dirs[0])

        return True, "", result_dir

    except Exception as e:
        terminate_process_tree(proc)
        return False, f"Error: {e}", ""


def verify_trajectory_files(result_dir: str) -> Tuple[bool, str]:
    """
    Verify that trajectory files exist and have content.

    Returns:
        Tuple of (success, error_message)
    """
    gt_file = os.path.join(result_dir, 'gt_trajectory.tum')
    est_file = os.path.join(result_dir, 'est_trajectory.tum')

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
    result_dir: str,
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
    gt_file = os.path.join(result_dir, 'gt_trajectory.tum')
    est_file = os.path.join(result_dir, 'est_trajectory.tum')

    try:
        evaluate_script = find_script_path('evaluate_run.py')
    except FileNotFoundError as e:
        return False, str(e), None

    # Build evaluation command
    eval_cmd = [
        sys.executable, evaluate_script,
        '--gt_file', gt_file,
        '--est_file', est_file,
        '--output_dir', result_dir,
        '--dataset', dataset,
        '--algorithm', algorithm,
    ]

    if save_plots:
        eval_cmd.append('--save_plots')

    if verbose:
        eval_cmd.append('--verbose')
        print(f"[run_live] Running evaluation")
        print(f"[run_live] Command: {' '.join(eval_cmd)}")

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
        metrics_path = os.path.join(result_dir, 'metrics.json')
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
        description='Run a live SLAM experiment (no rosbag recording/replay).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --trajectory traj_01_easy.csv --algorithm slam_toolbox
  %(prog)s --trajectory traj_02_loop.csv --algorithm cartographer --verbose
  %(prog)s --trajectory /path/to/traj.csv --algorithm slam_toolbox --output_dir /path/to/results
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
        '--output_dir',
        type=str,
        default=os.path.expanduser('~/thesis/ros2_ws/results'),
        help='Base output directory for results (default: ~/thesis/ros2_ws/results)'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=600.0,
        help='Maximum time for experiment in seconds (default: 600)'
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

    # Ensure output directory exists
    os.makedirs(args.output_dir, exist_ok=True)

    save_plots = args.save_plots and not args.no_plots
    dataset_name = os.path.basename(args.trajectory).replace('.csv', '')

    # Print configuration
    if args.verbose:
        print(f"\n{'='*60}")
        print(f"Live SLAM Experiment: {dataset_name}")
        print(f"{'='*60}")
        print(f"Trajectory: {args.trajectory}")
        print(f"Algorithm:  {args.algorithm}")
        print(f"Timeout:    {args.timeout}s")
        print(f"Output:     {args.output_dir}")
        print(f"{'='*60}\n")

    # Run live SLAM experiment
    print(f"[run_live] Running {args.algorithm} on {dataset_name}...")
    success, error, result_dir = run_live_slam(
        trajectory=args.trajectory,
        algorithm=args.algorithm,
        output_dir=args.output_dir,
        timeout=args.timeout,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_live] Experiment failed: {error}", file=sys.stderr)
        sys.exit(1)

    if args.verbose:
        print(f"[run_live] Result directory: {result_dir}")

    # Verify trajectory files
    success, error = verify_trajectory_files(result_dir)
    if not success:
        print(f"[run_live] Trajectory verification failed: {error}", file=sys.stderr)
        # Create failure metrics.json
        failure_result = {
            'success': False,
            'dataset': dataset_name,
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"Trajectory export failed: {error}",
            'ate': None,
            'rpe': None,
        }
        with open(os.path.join(result_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"[run_live] Trajectory files verified")

    # Run evaluation
    print(f"[run_live] Running evaluation...")
    success, error, metrics = run_evaluation(
        result_dir=result_dir,
        dataset=dataset_name,
        algorithm=args.algorithm,
        save_plots=save_plots,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_live] Evaluation failed: {error}", file=sys.stderr)
        sys.exit(1)

    # Print summary
    print(f"\n{'='*60}")
    print(f"RESULTS: {dataset_name} / {args.algorithm}")
    print(f"{'='*60}")
    print(f"  ATE RMSE: {metrics['ate']['rmse']:.4f} m")
    print(f"  ATE Mean: {metrics['ate']['mean']:.4f} m")
    print(f"  RPE RMSE: {metrics['rpe']['rmse']:.4f} m")
    print(f"  RPE Mean: {metrics['rpe']['mean']:.4f} m")
    print(f"  Trajectory: {metrics['trajectory_length_m']:.2f} m over {metrics['duration_s']:.2f} s")
    print(f"  Poses: {metrics['num_poses']}")
    print(f"{'='*60}")
    print(f"Results saved to: {result_dir}")

    sys.exit(0)


if __name__ == '__main__':
    main()
