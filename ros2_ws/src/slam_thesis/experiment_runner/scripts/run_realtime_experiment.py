#!/usr/bin/env python3
"""
Real-time SLAM experiment runner with post-hoc trajectory extraction.

Runs simulation + SLAM live, records all relevant topics to a bag,
then extracts trajectories from the bag for evaluation.

This approach avoids real-time TF lookup issues while still running
SLAM in real-time (not from bag replay).

Usage:
    python3 run_realtime_experiment.py --trajectory traj_01_easy.csv --algorithm slam_toolbox
    python3 run_realtime_experiment.py --trajectory traj_02_loop.csv --algorithm cartographer --verbose
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


def find_script_path(script_name):
    """Find script in experiment_runner package."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(script_dir, script_name)
    if os.path.exists(script_path):
        return script_path
    raise FileNotFoundError(f"Could not find script: {script_name}")


def run_experiment(trajectory, algorithm, output_dir, timeout=600.0, verbose=False):
    """
    Run real-time SLAM experiment with bag recording.

    Returns: (success, error_message, result_dir)
    """
    # Create result directory
    traj_name = os.path.basename(trajectory).replace('.csv', '')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    result_dir = os.path.join(output_dir, f'{traj_name}_{algorithm}_{timestamp}')
    os.makedirs(result_dir, exist_ok=True)

    bag_path = os.path.join(result_dir, 'recording')

    if verbose:
        print(f"[run_realtime] Starting real-time SLAM experiment")
        print(f"[run_realtime] Algorithm: {algorithm}")
        print(f"[run_realtime] Trajectory: {trajectory}")
        print(f"[run_realtime] Output dir: {result_dir}")

    # Build launch command for simulation + SLAM (without trajectory exporters)
    launch_cmd = [
        'ros2', 'launch', 'slam_launch', 'realtime_slam.launch.py',
        f'algorithm:={algorithm}',
        f'trajectory:={trajectory}',
        f'bag_output:={bag_path}',
        'use_sim_time:=true',
    ]

    if verbose:
        print(f"[run_realtime] Command: {' '.join(launch_cmd)}")

    proc = subprocess.Popen(
        launch_cmd,
        stdout=None if verbose else subprocess.PIPE,
        stderr=None if verbose else subprocess.PIPE,
    )

    try:
        start_time = time.time()
        while proc.poll() is None:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                terminate_process_tree(proc)
                return False, f"Experiment timed out after {timeout}s", result_dir
            time.sleep(1.0)

        if verbose:
            print(f"[run_realtime] Launch process exited with code: {proc.returncode}")

        return True, "", result_dir

    except Exception as e:
        terminate_process_tree(proc)
        return False, f"Error: {e}", result_dir


def extract_trajectories(result_dir, verbose=False):
    """
    Extract GT and EST trajectories from recorded bag.

    Returns: (success, error_message)
    """
    bag_path = os.path.join(result_dir, 'recording')
    gt_file = os.path.join(result_dir, 'gt_trajectory.tum')
    est_file = os.path.join(result_dir, 'est_trajectory.tum')

    # Check if bag exists
    if not os.path.exists(bag_path):
        return False, f"Bag not found: {bag_path}"

    try:
        bag_to_tum = find_script_path('bag_to_tum.py')
    except FileNotFoundError as e:
        return False, str(e)

    # Extract ground truth from /gt_pose topic
    if verbose:
        print(f"[run_realtime] Extracting ground truth trajectory...")

    gt_cmd = [
        sys.executable, bag_to_tum,
        '--bag', bag_path,
        '--output', gt_file,
        '--topic', '/gt_pose',
    ]

    result = subprocess.run(gt_cmd, capture_output=True, text=True)
    if verbose and result.stdout:
        print(result.stdout)
    if result.returncode != 0:
        return False, f"GT extraction failed: {result.stderr}"

    # Extract estimated trajectory from TF chain: map -> odom -> base_footprint
    if verbose:
        print(f"[run_realtime] Extracting estimated trajectory...")

    est_cmd = [
        sys.executable, bag_to_tum,
        '--bag', bag_path,
        '--output', est_file,
        '--chain', 'map,odom,base_footprint',
    ]

    result = subprocess.run(est_cmd, capture_output=True, text=True)
    if verbose and result.stdout:
        print(result.stdout)
    if result.returncode != 0:
        # Try direct map -> base_footprint as fallback
        if verbose:
            print(f"[run_realtime] Chain extraction failed, trying direct transform...")
        est_cmd = [
            sys.executable, bag_to_tum,
            '--bag', bag_path,
            '--output', est_file,
            '--parent_frame', 'map',
            '--child_frame', 'base_footprint',
        ]
        result = subprocess.run(est_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            return False, f"EST extraction failed: {result.stderr}"

    # Verify files have content
    for name, filepath in [('GT', gt_file), ('EST', est_file)]:
        if not os.path.exists(filepath):
            return False, f"{name} file not created"
        with open(filepath, 'r') as f:
            lines = [l for l in f.readlines() if l.strip() and not l.startswith('#')]
            if len(lines) < 10:
                return False, f"{name} has too few poses: {len(lines)}"

    return True, ""


def run_evaluation(result_dir, dataset, algorithm, verbose=False):
    """
    Run evaluation using evo metrics.

    Returns: (success, error_message, metrics)
    """
    gt_file = os.path.join(result_dir, 'gt_trajectory.tum')
    est_file = os.path.join(result_dir, 'est_trajectory.tum')

    try:
        evaluate_script = find_script_path('evaluate_run.py')
    except FileNotFoundError as e:
        return False, str(e), None

    eval_cmd = [
        sys.executable, evaluate_script,
        '--gt_file', gt_file,
        '--est_file', est_file,
        '--output_dir', result_dir,
        '--dataset', dataset,
        '--algorithm', algorithm,
        '--save_plots',
    ]

    if verbose:
        eval_cmd.append('--verbose')
        print(f"[run_realtime] Running evaluation...")

    result = subprocess.run(eval_cmd, capture_output=True, text=True, timeout=120)

    if verbose:
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)

    metrics_path = os.path.join(result_dir, 'metrics.json')
    if os.path.exists(metrics_path):
        with open(metrics_path, 'r') as f:
            metrics = json.load(f)
        return metrics.get('success', False), metrics.get('error', ''), metrics

    return False, "metrics.json not created", None


def main():
    parser = argparse.ArgumentParser(
        description='Run real-time SLAM experiment with post-hoc trajectory extraction.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--trajectory', required=True, help='Trajectory CSV filename or path')
    parser.add_argument('--algorithm', required=True, choices=VALID_ALGORITHMS, help='SLAM algorithm')
    parser.add_argument('--output_dir', default=os.path.expanduser('~/thesis/ros2_ws/results'),
                        help='Output directory (default: ~/thesis/ros2_ws/results)')
    parser.add_argument('--timeout', type=float, default=600.0, help='Max time in seconds')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    dataset_name = os.path.basename(args.trajectory).replace('.csv', '')

    if args.verbose:
        print(f"\n{'='*60}")
        print(f"Real-time SLAM Experiment: {dataset_name}")
        print(f"{'='*60}")
        print(f"Trajectory: {args.trajectory}")
        print(f"Algorithm:  {args.algorithm}")
        print(f"Timeout:    {args.timeout}s")
        print(f"Output:     {args.output_dir}")
        print(f"{'='*60}\n")

    # Step 1: Run experiment
    print(f"[run_realtime] Running {args.algorithm} on {dataset_name}...")
    success, error, result_dir = run_experiment(
        trajectory=args.trajectory,
        algorithm=args.algorithm,
        output_dir=args.output_dir,
        timeout=args.timeout,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_realtime] Experiment failed: {error}", file=sys.stderr)
        sys.exit(1)

    print(f"[run_realtime] Experiment completed, result dir: {result_dir}")

    # Step 2: Extract trajectories from bag
    print(f"[run_realtime] Extracting trajectories from bag...")
    success, error = extract_trajectories(result_dir, verbose=args.verbose)

    if not success:
        print(f"[run_realtime] Trajectory extraction failed: {error}", file=sys.stderr)
        # Save failure metrics
        failure_result = {
            'success': False,
            'dataset': dataset_name,
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"Trajectory extraction failed: {error}",
        }
        with open(os.path.join(result_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"[run_realtime] Trajectories extracted successfully")

    # Step 3: Run evaluation
    print(f"[run_realtime] Running evaluation...")
    success, error, metrics = run_evaluation(
        result_dir=result_dir,
        dataset=dataset_name,
        algorithm=args.algorithm,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_realtime] Evaluation failed: {error}", file=sys.stderr)
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
