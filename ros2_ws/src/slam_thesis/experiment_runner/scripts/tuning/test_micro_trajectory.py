#!/usr/bin/env python3
"""
Quick test to verify the micro-trajectory works with both algorithms.

Run this before starting full hyperparameter tuning to ensure:
1. The micro-trajectory file is valid
2. Both SLAM algorithms can complete it
3. Metrics are properly computed

Usage:
    python3 test_micro_trajectory.py
    python3 test_micro_trajectory.py --algorithm slam_toolbox
    python3 test_micro_trajectory.py --algorithm cartographer
"""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


THESIS_DIR = Path.home() / 'thesis'
ROS2_WS = THESIS_DIR / 'ros2_ws'
TRAJECTORIES_DIR = THESIS_DIR / 'trajectories'

MICRO_TRAJECTORY = 'traj_micro_tune'
TIMEOUT = 90  # seconds


def run_test(algorithm: str) -> dict:
    """Run a single test with the micro-trajectory."""
    print(f"\n{'='*60}")
    print(f"Testing {algorithm} with micro-trajectory")
    print(f"{'='*60}")

    # Check trajectory exists
    traj_file = TRAJECTORIES_DIR / f'{MICRO_TRAJECTORY}.csv'
    if not traj_file.exists():
        print(f"ERROR: Trajectory file not found: {traj_file}")
        return {'status': 'failed', 'error': 'trajectory_not_found'}

    print(f"Trajectory: {traj_file}")

    # Create output directory
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = ROS2_WS / 'results' / 'micro_test' / f'{algorithm}_{timestamp}'
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Output: {output_dir}")

    # Run experiment
    run_one_script = ROS2_WS / 'src/slam_thesis/experiment_runner/scripts/run_one.py'

    cmd = [
        'python3', str(run_one_script),
        '--algo', algorithm,
        '--trajectory', MICRO_TRAJECTORY,
        '--output_dir', str(output_dir),
        '--timeout', str(TIMEOUT),
        '--pose_mode', 'slam',
        '--verbose',
    ]

    print(f"\nRunning: {' '.join(cmd)}")
    print("-" * 60)

    start_time = time.time()
    try:
        result = subprocess.run(
            cmd,
            timeout=TIMEOUT + 120,
        )
        exit_code = result.returncode
    except subprocess.TimeoutExpired:
        print("ERROR: Experiment timed out!")
        return {'status': 'timeout', 'error': 'experiment_timeout'}

    elapsed = time.time() - start_time
    print("-" * 60)
    print(f"Completed in {elapsed:.1f}s with exit code {exit_code}")

    # Load metrics
    metrics_file = output_dir / 'metrics.json'
    if not metrics_file.exists():
        print(f"ERROR: Metrics file not found: {metrics_file}")
        return {'status': 'failed', 'error': 'no_metrics'}

    with open(metrics_file, 'r') as f:
        metrics = json.load(f)

    # Display results
    print(f"\n{'='*60}")
    print(f"RESULTS for {algorithm}")
    print(f"{'='*60}")
    print(f"Status: {metrics.get('status', 'unknown')}")

    if 'ate' in metrics:
        ate = metrics['ate']
        print(f"ATE RMSE: {ate['rmse']:.4f} m ({ate['rmse']*100:.2f} cm)")
        print(f"ATE Mean: {ate['mean']:.4f} m")

    if 'rpe' in metrics:
        rpe = metrics['rpe']
        print(f"RPE RMSE: {rpe['rmse']:.4f} m")

    if 'completion' in metrics:
        comp = metrics['completion']
        print(f"Completion: {comp['completion_rate']*100:.1f}%")
        print(f"Goal reached: {comp['goal_reached']}")

    return {
        'status': metrics.get('status', 'unknown'),
        'ate_rmse': metrics.get('ate', {}).get('rmse'),
        'elapsed': elapsed,
        'output_dir': str(output_dir),
    }


def main():
    parser = argparse.ArgumentParser(description='Test micro-trajectory with SLAM algorithms')
    parser.add_argument(
        '--algorithm', '-a',
        choices=['slam_toolbox', 'cartographer', 'both'],
        default='both',
        help='Algorithm to test (default: both)'
    )
    args = parser.parse_args()

    algorithms = ['slam_toolbox', 'cartographer'] if args.algorithm == 'both' else [args.algorithm]

    results = {}
    for algo in algorithms:
        results[algo] = run_test(algo)
        if algo != algorithms[-1]:
            print("\nWaiting 10s before next test...")
            time.sleep(10)

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")

    all_passed = True
    for algo, res in results.items():
        status = res['status']
        ate = res.get('ate_rmse')
        ate_str = f"{ate*100:.2f}cm" if ate else "N/A"
        elapsed = res.get('elapsed', 0)

        if status == 'success':
            status_str = "PASS"
        else:
            status_str = f"FAIL ({status})"
            all_passed = False

        print(f"  {algo}: {status_str} | ATE: {ate_str} | Time: {elapsed:.1f}s")

    if all_passed:
        print("\nAll tests PASSED! Ready for hyperparameter tuning.")
        print("\nRun full tuning with:")
        print("  cd ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/tuning")
        print("  ./run_tuning.sh both")
        sys.exit(0)
    else:
        print("\nSome tests FAILED. Fix issues before running tuning.")
        sys.exit(1)


if __name__ == '__main__':
    main()
