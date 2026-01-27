#!/usr/bin/env python3
"""
Batch experiment runner for real-time SLAM evaluation.

Discovers all trajectory CSV files and runs SLAM experiments with specified algorithms.
Runs experiments sequentially (Gazebo can only run one instance at a time).

Usage:
    python3 run_all_realtime.py --trajectories_dir ~/thesis/trajectories
    python3 run_all_realtime.py --trajectories_dir /path/to/trajectories --algorithms slam_toolbox
    python3 run_all_realtime.py --trajectories_dir /path/to/trajectories --conditions baseline degraded

Output:
    For each trajectory, algorithm, and condition combination:
    - results/realtime/{traj}__{condition}__{algorithm}__{timestamp}/
        - gt_trajectory.tum
        - est_trajectory.tum
        - metrics.json
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def discover_trajectories(trajectories_dir: str) -> List[str]:
    """
    Discover all trajectory CSV files in the given directory.

    Args:
        trajectories_dir: Directory containing trajectory CSV files

    Returns:
        List of trajectory filenames (not full paths)
    """
    trajectories_dir = os.path.expanduser(trajectories_dir)
    trajectories_dir = os.path.abspath(trajectories_dir)

    if not os.path.isdir(trajectories_dir):
        return []

    trajectories = []
    for entry in sorted(os.listdir(trajectories_dir)):
        if entry.endswith('.csv'):
            trajectories.append(entry)

    return trajectories


def run_single_experiment(
    trajectory: str,
    algorithm: str,
    condition: str,
    timeout: float,
    verbose: bool
) -> Tuple[str, str, str, bool, Optional[dict], str, str]:
    """
    Run a single real-time experiment.

    Returns:
        Tuple of (trajectory, algorithm, condition, success, metrics, error_message, output_dir)
    """
    try:
        run_realtime_script = find_script_path('run_realtime.py')
    except FileNotFoundError as e:
        return trajectory, algorithm, condition, False, None, str(e), ""

    cmd = [
        sys.executable, run_realtime_script,
        '--trajectory', trajectory,
        '--algorithm', algorithm,
        '--condition', condition,
        '--timeout', str(timeout),
    ]

    if verbose:
        cmd.append('--verbose')

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 120  # Extra buffer for evaluation
        )

        # Parse output to find output directory
        output_dir = ""
        for line in result.stdout.split('\n'):
            if 'Results saved to:' in line:
                output_dir = line.split('Results saved to:')[1].strip()
                break

        # Check for metrics.json
        metrics = None
        if output_dir:
            metrics_path = os.path.join(output_dir, 'metrics.json')
            if os.path.exists(metrics_path):
                with open(metrics_path, 'r') as f:
                    metrics = json.load(f)

        success = result.returncode == 0 and metrics is not None and metrics.get('success', False)
        error = '' if success else (metrics.get('error', '') if metrics else result.stderr[:500])

        return trajectory, algorithm, condition, success, metrics, error, output_dir

    except subprocess.TimeoutExpired:
        return trajectory, algorithm, condition, False, None, "Experiment timed out", ""
    except Exception as e:
        return trajectory, algorithm, condition, False, None, str(e), ""


def format_duration(seconds: float) -> str:
    """Format seconds as human-readable duration."""
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        mins = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{mins}m {secs}s"
    else:
        hours = int(seconds // 3600)
        mins = int((seconds % 3600) // 60)
        return f"{hours}h {mins}m"


def main():
    parser = argparse.ArgumentParser(
        description='Run batch real-time SLAM experiments.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --trajectories_dir ~/thesis/trajectories
  %(prog)s --trajectories_dir /path/to/trajectories --algorithms slam_toolbox
  %(prog)s --trajectories_dir /path/to/trajectories --conditions baseline degraded
  %(prog)s --trajectories_dir /path/to/trajectories --filter traj_01
        """
    )

    parser.add_argument(
        '--trajectories_dir',
        type=str,
        default=os.path.expanduser('~/thesis/trajectories'),
        help='Directory containing trajectory CSV files (default: ~/thesis/trajectories)'
    )

    parser.add_argument(
        '--algorithms',
        type=str,
        nargs='+',
        default=VALID_ALGORITHMS,
        choices=VALID_ALGORITHMS,
        help=f'SLAM algorithms to run (default: all)'
    )

    parser.add_argument(
        '--conditions',
        type=str,
        nargs='+',
        default=['baseline'],
        choices=VALID_CONDITIONS,
        help='Experiment conditions to run (default: baseline)'
    )

    parser.add_argument(
        '--filter',
        type=str,
        default=None,
        help='Only process trajectories whose names contain this string'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=600.0,
        help='Maximum time per experiment in seconds (default: 600)'
    )

    parser.add_argument(
        '--output_dir',
        type=str,
        default=os.path.expanduser('~/thesis/ros2_ws/results/realtime'),
        help='Output directory for results'
    )

    parser.add_argument(
        '--dry_run',
        action='store_true',
        help='Show what would be run without executing'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Validate trajectories directory
    trajectories_dir = os.path.expanduser(args.trajectories_dir)
    trajectories_dir = os.path.abspath(trajectories_dir)

    if not os.path.isdir(trajectories_dir):
        print(f"Error: Trajectories directory does not exist: {trajectories_dir}", file=sys.stderr)
        sys.exit(1)

    # Discover trajectories
    trajectories = discover_trajectories(trajectories_dir)

    if not trajectories:
        print(f"Error: No trajectory CSV files found in {trajectories_dir}", file=sys.stderr)
        sys.exit(1)

    # Apply filter if specified
    if args.filter:
        trajectories = [t for t in trajectories if args.filter in t]
        if not trajectories:
            print(f"Error: No trajectories match filter '{args.filter}'", file=sys.stderr)
            sys.exit(1)

    # Build experiment list
    experiments = []
    for trajectory in trajectories:
        for algorithm in args.algorithms:
            for condition in args.conditions:
                experiments.append((trajectory, algorithm, condition))

    if not experiments:
        print("No experiments to run")
        sys.exit(0)

    # Print experiment plan
    print(f"\n{'='*70}")
    print(f"Real-Time SLAM Batch Experiment Runner")
    print(f"{'='*70}")
    print(f"Trajectories dir: {trajectories_dir}")
    print(f"Trajectories:     {len(trajectories)}")
    print(f"Algorithms:       {', '.join(args.algorithms)}")
    print(f"Conditions:       {', '.join(args.conditions)}")
    print(f"Experiments:      {len(experiments)}")
    print(f"Timeout:          {args.timeout}s per experiment")
    print(f"{'='*70}")

    if args.dry_run:
        print("\nExperiments to run:")
        for trajectory, algorithm, condition in experiments:
            traj_name = Path(trajectory).stem
            print(f"  - {traj_name} / {algorithm} / {condition}")
        print(f"\nTotal: {len(experiments)} experiments")
        print("(Dry run - no experiments executed)")
        sys.exit(0)

    # Confirm execution
    print(f"\nStarting {len(experiments)} experiments (sequential - Gazebo limitation)...")
    print()

    # Track results
    start_time = time.time()
    results: List[Tuple[str, str, str, bool, Optional[dict], str, str]] = []
    completed = 0
    failed = 0

    # Sequential execution only (Gazebo can't run multiple instances)
    for trajectory, algorithm, condition in experiments:
        traj_name = Path(trajectory).stem
        print(f"[{completed + 1}/{len(experiments)}] Running: {traj_name} / {algorithm} / {condition}...")

        result = run_single_experiment(
            trajectory, algorithm, condition, args.timeout, args.verbose
        )
        results.append(result)
        trajectory, algorithm, condition, success, metrics, error, output_dir = result
        completed += 1

        if success:
            ate_rmse = metrics['ate']['rmse'] if metrics else 0
            print(f"[{completed}/{len(experiments)}] PASS: {traj_name} / {algorithm} / {condition} "
                  f"(ATE RMSE: {ate_rmse:.4f}m)")
        else:
            failed += 1
            print(f"[{completed}/{len(experiments)}] FAIL: {traj_name} / {algorithm} / {condition} "
                  f"- {error[:60]}")

    # Calculate elapsed time
    elapsed = time.time() - start_time

    # Print summary
    print(f"\n{'='*70}")
    print(f"BATCH RESULTS SUMMARY")
    print(f"{'='*70}")
    print(f"Total experiments: {len(experiments)}")
    print(f"Passed:            {completed - failed}")
    print(f"Failed:            {failed}")
    print(f"Elapsed time:      {format_duration(elapsed)}")
    print(f"{'='*70}")

    # Print detailed results table
    print(f"\nDetailed Results:")
    print(f"{'-'*80}")
    print(f"{'Trajectory':<25} {'Algorithm':<15} {'Condition':<10} {'ATE RMSE':>10} {'Status':>8}")
    print(f"{'-'*80}")

    for trajectory, algorithm, condition, success, metrics, error, output_dir in results:
        traj_name = Path(trajectory).stem[:24]
        if success and metrics:
            ate_rmse = f"{metrics['ate']['rmse']:.4f}m"
            status = "PASS"
        else:
            ate_rmse = "-"
            status = "FAIL"
        print(f"{traj_name:<25} {algorithm:<15} {condition:<10} {ate_rmse:>10} {status:>8}")

    print(f"{'-'*80}")

    # Save batch summary
    output_dir = os.path.expanduser(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    summary = {
        'timestamp': datetime.now().isoformat(),
        'trajectories_dir': trajectories_dir,
        'algorithms': args.algorithms,
        'conditions': args.conditions,
        'total_experiments': len(experiments),
        'passed': completed - failed,
        'failed': failed,
        'elapsed_seconds': elapsed,
        'results': [
            {
                'trajectory': trajectory,
                'algorithm': algorithm,
                'condition': condition,
                'success': success,
                'ate_rmse': metrics['ate']['rmse'] if success and metrics else None,
                'error': error if not success else None,
                'output_dir': result_output_dir,
            }
            for trajectory, algorithm, condition, success, metrics, error, result_output_dir in results
        ]
    }

    summary_path = os.path.join(output_dir, 'batch_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\nBatch summary saved to: {summary_path}")

    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
