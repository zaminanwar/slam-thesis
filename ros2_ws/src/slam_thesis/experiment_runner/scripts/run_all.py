#!/usr/bin/env python3
"""
Batch experiment runner for SLAM evaluation.

Discovers all recorded bags and runs SLAM experiments with specified algorithms.
Supports sequential and parallel execution modes.

Usage:
    python3 run_all.py --bags_dir ~/thesis/ros2_ws/bags
    python3 run_all.py --bags_dir /path/to/bags --algorithms slam_toolbox cartographer
    python3 run_all.py --bags_dir /path/to/bags --parallel 2

Output:
    For each bag and algorithm combination:
    - bag_path/results/{algorithm}/metrics.json
    - bag_path/{algorithm}_trajectory.tum
    - bag_path/gt_trajectory.tum
"""

import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def discover_bags(bags_dir: str) -> List[str]:
    """
    Discover all rosbag directories in the given directory.

    Args:
        bags_dir: Directory containing rosbag directories

    Returns:
        List of absolute paths to valid rosbag directories
    """
    bags_dir = os.path.expanduser(bags_dir)
    bags_dir = os.path.abspath(bags_dir)

    if not os.path.isdir(bags_dir):
        return []

    bags = []
    for entry in sorted(os.listdir(bags_dir)):
        bag_path = os.path.join(bags_dir, entry)
        if os.path.isdir(bag_path):
            # Check for metadata.yaml (indicates valid rosbag)
            metadata_file = os.path.join(bag_path, 'metadata.yaml')
            if os.path.exists(metadata_file):
                bags.append(bag_path)

    return bags


def run_single_experiment(
    bag_path: str,
    algorithm: str,
    rate: float,
    timeout: float,
    verbose: bool
) -> Tuple[str, str, bool, Optional[dict], str]:
    """
    Run a single experiment (used for parallel execution).

    Returns:
        Tuple of (bag_path, algorithm, success, metrics, error_message)
    """
    try:
        run_one_script = find_script_path('run_one.py')
    except FileNotFoundError as e:
        return bag_path, algorithm, False, None, str(e)

    cmd = [
        sys.executable, run_one_script,
        '--bag_path', bag_path,
        '--algorithm', algorithm,
        '--rate', str(rate),
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

        # Check for metrics.json
        metrics_path = os.path.join(bag_path, 'results', algorithm, 'metrics.json')
        metrics = None
        if os.path.exists(metrics_path):
            with open(metrics_path, 'r') as f:
                metrics = json.load(f)

        success = result.returncode == 0 and metrics is not None and metrics.get('success', False)
        error = '' if success else (metrics.get('error', '') if metrics else result.stderr[:500])

        return bag_path, algorithm, success, metrics, error

    except subprocess.TimeoutExpired:
        return bag_path, algorithm, False, None, "Experiment timed out"
    except Exception as e:
        return bag_path, algorithm, False, None, str(e)


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
        description='Run batch SLAM experiments.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --bags_dir ~/thesis/ros2_ws/bags
  %(prog)s --bags_dir /path/to/bags --algorithms slam_toolbox
  %(prog)s --bags_dir /path/to/bags --algorithms slam_toolbox cartographer --parallel 2
  %(prog)s --bags_dir /path/to/bags --filter traj_01
        """
    )

    parser.add_argument(
        '--bags_dir',
        type=str,
        required=True,
        help='Directory containing rosbag directories'
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
        '--filter',
        type=str,
        default=None,
        help='Only process bags whose names contain this string'
    )

    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='Playback rate multiplier (default: 1.0)'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=300.0,
        help='Maximum time per experiment in seconds (default: 300)'
    )

    parser.add_argument(
        '--parallel',
        type=int,
        default=1,
        help='Number of parallel experiments (default: 1, sequential)'
    )

    parser.add_argument(
        '--dry_run',
        action='store_true',
        help='Show what would be run without executing'
    )

    parser.add_argument(
        '--skip_existing',
        action='store_true',
        help='Skip experiments that already have results'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Validate bags directory
    bags_dir = os.path.expanduser(args.bags_dir)
    bags_dir = os.path.abspath(bags_dir)

    if not os.path.isdir(bags_dir):
        print(f"Error: Bags directory does not exist: {bags_dir}", file=sys.stderr)
        sys.exit(1)

    # Discover bags
    bags = discover_bags(bags_dir)

    if not bags:
        print(f"Error: No rosbags found in {bags_dir}", file=sys.stderr)
        sys.exit(1)

    # Apply filter if specified
    if args.filter:
        bags = [b for b in bags if args.filter in os.path.basename(b)]
        if not bags:
            print(f"Error: No bags match filter '{args.filter}'", file=sys.stderr)
            sys.exit(1)

    # Build experiment list
    experiments = []
    for bag_path in bags:
        for algorithm in args.algorithms:
            # Check if should skip
            if args.skip_existing:
                metrics_path = os.path.join(bag_path, 'results', algorithm, 'metrics.json')
                if os.path.exists(metrics_path):
                    try:
                        with open(metrics_path, 'r') as f:
                            metrics = json.load(f)
                        if metrics.get('success', False):
                            continue  # Skip successful existing results
                    except Exception:
                        pass  # Run anyway if can't read

            experiments.append((bag_path, algorithm))

    if not experiments:
        print("No experiments to run (all completed or skipped)")
        sys.exit(0)

    # Print experiment plan
    print(f"\n{'='*70}")
    print(f"SLAM Batch Experiment Runner")
    print(f"{'='*70}")
    print(f"Bags directory: {bags_dir}")
    print(f"Bags found:     {len(bags)}")
    print(f"Algorithms:     {', '.join(args.algorithms)}")
    print(f"Experiments:    {len(experiments)}")
    print(f"Parallel:       {args.parallel}")
    print(f"Rate:           {args.rate}x")
    print(f"Timeout:        {args.timeout}s per experiment")
    print(f"{'='*70}")

    if args.dry_run:
        print("\nExperiments to run:")
        for bag_path, algorithm in experiments:
            bag_name = os.path.basename(bag_path)
            print(f"  - {bag_name} / {algorithm}")
        print(f"\nTotal: {len(experiments)} experiments")
        print("(Dry run - no experiments executed)")
        sys.exit(0)

    # Confirm execution
    print(f"\nStarting {len(experiments)} experiments...")
    print()

    # Track results
    start_time = time.time()
    results: List[Tuple[str, str, bool, Optional[dict], str]] = []
    completed = 0
    failed = 0

    if args.parallel > 1:
        # Parallel execution
        with ProcessPoolExecutor(max_workers=args.parallel) as executor:
            futures = {
                executor.submit(
                    run_single_experiment,
                    bag_path, algorithm, args.rate, args.timeout, args.verbose
                ): (bag_path, algorithm)
                for bag_path, algorithm in experiments
            }

            for future in as_completed(futures):
                result = future.result()
                results.append(result)
                bag_path, algorithm, success, metrics, error = result
                bag_name = os.path.basename(bag_path)
                completed += 1

                if success:
                    ate_rmse = metrics['ate']['rmse'] if metrics else 0
                    print(f"[{completed}/{len(experiments)}] PASS: {bag_name} / {algorithm} "
                          f"(ATE RMSE: {ate_rmse:.4f}m)")
                else:
                    failed += 1
                    print(f"[{completed}/{len(experiments)}] FAIL: {bag_name} / {algorithm} "
                          f"- {error[:60]}")
    else:
        # Sequential execution
        for bag_path, algorithm in experiments:
            bag_name = os.path.basename(bag_path)
            print(f"[{completed + 1}/{len(experiments)}] Running: {bag_name} / {algorithm}...")

            result = run_single_experiment(
                bag_path, algorithm, args.rate, args.timeout, args.verbose
            )
            results.append(result)
            bag_path, algorithm, success, metrics, error = result
            completed += 1

            if success:
                ate_rmse = metrics['ate']['rmse'] if metrics else 0
                print(f"[{completed}/{len(experiments)}] PASS: {bag_name} / {algorithm} "
                      f"(ATE RMSE: {ate_rmse:.4f}m)")
            else:
                failed += 1
                print(f"[{completed}/{len(experiments)}] FAIL: {bag_name} / {algorithm} "
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
    print(f"{'-'*70}")
    print(f"{'Dataset':<35} {'Algorithm':<15} {'ATE RMSE':>10} {'Status':>8}")
    print(f"{'-'*70}")

    for bag_path, algorithm, success, metrics, error in results:
        bag_name = os.path.basename(bag_path)[:34]
        if success and metrics:
            ate_rmse = f"{metrics['ate']['rmse']:.4f}m"
            status = "PASS"
        else:
            ate_rmse = "-"
            status = "FAIL"
        print(f"{bag_name:<35} {algorithm:<15} {ate_rmse:>10} {status:>8}")

    print(f"{'-'*70}")

    # Save batch summary
    summary = {
        'timestamp': datetime.now().isoformat(),
        'bags_dir': bags_dir,
        'algorithms': args.algorithms,
        'total_experiments': len(experiments),
        'passed': completed - failed,
        'failed': failed,
        'elapsed_seconds': elapsed,
        'results': [
            {
                'bag_path': bag_path,
                'algorithm': algorithm,
                'success': success,
                'ate_rmse': metrics['ate']['rmse'] if success and metrics else None,
                'error': error if not success else None,
            }
            for bag_path, algorithm, success, metrics, error in results
        ]
    }

    summary_path = os.path.join(bags_dir, 'batch_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\nBatch summary saved to: {summary_path}")

    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
