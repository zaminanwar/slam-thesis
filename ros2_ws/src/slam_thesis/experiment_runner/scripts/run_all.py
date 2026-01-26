#!/usr/bin/env python3
"""
run_all.py - Batch experiment runner.

Runs SLAM evaluation experiments for all combinations of:
- Algorithms: slam_toolbox, cartographer
- Trajectories: all CSV files in ~/thesis/trajectories/

Usage:
    python3 run_all.py
    python3 run_all.py --algorithms slam_toolbox
    python3 run_all.py --trajectories traj_01_easy traj_02_loop
    python3 run_all.py --output_dir ~/thesis/ros2_ws/results/batch_01

Output:
    results/{batch_timestamp}/
    ├── slam_toolbox_traj_01_easy/
    │   ├── gt.tum
    │   ├── est.tum
    │   ├── metrics.json
    │   └── run_info.json
    ├── slam_toolbox_traj_02_loop/
    │   └── ...
    ├── cartographer_traj_01_easy/
    │   └── ...
    └── batch_summary.json
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional


# Constants
DEFAULT_ALGORITHMS = ['slam_toolbox', 'cartographer']
DEFAULT_TRAJECTORY_DIR = Path.home() / 'thesis' / 'trajectories'
DEFAULT_RESULTS_DIR = Path.home() / 'thesis' / 'ros2_ws' / 'results'
DEFAULT_TIMEOUT = 300  # seconds per experiment (5 min for longer trajectories)
COOLDOWN_BETWEEN_RUNS = 5  # seconds between experiments


def discover_trajectories(trajectory_dir: Path) -> list[str]:
    """Discover all trajectory CSV files."""
    trajectories = []
    for csv_file in sorted(trajectory_dir.glob('*.csv')):
        trajectories.append(csv_file.stem)
    return trajectories


def run_single_experiment(
    algo: str,
    trajectory: str,
    pose_mode: str,
    output_dir: Path,
    timeout: float,
    speed_scale: float,
    verbose: bool
) -> dict:
    """
    Run a single experiment using run_one.py.

    Returns:
        dict with experiment result
    """
    run_one_script = Path(__file__).parent / 'run_one.py'

    cmd = [
        'python3', str(run_one_script),
        '--algo', algo,
        '--trajectory', trajectory,
        '--pose_mode', pose_mode,
        '--output_dir', str(output_dir),
        '--timeout', str(timeout),
        '--speed_scale', str(speed_scale)
    ]

    if verbose:
        cmd.append('--verbose')

    start_time = time.time()

    try:
        result = subprocess.run(
            cmd,
            capture_output=not verbose,  # Show output if verbose
            text=True,
            timeout=timeout + 120  # Extra buffer for startup/cleanup
        )

        elapsed = time.time() - start_time

        # Load run_info.json if available
        run_info_file = output_dir / 'run_info.json'
        if run_info_file.exists():
            with open(run_info_file, 'r') as f:
                run_info = json.load(f)
        else:
            run_info = {
                'status': 'failed' if result.returncode != 0 else 'unknown',
                'errors': ['run_info.json not found']
            }

        run_info['total_time'] = elapsed
        run_info['exit_code'] = result.returncode

        if not verbose and result.returncode != 0:
            run_info['stdout'] = result.stdout[-1000:] if result.stdout else ''
            run_info['stderr'] = result.stderr[-1000:] if result.stderr else ''

        return run_info

    except subprocess.TimeoutExpired:
        return {
            'algorithm': algo,
            'trajectory': trajectory,
            'status': 'failed',
            'errors': ['Experiment timed out'],
            'total_time': time.time() - start_time
        }
    except Exception as e:
        return {
            'algorithm': algo,
            'trajectory': trajectory,
            'status': 'failed',
            'errors': [f'Exception: {str(e)}'],
            'total_time': time.time() - start_time
        }


def run_batch(
    algorithms: list[str],
    trajectories: list[str],
    pose_modes: list[str],
    output_base: Path,
    timeout: float,
    speed_scale: float,
    verbose: bool,
    dry_run: bool
) -> dict:
    """
    Run all experiment combinations.

    Returns:
        dict with batch summary
    """
    total_experiments = len(algorithms) * len(trajectories) * len(pose_modes)

    batch_summary = {
        'start_time': datetime.now().isoformat(),
        'algorithms': algorithms,
        'trajectories': trajectories,
        'pose_modes': pose_modes,
        'total_experiments': total_experiments,
        'output_dir': str(output_base),
        'results': [],
        'summary': {
            'success': 0,
            'partial': 0,
            'failed': 0
        }
    }

    print(f"\n{'='*60}")
    print(f"SLAM Evaluation Batch Runner")
    print(f"{'='*60}")
    print(f"Algorithms: {', '.join(algorithms)}")
    print(f"Trajectories: {', '.join(trajectories)}")
    print(f"Pose modes: {', '.join(pose_modes)}")
    print(f"Total experiments: {total_experiments}")
    print(f"Output directory: {output_base}")
    print(f"Timeout per experiment: {timeout}s")
    print(f"{'='*60}\n")

    if dry_run:
        print("[DRY RUN] Would run the following experiments:")
        for algo in algorithms:
            for traj in trajectories:
                for mode in pose_modes:
                    exp_dir = output_base / f'{algo}_{traj}_{mode}'
                    print(f"  - {algo} + {traj} + {mode} -> {exp_dir}")
        return batch_summary

    # Create output directory
    output_base.mkdir(parents=True, exist_ok=True)

    experiment_num = 0
    for algo in algorithms:
        for traj in trajectories:
            for mode in pose_modes:
                experiment_num += 1
                exp_dir = output_base / f'{algo}_{traj}_{mode}'

                print(f"\n[{experiment_num}/{total_experiments}] {algo} + {traj} + {mode}")
                print(f"  Output: {exp_dir}")

                # Run experiment
                result = run_single_experiment(
                    algo=algo,
                    trajectory=traj,
                    pose_mode=mode,
                    output_dir=exp_dir,
                    timeout=timeout,
                    speed_scale=speed_scale,
                    verbose=verbose
                )

                # Track results
                batch_summary['results'].append({
                    'algorithm': algo,
                    'trajectory': traj,
                    'pose_mode': mode,
                    'output_dir': str(exp_dir),
                    'status': result.get('status', 'unknown'),
                    'total_time': result.get('total_time', 0),
                    'errors': result.get('errors', [])
                })

            # Update summary counts
            status = result.get('status', 'failed')
            if status in batch_summary['summary']:
                batch_summary['summary'][status] += 1
            else:
                batch_summary['summary']['failed'] += 1

            # Print result
            status_symbol = {'success': '✓', 'partial': '~', 'failed': '✗'}.get(status, '?')
            elapsed = result.get('total_time', 0)
            print(f"  Result: {status_symbol} {status} ({elapsed:.1f}s)")

            if result.get('errors'):
                for err in result['errors'][:3]:  # Show first 3 errors
                    print(f"    Error: {err[:80]}")

            # Cooldown between experiments
            if experiment_num < total_experiments:
                print(f"  Cooldown: {COOLDOWN_BETWEEN_RUNS}s...")
                time.sleep(COOLDOWN_BETWEEN_RUNS)

    batch_summary['end_time'] = datetime.now().isoformat()

    # Calculate total time
    start_dt = datetime.fromisoformat(batch_summary['start_time'])
    end_dt = datetime.fromisoformat(batch_summary['end_time'])
    batch_summary['total_time_seconds'] = (end_dt - start_dt).total_seconds()

    return batch_summary


def print_final_summary(summary: dict):
    """Print final batch summary."""
    print(f"\n{'='*60}")
    print("BATCH COMPLETE")
    print(f"{'='*60}")
    print(f"Total experiments: {summary['total_experiments']}")
    print(f"  Success: {summary['summary']['success']}")
    print(f"  Partial: {summary['summary']['partial']}")
    print(f"  Failed:  {summary['summary']['failed']}")

    if 'total_time_seconds' in summary:
        total_mins = summary['total_time_seconds'] / 60
        print(f"Total time: {total_mins:.1f} minutes")

    print(f"Output: {summary['output_dir']}")
    print(f"{'='*60}\n")

    # List failed experiments
    failed = [r for r in summary['results'] if r['status'] == 'failed']
    if failed:
        print("Failed experiments:")
        for r in failed:
            mode = r.get('pose_mode', 'unknown')
            print(f"  - {r['algorithm']} + {r['trajectory']} + {mode}")
            for err in r.get('errors', [])[:2]:
                print(f"      {err[:60]}")


def main():
    parser = argparse.ArgumentParser(
        description='Run batch SLAM evaluation experiments',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run all experiments (all algorithms × all trajectories, odometry mode)
    python3 run_all.py

    # Run with SLAM feedback
    python3 run_all.py --pose_modes slam

    # Run comparative study (odometry vs SLAM vs oracle)
    python3 run_all.py --pose_modes odometry slam ground_truth

    # Run only slam_toolbox
    python3 run_all.py --algorithms slam_toolbox

    # Run specific trajectories
    python3 run_all.py --trajectories traj_01_easy traj_02_loop

    # Custom output directory
    python3 run_all.py --output_dir ~/thesis/ros2_ws/results/batch_experiment

    # Dry run (show what would be executed)
    python3 run_all.py --dry-run
"""
    )

    parser.add_argument(
        '--algorithms', '-a',
        nargs='+',
        choices=DEFAULT_ALGORITHMS,
        default=DEFAULT_ALGORITHMS,
        help=f'SLAM algorithms to evaluate (default: {DEFAULT_ALGORITHMS})'
    )

    parser.add_argument(
        '--trajectories', '-t',
        nargs='+',
        help='Trajectory names (default: all in ~/thesis/trajectories/)'
    )

    parser.add_argument(
        '--pose_modes', '-p',
        nargs='+',
        choices=['odometry', 'slam', 'ground_truth'],
        default=['odometry'],
        help='Pose feedback modes to test (default: odometry)'
    )

    parser.add_argument(
        '--output_dir', '-o',
        type=Path,
        help='Base output directory (default: results/{timestamp})'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f'Timeout per experiment in seconds (default: {DEFAULT_TIMEOUT})'
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
        help='Show detailed output from each experiment'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be executed without running'
    )

    args = parser.parse_args()

    # Discover trajectories if not specified
    if args.trajectories:
        trajectories = args.trajectories
    else:
        trajectories = discover_trajectories(DEFAULT_TRAJECTORY_DIR)
        if not trajectories:
            print(f"Error: No trajectory files found in {DEFAULT_TRAJECTORY_DIR}")
            sys.exit(1)

    # Determine output directory
    if args.output_dir:
        output_base = args.output_dir
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_base = DEFAULT_RESULTS_DIR / f'batch_{timestamp}'

    # Run batch
    summary = run_batch(
        algorithms=args.algorithms,
        trajectories=trajectories,
        pose_modes=args.pose_modes,
        output_base=output_base,
        timeout=args.timeout,
        speed_scale=args.speed_scale,
        verbose=args.verbose,
        dry_run=args.dry_run
    )

    if not args.dry_run:
        # Save batch summary
        summary_file = output_base / 'batch_summary.json'
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"Batch summary saved to: {summary_file}")

        # Print final summary
        print_final_summary(summary)

        # Run aggregation if available
        aggregate_script = Path(__file__).parent / 'aggregate_results.py'
        if aggregate_script.exists():
            print("\nRunning result aggregation...")
            subprocess.run([
                'python3', str(aggregate_script),
                '--batch_dir', str(output_base)
            ])

    # Exit code based on success rate
    if args.dry_run:
        sys.exit(0)
    elif summary['summary']['failed'] == 0:
        sys.exit(0)
    elif summary['summary']['success'] + summary['summary']['partial'] > 0:
        sys.exit(0)  # At least some succeeded
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
