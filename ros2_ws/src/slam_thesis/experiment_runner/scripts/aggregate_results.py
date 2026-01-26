#!/usr/bin/env python3
"""
aggregate_results.py - Combine evaluation metrics into summary CSV.

Scans experiment result directories and aggregates metrics into a
single CSV file for analysis and comparison.

Usage:
    python3 aggregate_results.py --batch_dir ~/thesis/ros2_ws/results/batch_20260125
    python3 aggregate_results.py --results_dir ~/thesis/ros2_ws/results --output summary.csv

Output:
    summary.csv with columns:
    - algorithm, trajectory, pose_mode, status
    - ate_rmse, ate_mean, ate_median, ate_std, ate_min, ate_max
    - rpe_rmse, rpe_mean, rpe_median, rpe_std, rpe_min, rpe_max, rpe_delta_m
    - gt_poses, est_poses, trajectory_time
    - completion_rate, intended_distance, actual_distance, final_distance_to_goal, goal_reached
    - avg_cpu, peak_cpu, avg_memory_mb, peak_memory_mb
    - mean_cte, max_cte, rmse_cte
    - mean_pos_delta, max_pos_delta, mean_angle_delta
"""

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional


def find_result_dirs(base_dir: Path) -> list[Path]:
    """
    Find all directories containing metrics.json files.

    Handles both flat structure (direct subdirectories) and
    nested batch structure.
    """
    result_dirs = []

    # Direct search for metrics.json
    for metrics_file in base_dir.rglob('metrics.json'):
        result_dirs.append(metrics_file.parent)

    # Remove duplicates and sort
    result_dirs = sorted(set(result_dirs))
    return result_dirs


def parse_run_dir_name(dir_path: Path) -> tuple[Optional[str], Optional[str]]:
    """
    Extract algorithm and trajectory from directory name.

    Expected formats:
    - {algo}_{trajectory}
    - {algo}_{trajectory}_{timestamp}
    """
    name = dir_path.name

    # Known algorithm prefixes
    algorithms = ['slam_toolbox', 'cartographer']

    for algo in algorithms:
        if name.startswith(algo + '_'):
            remainder = name[len(algo) + 1:]
            # Remove timestamp suffix if present (format: _YYYYMMDD_HHMMSS)
            parts = remainder.rsplit('_', 2)
            if len(parts) >= 3 and len(parts[-1]) == 6 and len(parts[-2]) == 8:
                # Has timestamp suffix
                trajectory = '_'.join(parts[:-2])
            else:
                trajectory = remainder
            return algo, trajectory

    return None, None


def load_metrics(result_dir: Path) -> dict:
    """Load metrics from a result directory."""
    metrics_file = result_dir / 'metrics.json'
    run_info_file = result_dir / 'run_info.json'

    data = {
        'directory': str(result_dir),
        'algorithm': None,
        'trajectory': None,
        'pose_mode': None,
        'status': 'unknown',
        # ATE metrics
        'ate_rmse': None,
        'ate_mean': None,
        'ate_median': None,
        'ate_std': None,
        'ate_min': None,
        'ate_max': None,
        # RPE metrics
        'rpe_rmse': None,
        'rpe_mean': None,
        'rpe_median': None,
        'rpe_std': None,
        'rpe_min': None,
        'rpe_max': None,
        'rpe_delta_m': None,
        # Pose counts and time
        'gt_poses': None,
        'est_poses': None,
        'trajectory_time': None,
        # Completion metrics (Task 3.5)
        'completion_rate': None,
        'intended_distance': None,
        'actual_distance': None,
        'final_distance_to_goal': None,
        'goal_reached': None,
        # Resource usage metrics (Task 3.4)
        'avg_cpu': None,
        'peak_cpu': None,
        'avg_memory_mb': None,
        'peak_memory_mb': None,
        # CTE metrics (Task 3.2)
        'mean_cte': None,
        'max_cte': None,
        'rmse_cte': None,
        # Odom vs SLAM delta metrics (Task 3.1)
        'mean_pos_delta': None,
        'max_pos_delta': None,
        'mean_angle_delta': None,
        # Error tracking
        'errors': []
    }

    # Extract algorithm and trajectory from directory name
    algo, traj = parse_run_dir_name(result_dir)
    data['algorithm'] = algo
    data['trajectory'] = traj

    # Load run_info.json if available
    if run_info_file.exists():
        try:
            with open(run_info_file, 'r') as f:
                run_info = json.load(f)
                if not data['algorithm']:
                    data['algorithm'] = run_info.get('algorithm')
                if not data['trajectory']:
                    data['trajectory'] = run_info.get('trajectory')
                data['pose_mode'] = run_info.get('pose_mode')
                data['trajectory_time'] = run_info.get('trajectory_time')

                # Resource usage stats (Task 3.4)
                data['avg_cpu'] = run_info.get('avg_cpu')
                data['peak_cpu'] = run_info.get('peak_cpu')
                data['avg_memory_mb'] = run_info.get('avg_memory_mb')
                data['peak_memory_mb'] = run_info.get('peak_memory_mb')

                if run_info.get('errors'):
                    data['errors'].extend(run_info['errors'])
        except Exception as e:
            data['errors'].append(f'Failed to load run_info.json: {e}')

    # Load metrics.json
    if metrics_file.exists():
        try:
            with open(metrics_file, 'r') as f:
                metrics = json.load(f)

            data['status'] = metrics.get('status', 'unknown')
            data['gt_poses'] = metrics.get('gt_poses')
            data['est_poses'] = metrics.get('est_poses')

            # ATE metrics
            ate = metrics.get('ate')
            if ate:
                data['ate_rmse'] = ate.get('rmse')
                data['ate_mean'] = ate.get('mean')
                data['ate_median'] = ate.get('median')
                data['ate_std'] = ate.get('std')
                data['ate_min'] = ate.get('min')
                data['ate_max'] = ate.get('max')

            # RPE metrics
            rpe = metrics.get('rpe')
            if rpe:
                data['rpe_rmse'] = rpe.get('rmse')
                data['rpe_mean'] = rpe.get('mean')
                data['rpe_median'] = rpe.get('median')
                data['rpe_std'] = rpe.get('std')
                data['rpe_min'] = rpe.get('min')
                data['rpe_max'] = rpe.get('max')
                data['rpe_delta_m'] = rpe.get('delta_m')

            # Completion metrics (Task 3.5)
            completion = metrics.get('completion')
            if completion:
                data['completion_rate'] = completion.get('completion_rate')
                data['intended_distance'] = completion.get('intended_distance')
                data['actual_distance'] = completion.get('actual_distance')
                data['final_distance_to_goal'] = completion.get('final_distance_to_goal')
                data['goal_reached'] = completion.get('goal_reached')

            if metrics.get('errors'):
                data['errors'].extend(metrics['errors'])

        except Exception as e:
            data['errors'].append(f'Failed to load metrics.json: {e}')
            data['status'] = 'failed'
    else:
        data['errors'].append('metrics.json not found')
        data['status'] = 'failed'

    # Load path_deviation.json for CTE metrics (Task 3.2)
    cte_file = result_dir / 'path_deviation.json'
    if cte_file.exists():
        try:
            with open(cte_file, 'r') as f:
                cte_data = json.load(f)
            cte_metrics = cte_data.get('metrics')
            if cte_metrics:
                data['mean_cte'] = cte_metrics.get('mean_cte')
                data['max_cte'] = cte_metrics.get('max_cte')
                data['rmse_cte'] = cte_metrics.get('rmse_cte')
        except Exception as e:
            data['errors'].append(f'Failed to load path_deviation.json: {e}')

    # Load odom_slam_delta.csv and compute summary stats (Task 3.1)
    delta_file = result_dir / 'odom_slam_delta.csv'
    if delta_file.exists():
        try:
            pos_deltas = []
            angle_deltas = []
            with open(delta_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if 'pos_delta' in row and row['pos_delta']:
                        pos_deltas.append(float(row['pos_delta']))
                    if 'angle_delta' in row and row['angle_delta']:
                        angle_deltas.append(float(row['angle_delta']))

            if pos_deltas:
                data['mean_pos_delta'] = sum(pos_deltas) / len(pos_deltas)
                data['max_pos_delta'] = max(pos_deltas)
            if angle_deltas:
                data['mean_angle_delta'] = sum(angle_deltas) / len(angle_deltas)
        except Exception as e:
            data['errors'].append(f'Failed to load odom_slam_delta.csv: {e}')

    return data


def aggregate_results(result_dirs: list[Path]) -> list[dict]:
    """Aggregate metrics from all result directories."""
    results = []
    for result_dir in result_dirs:
        data = load_metrics(result_dir)
        results.append(data)
    return results


def write_csv(results: list[dict], output_file: Path):
    """Write aggregated results to CSV file."""
    # Define column order
    columns = [
        'algorithm',
        'trajectory',
        'pose_mode',
        'status',
        # ATE metrics
        'ate_rmse',
        'ate_mean',
        'ate_median',
        'ate_std',
        'ate_min',
        'ate_max',
        # RPE metrics
        'rpe_rmse',
        'rpe_mean',
        'rpe_median',
        'rpe_std',
        'rpe_min',
        'rpe_max',
        'rpe_delta_m',
        # Pose counts and time
        'gt_poses',
        'est_poses',
        'trajectory_time',
        # Completion metrics (Task 3.5)
        'completion_rate',
        'intended_distance',
        'actual_distance',
        'final_distance_to_goal',
        'goal_reached',
        # Resource usage (Task 3.4)
        'avg_cpu',
        'peak_cpu',
        'avg_memory_mb',
        'peak_memory_mb',
        # CTE metrics (Task 3.2)
        'mean_cte',
        'max_cte',
        'rmse_cte',
        # Odom vs SLAM delta (Task 3.1)
        'mean_pos_delta',
        'max_pos_delta',
        'mean_angle_delta',
        # Directory reference
        'directory'
    ]

    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()

        for result in results:
            # Format numeric values
            row = {}
            for col in columns:
                val = result.get(col)
                if val is None:
                    row[col] = ''
                elif isinstance(val, float):
                    row[col] = f'{val:.6f}'
                else:
                    row[col] = val
            writer.writerow(row)


def print_summary_table(results: list[dict]):
    """Print a human-readable summary table."""
    print("\n" + "=" * 100)
    print("AGGREGATED RESULTS SUMMARY")
    print("=" * 100)

    # Group by algorithm
    by_algo = {}
    for r in results:
        algo = r['algorithm'] or 'unknown'
        if algo not in by_algo:
            by_algo[algo] = []
        by_algo[algo].append(r)

    for algo, algo_results in sorted(by_algo.items()):
        print(f"\n{algo.upper()}")
        print("-" * 100)
        print(f"{'Trajectory':<16} {'Mode':<8} {'Status':<8} {'ATE RMSE':>10} {'RPE RMSE':>10} "
              f"{'Compl%':>8} {'CTE':>8} {'CPU%':>8}")
        print("-" * 100)

        for r in sorted(algo_results, key=lambda x: (x.get('trajectory') or '', x.get('pose_mode') or '')):
            traj = (r['trajectory'] or 'unknown')[:16]
            mode = (r['pose_mode'] or '-')[:8]
            status = r['status'][:8]
            ate = f"{r['ate_rmse']:.4f}" if r['ate_rmse'] is not None else 'N/A'
            rpe = f"{r['rpe_rmse']:.4f}" if r['rpe_rmse'] is not None else 'N/A'
            compl = f"{r['completion_rate']*100:.1f}" if r['completion_rate'] is not None else 'N/A'
            cte = f"{r['mean_cte']:.4f}" if r['mean_cte'] is not None else 'N/A'
            cpu = f"{r['avg_cpu']:.1f}" if r['avg_cpu'] is not None else 'N/A'
            print(f"{traj:<16} {mode:<8} {status:<8} {ate:>10} {rpe:>10} {compl:>8} {cte:>8} {cpu:>8}")

    # Overall statistics
    print("\n" + "=" * 100)
    print("OVERALL STATISTICS")
    print("=" * 100)

    total = len(results)
    success = sum(1 for r in results if r['status'] == 'success')
    partial = sum(1 for r in results if r['status'] == 'partial')
    failed = sum(1 for r in results if r['status'] == 'failed')

    print(f"Total experiments: {total}")
    print(f"  Success: {success}")
    print(f"  Partial: {partial}")
    print(f"  Failed:  {failed}")

    # Goals reached
    with_completion = [r for r in results if r['goal_reached'] is not None]
    if with_completion:
        goals_reached = sum(1 for r in with_completion if r['goal_reached'])
        print(f"  Goals reached: {goals_reached}/{len(with_completion)}")

    # Best results per algorithm
    for algo, algo_results in sorted(by_algo.items()):
        successful = [r for r in algo_results if r['ate_rmse'] is not None]
        if successful:
            best = min(successful, key=lambda x: x['ate_rmse'])
            print(f"\nBest {algo} (by ATE RMSE):")
            print(f"  Trajectory: {best['trajectory']}")
            if best['pose_mode']:
                print(f"  Pose Mode: {best['pose_mode']}")
            print(f"  ATE RMSE: {best['ate_rmse']:.4f} m")
            if best['rpe_rmse'] is not None:
                print(f"  RPE RMSE: {best['rpe_rmse']:.4f} m")
            if best['completion_rate'] is not None:
                print(f"  Completion: {best['completion_rate']*100:.1f}%")
            if best['mean_cte'] is not None:
                print(f"  Mean CTE: {best['mean_cte']:.4f} m")
            if best['avg_cpu'] is not None:
                print(f"  Avg CPU: {best['avg_cpu']:.1f}%")


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate SLAM evaluation metrics into summary CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Aggregate results from a batch directory
    python3 aggregate_results.py --batch_dir ~/thesis/ros2_ws/results/batch_20260125

    # Aggregate all results in results directory
    python3 aggregate_results.py --results_dir ~/thesis/ros2_ws/results

    # Custom output file
    python3 aggregate_results.py --batch_dir ./results --output custom_summary.csv
"""
    )

    parser.add_argument(
        '--batch_dir', '-b',
        type=Path,
        help='Batch directory containing experiment results'
    )

    parser.add_argument(
        '--results_dir', '-r',
        type=Path,
        help='Results directory to scan recursively'
    )

    parser.add_argument(
        '--output', '-o',
        type=Path,
        help='Output CSV file (default: summary.csv in input directory)'
    )

    parser.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='Suppress summary table output'
    )

    args = parser.parse_args()

    # Determine input directory
    if args.batch_dir:
        input_dir = args.batch_dir
    elif args.results_dir:
        input_dir = args.results_dir
    else:
        # Default to results directory
        input_dir = Path.home() / 'thesis' / 'ros2_ws' / 'results'

    if not input_dir.exists():
        print(f"Error: Directory not found: {input_dir}")
        sys.exit(1)

    # Find result directories
    print(f"Scanning for results in: {input_dir}")
    result_dirs = find_result_dirs(input_dir)

    if not result_dirs:
        print("No result directories found (no metrics.json files)")
        sys.exit(1)

    print(f"Found {len(result_dirs)} result directories")

    # Aggregate results
    results = aggregate_results(result_dirs)

    # Determine output file
    if args.output:
        output_file = args.output
    else:
        output_file = input_dir / 'summary.csv'

    # Write CSV
    write_csv(results, output_file)
    print(f"\nSummary CSV written to: {output_file}")

    # Print summary table
    if not args.quiet:
        print_summary_table(results)

    sys.exit(0)


if __name__ == '__main__':
    main()
