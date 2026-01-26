#!/usr/bin/env python3
"""
Aggregate results from multiple SLAM experiments.

Collects metrics.json files from experiment results and generates
summary statistics for analysis.

Usage:
    python3 aggregate_results.py --bags_dir ~/thesis/ros2_ws/bags
    python3 aggregate_results.py --bags_dir /path/to/bags --output results_summary.csv

Output:
    - results_summary.csv: Flat CSV with all results
    - results_summary.json: Detailed JSON with aggregated statistics
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


def discover_results(bags_dir: str) -> List[Tuple[str, str, str]]:
    """
    Discover all metrics.json files in the bags directory.

    Args:
        bags_dir: Directory containing rosbag directories with results

    Returns:
        List of (bag_path, algorithm, metrics_path) tuples
    """
    bags_dir = os.path.expanduser(bags_dir)
    bags_dir = os.path.abspath(bags_dir)

    results = []
    if not os.path.isdir(bags_dir):
        return results

    for bag_name in sorted(os.listdir(bags_dir)):
        bag_path = os.path.join(bags_dir, bag_name)
        if not os.path.isdir(bag_path):
            continue

        results_dir = os.path.join(bag_path, 'results')
        if not os.path.isdir(results_dir):
            continue

        for algorithm in os.listdir(results_dir):
            algo_dir = os.path.join(results_dir, algorithm)
            if not os.path.isdir(algo_dir):
                continue

            metrics_path = os.path.join(algo_dir, 'metrics.json')
            if os.path.exists(metrics_path):
                results.append((bag_path, algorithm, metrics_path))

    return results


def load_metrics(metrics_path: str) -> Optional[Dict[str, Any]]:
    """Load metrics from a JSON file."""
    try:
        with open(metrics_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Failed to load {metrics_path}: {e}", file=sys.stderr)
        return None


def parse_dataset_name(dataset: str) -> Dict[str, str]:
    """
    Parse dataset name into components.

    Expected formats:
        - traj_01_easy_baseline_TIMESTAMP
        - traj_02_loop_degraded_TIMESTAMP

    Returns:
        Dictionary with 'trajectory', 'condition', 'timestamp' keys
    """
    parts = dataset.split('_')

    # Default values
    result = {
        'trajectory': dataset,
        'condition': 'unknown',
        'timestamp': '',
    }

    # Try to parse structured names
    if len(parts) >= 4:
        # Find condition (baseline or degraded)
        for i, part in enumerate(parts):
            if part in ['baseline', 'degraded']:
                result['trajectory'] = '_'.join(parts[:i])
                result['condition'] = part
                if i + 1 < len(parts):
                    result['timestamp'] = '_'.join(parts[i+1:])
                break
        else:
            # No condition found, assume all is trajectory name
            result['trajectory'] = '_'.join(parts[:-1]) if len(parts) > 1 else parts[0]

    return result


def compute_statistics(values: List[float]) -> Dict[str, float]:
    """Compute summary statistics for a list of values."""
    if not values:
        return {
            'count': 0,
            'mean': None,
            'std': None,
            'min': None,
            'max': None,
            'median': None,
        }

    arr = np.array(values)
    return {
        'count': len(values),
        'mean': float(np.mean(arr)),
        'std': float(np.std(arr)),
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
        'median': float(np.median(arr)),
    }


def aggregate_by_group(
    results: List[Dict[str, Any]],
    group_keys: List[str]
) -> Dict[str, Dict[str, Any]]:
    """
    Aggregate results by specified grouping keys.

    Args:
        results: List of result dictionaries
        group_keys: Keys to group by (e.g., ['algorithm'], ['trajectory', 'condition'])

    Returns:
        Dictionary mapping group tuples to aggregated statistics
    """
    groups = defaultdict(list)

    for r in results:
        if not r.get('success', False):
            continue

        # Build group key
        key_parts = []
        for k in group_keys:
            if k in r:
                key_parts.append(str(r[k]))
            elif k in r.get('parsed', {}):
                key_parts.append(str(r['parsed'][k]))
            else:
                key_parts.append('unknown')
        key = tuple(key_parts)

        groups[key].append(r)

    # Compute statistics for each group
    aggregated = {}
    for key, group_results in groups.items():
        ate_rmse_values = [r['ate']['rmse'] for r in group_results if r.get('ate')]
        rpe_rmse_values = [r['rpe']['rmse'] for r in group_results if r.get('rpe')]
        traj_lengths = [r['trajectory_length_m'] for r in group_results if r.get('trajectory_length_m')]

        group_name = '__'.join(key) if len(key) > 1 else key[0]
        aggregated[group_name] = {
            'group_keys': dict(zip(group_keys, key)),
            'count': len(group_results),
            'ate_rmse': compute_statistics(ate_rmse_values),
            'rpe_rmse': compute_statistics(rpe_rmse_values),
            'trajectory_length_m': compute_statistics(traj_lengths),
        }

    return aggregated


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate SLAM experiment results.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --bags_dir ~/thesis/ros2_ws/bags
  %(prog)s --bags_dir /path/to/bags --output my_results.csv
  %(prog)s --bags_dir /path/to/bags --format json
        """
    )

    parser.add_argument(
        '--bags_dir',
        type=str,
        required=True,
        help='Directory containing rosbag directories with results'
    )

    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Output file path (default: bags_dir/results_summary.{csv,json})'
    )

    parser.add_argument(
        '--format',
        type=str,
        choices=['csv', 'json', 'both'],
        default='both',
        help='Output format (default: both)'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Validate input
    bags_dir = os.path.expanduser(args.bags_dir)
    bags_dir = os.path.abspath(bags_dir)

    if not os.path.isdir(bags_dir):
        print(f"Error: Directory does not exist: {bags_dir}", file=sys.stderr)
        sys.exit(1)

    # Discover results
    discovered = discover_results(bags_dir)

    if not discovered:
        print(f"Error: No metrics.json files found in {bags_dir}", file=sys.stderr)
        sys.exit(1)

    if args.verbose:
        print(f"Found {len(discovered)} result files")

    # Load all metrics
    all_results = []
    for bag_path, algorithm, metrics_path in discovered:
        metrics = load_metrics(metrics_path)
        if metrics is None:
            continue

        # Parse dataset name
        dataset = metrics.get('dataset', os.path.basename(bag_path))
        parsed = parse_dataset_name(dataset)

        # Add to results with parsed info
        result = {
            **metrics,
            'bag_path': bag_path,
            'metrics_path': metrics_path,
            'parsed': parsed,
        }
        all_results.append(result)

    if not all_results:
        print("Error: No valid results loaded", file=sys.stderr)
        sys.exit(1)

    # Separate successful and failed
    successful = [r for r in all_results if r.get('success', False)]
    failed = [r for r in all_results if not r.get('success', False)]

    print(f"\n{'='*70}")
    print(f"RESULTS AGGREGATION")
    print(f"{'='*70}")
    print(f"Total results:  {len(all_results)}")
    print(f"Successful:     {len(successful)}")
    print(f"Failed:         {len(failed)}")
    print(f"{'='*70}")

    if successful:
        # Aggregate by algorithm
        by_algorithm = aggregate_by_group(successful, ['algorithm'])
        print(f"\nBy Algorithm:")
        print(f"{'-'*50}")
        print(f"{'Algorithm':<20} {'Count':>6} {'ATE RMSE (m)':>15} {'Std':>10}")
        print(f"{'-'*50}")
        for algo, stats in sorted(by_algorithm.items()):
            ate = stats['ate_rmse']
            print(f"{algo:<20} {stats['count']:>6} {ate['mean']:>15.4f} {ate['std']:>10.4f}")
        print(f"{'-'*50}")

        # Aggregate by trajectory
        by_trajectory = aggregate_by_group(successful, ['parsed.trajectory'])
        if len(by_trajectory) > 1:
            print(f"\nBy Trajectory:")
            print(f"{'-'*50}")
            print(f"{'Trajectory':<20} {'Count':>6} {'ATE RMSE (m)':>15} {'Std':>10}")
            print(f"{'-'*50}")
            for traj, stats in sorted(by_trajectory.items()):
                ate = stats['ate_rmse']
                traj_name = traj[:19] if len(traj) > 19 else traj
                print(f"{traj_name:<20} {stats['count']:>6} {ate['mean']:>15.4f} {ate['std']:>10.4f}")
            print(f"{'-'*50}")

        # Aggregate by condition
        by_condition = aggregate_by_group(successful, ['parsed.condition'])
        conditions = list(by_condition.keys())
        if len(conditions) > 1 and 'unknown' not in conditions:
            print(f"\nBy Condition:")
            print(f"{'-'*50}")
            print(f"{'Condition':<20} {'Count':>6} {'ATE RMSE (m)':>15} {'Std':>10}")
            print(f"{'-'*50}")
            for cond, stats in sorted(by_condition.items()):
                ate = stats['ate_rmse']
                print(f"{cond:<20} {stats['count']:>6} {ate['mean']:>15.4f} {ate['std']:>10.4f}")
            print(f"{'-'*50}")

        # Aggregate by algorithm + condition
        by_algo_cond = aggregate_by_group(successful, ['algorithm', 'parsed.condition'])
        if len(by_algo_cond) > 2:
            print(f"\nBy Algorithm + Condition:")
            print(f"{'-'*60}")
            print(f"{'Algorithm':<15} {'Condition':<12} {'Count':>6} {'ATE RMSE':>12} {'Std':>10}")
            print(f"{'-'*60}")
            for key, stats in sorted(by_algo_cond.items()):
                parts = key.split('__')
                algo = parts[0] if len(parts) > 0 else 'unknown'
                cond = parts[1] if len(parts) > 1 else 'unknown'
                ate = stats['ate_rmse']
                print(f"{algo:<15} {cond:<12} {stats['count']:>6} {ate['mean']:>12.4f} {ate['std']:>10.4f}")
            print(f"{'-'*60}")

    # Prepare output data
    output_base = args.output or os.path.join(bags_dir, 'results_summary')

    # CSV output - flat table of all results
    if args.format in ['csv', 'both']:
        csv_path = output_base if output_base.endswith('.csv') else output_base + '.csv'

        fieldnames = [
            'dataset', 'algorithm', 'trajectory', 'condition',
            'success', 'ate_rmse', 'ate_mean', 'ate_std',
            'rpe_rmse', 'rpe_mean', 'rpe_std',
            'trajectory_length_m', 'duration_s', 'num_poses',
            'timestamp', 'error'
        ]

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for r in all_results:
                row = {
                    'dataset': r.get('dataset', ''),
                    'algorithm': r.get('algorithm', ''),
                    'trajectory': r['parsed'].get('trajectory', ''),
                    'condition': r['parsed'].get('condition', ''),
                    'success': r.get('success', False),
                    'ate_rmse': r['ate']['rmse'] if r.get('ate') else '',
                    'ate_mean': r['ate']['mean'] if r.get('ate') else '',
                    'ate_std': r['ate']['std'] if r.get('ate') else '',
                    'rpe_rmse': r['rpe']['rmse'] if r.get('rpe') else '',
                    'rpe_mean': r['rpe']['mean'] if r.get('rpe') else '',
                    'rpe_std': r['rpe']['std'] if r.get('rpe') else '',
                    'trajectory_length_m': r.get('trajectory_length_m', ''),
                    'duration_s': r.get('duration_s', ''),
                    'num_poses': r.get('num_poses', ''),
                    'timestamp': r.get('timestamp', ''),
                    'error': r.get('error', ''),
                }
                writer.writerow(row)

        print(f"\nCSV saved to: {csv_path}")

    # JSON output - detailed with aggregations
    if args.format in ['json', 'both']:
        json_path = output_base if output_base.endswith('.json') else output_base + '.json'

        summary = {
            'generated_at': datetime.now().isoformat(),
            'bags_dir': bags_dir,
            'total_results': len(all_results),
            'successful': len(successful),
            'failed': len(failed),
            'aggregations': {
                'by_algorithm': aggregate_by_group(successful, ['algorithm']),
                'by_trajectory': aggregate_by_group(successful, ['parsed.trajectory']),
                'by_condition': aggregate_by_group(successful, ['parsed.condition']),
                'by_algorithm_condition': aggregate_by_group(
                    successful, ['algorithm', 'parsed.condition']
                ),
            },
            'individual_results': [
                {
                    'dataset': r.get('dataset', ''),
                    'algorithm': r.get('algorithm', ''),
                    'trajectory': r['parsed'].get('trajectory', ''),
                    'condition': r['parsed'].get('condition', ''),
                    'success': r.get('success', False),
                    'ate': r.get('ate'),
                    'rpe': r.get('rpe'),
                    'trajectory_length_m': r.get('trajectory_length_m'),
                    'duration_s': r.get('duration_s'),
                    'num_poses': r.get('num_poses'),
                    'timestamp': r.get('timestamp', ''),
                    'error': r.get('error'),
                }
                for r in all_results
            ],
        }

        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"JSON saved to: {json_path}")

    # Print failed experiments if any
    if failed and args.verbose:
        print(f"\nFailed Experiments:")
        print(f"{'-'*70}")
        for r in failed:
            dataset = r.get('dataset', 'unknown')
            algorithm = r.get('algorithm', 'unknown')
            error = r.get('error', 'Unknown error')[:50]
            print(f"  {dataset} / {algorithm}: {error}")
        print(f"{'-'*70}")

    print(f"\nAggregation complete!")
    sys.exit(0)


if __name__ == '__main__':
    main()
