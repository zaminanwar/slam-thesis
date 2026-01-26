#!/usr/bin/env python3
"""
evaluate_run.py - Compute ATE/RPE metrics for a single SLAM run.

Uses the evo library to evaluate trajectory accuracy by comparing
ground truth (gt.tum) against SLAM estimate (est.tum).

Usage:
    python3 evaluate_run.py --run_dir /path/to/run
    python3 evaluate_run.py --gt /path/to/gt.tum --est /path/to/est.tum --output /path/to/metrics.json

Outputs:
    metrics.json with ATE and RPE statistics
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from datetime import datetime


# Minimum number of poses required for meaningful evaluation
MIN_POSES = 10

# Maximum timestamp difference for pose association (seconds)
MAX_T_DIFF = 0.1


def find_evo_binary(name: str) -> str:
    """Find evo binary, checking common locations."""
    # Check in PATH first
    result = subprocess.run(['which', name], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()

    # Check common user install location
    user_bin = Path.home() / '.local' / 'bin' / name
    if user_bin.exists():
        return str(user_bin)

    raise FileNotFoundError(f"Could not find {name}. Install with: pip install evo")


def count_poses(tum_file: Path) -> int:
    """Count number of poses in a TUM format file."""
    count = 0
    with open(tum_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                count += 1
    return count


def run_evo_command(cmd: list, description: str) -> dict:
    """
    Run an evo command and parse the JSON output.

    Returns dict with either 'stats' or 'error' key.
    """
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60
        )

        if result.returncode != 0:
            # Parse common error messages
            stderr = result.stderr.lower()
            if 'could not associate' in stderr or 'no matching' in stderr:
                return {'error': f'{description}: No matching timestamps between trajectories'}
            elif 'singular matrix' in stderr or 'alignment' in stderr:
                return {'error': f'{description}: Alignment failed (trajectories may be degenerate)'}
            elif 'not enough' in stderr or 'insufficient' in stderr:
                return {'error': f'{description}: Insufficient data for evaluation'}
            else:
                return {'error': f'{description}: {result.stderr.strip()[:200]}'}

        # Parse the saved JSON results
        return {'success': True, 'stdout': result.stdout, 'stderr': result.stderr}

    except subprocess.TimeoutExpired:
        return {'error': f'{description}: Command timed out after 60 seconds'}
    except Exception as e:
        return {'error': f'{description}: {str(e)}'}


def parse_evo_zip(zip_path: Path) -> dict:
    """Parse evo results from a .zip file."""
    import zipfile
    import tempfile

    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            with zipfile.ZipFile(zip_path, 'r') as zf:
                zf.extractall(tmpdir)

            stats_file = Path(tmpdir) / 'stats.json'
            if stats_file.exists():
                with open(stats_file, 'r') as f:
                    return json.load(f)

            # Try to find any JSON file
            for json_file in Path(tmpdir).glob('*.json'):
                with open(json_file, 'r') as f:
                    return json.load(f)

        return {'error': 'No stats.json found in evo output'}
    except Exception as e:
        return {'error': f'Failed to parse evo output: {str(e)}'}


def compute_ate(gt_file: Path, est_file: Path, output_dir: Path) -> dict:
    """
    Compute Absolute Trajectory Error (ATE).

    ATE measures the global consistency of the trajectory by comparing
    absolute poses after Umeyama alignment.
    """
    evo_ape = find_evo_binary('evo_ape')
    results_file = output_dir / 'ate_results.zip'

    cmd = [
        evo_ape, 'tum',
        str(gt_file),
        str(est_file),
        '--align',  # Umeyama alignment (no scale correction)
        '--t_max_diff', str(MAX_T_DIFF),
        '--save_results', str(results_file),
        '--no_warnings',
        '--silent'
    ]

    result = run_evo_command(cmd, 'ATE computation')

    if 'error' in result:
        return result

    if results_file.exists():
        stats = parse_evo_zip(results_file)
        # Clean up
        results_file.unlink()
        return stats

    return {'error': 'ATE: No results file generated'}


def estimate_trajectory_length(tum_file: Path) -> float:
    """Estimate total trajectory length from TUM file."""
    import math
    positions = []
    with open(tum_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                parts = line.split()
                if len(parts) >= 4:
                    positions.append((float(parts[1]), float(parts[2]), float(parts[3])))

    if len(positions) < 2:
        return 0.0

    total_length = 0.0
    for i in range(1, len(positions)):
        dx = positions[i][0] - positions[i-1][0]
        dy = positions[i][1] - positions[i-1][1]
        dz = positions[i][2] - positions[i-1][2]
        total_length += math.sqrt(dx*dx + dy*dy + dz*dz)

    return total_length


def load_trajectory_csv(csv_file: Path) -> list:
    """
    Load intended trajectory from CSV file.

    Expected format: x,y,yaw,speed (with header row)

    Returns:
        List of (x, y) tuples representing waypoints
    """
    import csv
    waypoints = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append((float(row['x']), float(row['y'])))
    return waypoints


def compute_intended_trajectory_length(waypoints: list) -> float:
    """
    Compute total length of intended trajectory from waypoints.

    Args:
        waypoints: List of (x, y) tuples

    Returns:
        Total path length in meters
    """
    import math
    if len(waypoints) < 2:
        return 0.0

    total_length = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i-1][0]
        dy = waypoints[i][1] - waypoints[i-1][1]
        total_length += math.sqrt(dx*dx + dy*dy)

    return total_length


def get_tum_positions(tum_file: Path) -> list:
    """
    Extract (x, y) positions from TUM file.

    Returns:
        List of (x, y) tuples
    """
    positions = []
    with open(tum_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                parts = line.split()
                if len(parts) >= 4:
                    positions.append((float(parts[1]), float(parts[2])))
    return positions


def compute_completion_metrics(
    est_file: Path,
    trajectory_file: Path,
    goal_tolerance: float = 0.5
) -> dict:
    """
    Compute trajectory completion metrics.

    Compares actual traveled path (from est.tum) against intended trajectory (from CSV).

    Args:
        est_file: Path to estimated trajectory (TUM format)
        trajectory_file: Path to intended trajectory (CSV format)
        goal_tolerance: Distance threshold for considering goal reached (meters)

    Returns:
        dict with completion metrics:
        - intended_distance: Total intended path length (m)
        - actual_distance: Total distance traveled (m)
        - completion_rate: actual/intended ratio (0-1+, can exceed 1 if robot wanders)
        - final_distance_to_goal: Distance from final pose to final waypoint (m)
        - goal_reached: Whether final pose is within tolerance of final waypoint
        - start_distance: Distance from starting pose to first waypoint (m)
    """
    import math

    result = {
        'intended_distance': None,
        'actual_distance': None,
        'completion_rate': None,
        'final_distance_to_goal': None,
        'goal_reached': None,
        'start_distance': None
    }

    # Load intended trajectory
    try:
        waypoints = load_trajectory_csv(trajectory_file)
        if len(waypoints) < 2:
            return {'error': 'Intended trajectory has fewer than 2 waypoints'}
        result['intended_distance'] = compute_intended_trajectory_length(waypoints)
    except Exception as e:
        return {'error': f'Failed to load trajectory CSV: {str(e)}'}

    # Load actual positions
    try:
        actual_positions = get_tum_positions(est_file)
        if len(actual_positions) < 2:
            return {'error': 'Actual trajectory has fewer than 2 positions'}
    except Exception as e:
        return {'error': f'Failed to load est.tum: {str(e)}'}

    # Compute actual distance traveled (2D)
    actual_distance = 0.0
    for i in range(1, len(actual_positions)):
        dx = actual_positions[i][0] - actual_positions[i-1][0]
        dy = actual_positions[i][1] - actual_positions[i-1][1]
        actual_distance += math.sqrt(dx*dx + dy*dy)
    result['actual_distance'] = actual_distance

    # Compute completion rate
    if result['intended_distance'] > 0:
        result['completion_rate'] = actual_distance / result['intended_distance']
    else:
        result['completion_rate'] = 0.0

    # Compute distance from starting pose to first waypoint
    first_waypoint = waypoints[0]
    first_actual = actual_positions[0]
    start_dx = first_actual[0] - first_waypoint[0]
    start_dy = first_actual[1] - first_waypoint[1]
    result['start_distance'] = math.sqrt(start_dx*start_dx + start_dy*start_dy)

    # Compute distance from final pose to final waypoint
    final_waypoint = waypoints[-1]
    final_actual = actual_positions[-1]
    final_dx = final_actual[0] - final_waypoint[0]
    final_dy = final_actual[1] - final_waypoint[1]
    result['final_distance_to_goal'] = math.sqrt(final_dx*final_dx + final_dy*final_dy)

    # Check if goal was reached
    result['goal_reached'] = result['final_distance_to_goal'] <= goal_tolerance

    return result


def compute_rpe(gt_file: Path, est_file: Path, output_dir: Path) -> dict:
    """
    Compute Relative Pose Error (RPE).

    RPE measures the local consistency by comparing relative pose changes
    over fixed distance intervals. Automatically selects appropriate delta
    based on trajectory length.
    """
    evo_rpe = find_evo_binary('evo_rpe')
    results_file = output_dir / 'rpe_results.zip'

    # Estimate trajectory length to choose appropriate delta
    traj_length = estimate_trajectory_length(gt_file)

    # Try multiple delta values, starting with preferred value
    # Use smaller deltas for shorter trajectories
    if traj_length < 5.0:
        deltas = ['0.5', '0.25', '0.1']
    elif traj_length < 20.0:
        deltas = ['1.0', '0.5', '0.25']
    else:
        deltas = ['1.0', '2.0', '0.5']

    for delta in deltas:
        cmd = [
            evo_rpe, 'tum',
            str(gt_file),
            str(est_file),
            '--align',  # Umeyama alignment
            '--delta', delta,
            '--delta_unit', 'm',
            '--t_max_diff', str(MAX_T_DIFF),
            '--save_results', str(results_file),
            '--no_warnings',
            '--silent'
        ]

        result = run_evo_command(cmd, f'RPE computation (delta={delta}m)')

        if 'error' not in result and results_file.exists():
            stats = parse_evo_zip(results_file)
            # Clean up
            results_file.unlink()
            if 'error' not in stats:
                stats['delta_m'] = float(delta)
                return stats

        # Clean up failed attempt
        if results_file.exists():
            results_file.unlink()

    return {'error': f'RPE: Failed with all delta values {deltas} (trajectory length: {traj_length:.1f}m)'}


def evaluate_run(
    gt_file: Path,
    est_file: Path,
    output_file: Path,
    trajectory_file: Path = None,
    goal_tolerance: float = 0.5
) -> dict:
    """
    Run full evaluation pipeline.

    Args:
        gt_file: Ground truth trajectory (TUM format)
        est_file: Estimated trajectory (TUM format)
        output_file: Output metrics.json path
        trajectory_file: Optional intended trajectory CSV for completion metrics
        goal_tolerance: Distance threshold for goal reached (default 0.5m)

    Returns:
        dict with evaluation results and metadata
    """
    output_dir = output_file.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    results = {
        'timestamp': datetime.now().isoformat(),
        'gt_file': str(gt_file),
        'est_file': str(est_file),
        'status': 'unknown',
        'errors': []
    }

    # Validate input files exist
    if not gt_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Ground truth file not found: {gt_file}')
        return results

    if not est_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Estimate file not found: {est_file}')
        return results

    # Count poses
    gt_count = count_poses(gt_file)
    est_count = count_poses(est_file)

    results['gt_poses'] = gt_count
    results['est_poses'] = est_count

    if gt_count < MIN_POSES:
        results['status'] = 'failed'
        results['errors'].append(f'Insufficient ground truth poses: {gt_count} < {MIN_POSES}')
        return results

    if est_count < MIN_POSES:
        results['status'] = 'failed'
        results['errors'].append(f'Insufficient estimate poses: {est_count} < {MIN_POSES}')
        return results

    # Compute ATE
    ate_result = compute_ate(gt_file, est_file, output_dir)
    if 'error' in ate_result:
        results['errors'].append(ate_result['error'])
        results['ate'] = None
    else:
        results['ate'] = {
            'rmse': ate_result.get('rmse'),
            'mean': ate_result.get('mean'),
            'median': ate_result.get('median'),
            'std': ate_result.get('std'),
            'min': ate_result.get('min'),
            'max': ate_result.get('max'),
            'sse': ate_result.get('sse'),
        }

    # Compute RPE
    rpe_result = compute_rpe(gt_file, est_file, output_dir)
    if 'error' in rpe_result:
        results['errors'].append(rpe_result['error'])
        results['rpe'] = None
    else:
        results['rpe'] = {
            'rmse': rpe_result.get('rmse'),
            'mean': rpe_result.get('mean'),
            'median': rpe_result.get('median'),
            'std': rpe_result.get('std'),
            'min': rpe_result.get('min'),
            'max': rpe_result.get('max'),
            'sse': rpe_result.get('sse'),
            'delta_m': rpe_result.get('delta_m'),
        }

    # Compute completion metrics (if trajectory file provided)
    results['completion'] = None
    if trajectory_file and trajectory_file.exists():
        completion_result = compute_completion_metrics(
            est_file, trajectory_file, goal_tolerance
        )
        if 'error' in completion_result:
            results['errors'].append(f"Completion metrics: {completion_result['error']}")
        else:
            results['completion'] = {
                'intended_distance': completion_result['intended_distance'],
                'actual_distance': completion_result['actual_distance'],
                'completion_rate': completion_result['completion_rate'],
                'final_distance_to_goal': completion_result['final_distance_to_goal'],
                'goal_reached': completion_result['goal_reached'],
                'start_distance': completion_result['start_distance'],
            }
    elif trajectory_file:
        results['errors'].append(f'Trajectory file not found: {trajectory_file}')

    # Determine overall status
    if results['ate'] is not None and results['rpe'] is not None:
        results['status'] = 'success'
    elif results['ate'] is not None or results['rpe'] is not None:
        results['status'] = 'partial'
    else:
        results['status'] = 'failed'

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate SLAM trajectory accuracy using ATE/RPE metrics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Evaluate a run directory (expects gt.tum and est.tum)
    python3 evaluate_run.py --run_dir ~/thesis/ros2_ws/results/slam_toolbox_run1

    # Evaluate with trajectory completion metrics
    python3 evaluate_run.py --run_dir ~/thesis/ros2_ws/results/slam_toolbox_run1 \\
        --trajectory ~/thesis/trajectories/traj_01_easy.csv

    # Evaluate specific files
    python3 evaluate_run.py --gt gt.tum --est est.tum --output metrics.json

Output:
    metrics.json with structure:
    {
        "timestamp": "2026-01-25T12:00:00",
        "status": "success|partial|failed",
        "ate": {"rmse": 0.05, "mean": 0.04, ...},
        "rpe": {"rmse": 0.02, "mean": 0.015, ...},
        "completion": {
            "intended_distance": 12.5,
            "actual_distance": 11.8,
            "completion_rate": 0.94,
            "final_distance_to_goal": 0.15,
            "goal_reached": true
        },
        "errors": []
    }
"""
    )

    parser.add_argument(
        '--run_dir',
        type=Path,
        help='Directory containing gt.tum and est.tum files'
    )
    parser.add_argument(
        '--gt',
        type=Path,
        help='Ground truth trajectory file (TUM format)'
    )
    parser.add_argument(
        '--est',
        type=Path,
        help='Estimated trajectory file (TUM format)'
    )
    parser.add_argument(
        '--output', '-o',
        type=Path,
        help='Output metrics.json file path'
    )
    parser.add_argument(
        '--trajectory',
        type=Path,
        help='Intended trajectory CSV file (for completion rate metrics)'
    )
    parser.add_argument(
        '--goal_tolerance',
        type=float,
        default=0.5,
        help='Distance threshold for goal reached (default: 0.5m)'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed output'
    )

    args = parser.parse_args()

    # Determine input/output paths
    if args.run_dir:
        gt_file = args.run_dir / 'gt.tum'
        est_file = args.run_dir / 'est.tum'
        output_file = args.run_dir / 'metrics.json'
    elif args.gt and args.est:
        gt_file = args.gt
        est_file = args.est
        output_file = args.output or Path('metrics.json')
    else:
        parser.error('Either --run_dir or both --gt and --est are required')
        sys.exit(1)

    if args.verbose:
        print(f"Ground truth: {gt_file}")
        print(f"Estimate: {est_file}")
        print(f"Output: {output_file}")
        if args.trajectory:
            print(f"Trajectory: {args.trajectory}")

    # Run evaluation
    results = evaluate_run(
        gt_file, est_file, output_file,
        trajectory_file=args.trajectory,
        goal_tolerance=args.goal_tolerance
    )

    # Save results
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)

    # Print summary
    if args.verbose or results['status'] != 'success':
        print(f"\nStatus: {results['status']}")
        if results.get('ate'):
            print(f"ATE RMSE: {results['ate']['rmse']:.4f} m")
        if results.get('rpe'):
            print(f"RPE RMSE: {results['rpe']['rmse']:.4f} m")
        if results.get('completion'):
            c = results['completion']
            print(f"Completion: {c['completion_rate']*100:.1f}% "
                  f"({c['actual_distance']:.2f}m / {c['intended_distance']:.2f}m)")
            print(f"Final distance to goal: {c['final_distance_to_goal']:.3f} m "
                  f"({'REACHED' if c['goal_reached'] else 'NOT REACHED'})")
        if results['errors']:
            print("Errors:")
            for err in results['errors']:
                print(f"  - {err}")

    print(f"Results saved to: {output_file}")

    # Exit code based on status
    if results['status'] == 'success':
        sys.exit(0)
    elif results['status'] == 'partial':
        sys.exit(0)  # Partial success is still acceptable
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
