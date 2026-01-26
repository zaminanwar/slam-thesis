#!/usr/bin/env python3
"""
compute_cte.py - Compute Cross-Track Error (CTE) for path following evaluation.

Post-processing script that compares the actual robot path against the intended
trajectory to measure path following accuracy.

For each actual pose, finds the closest point on the intended path and computes
the perpendicular distance (signed CTE).

Usage:
    python3 compute_cte.py --trajectory traj_01_easy.csv --actual est.tum --output path_deviation.json
    python3 compute_cte.py --run_dir /path/to/run --trajectory_dir ~/thesis/trajectories

Outputs:
    path_deviation.json with CTE statistics and timeline
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path
from datetime import datetime


def load_trajectory_csv(filepath: Path) -> list:
    """
    Load intended trajectory from CSV file.

    Format: x,y,yaw,speed (header line expected)

    Returns list of (x, y, yaw) tuples.
    """
    waypoints = []
    with open(filepath, 'r') as f:
        header = f.readline()  # Skip header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) >= 3:
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2])
                waypoints.append((x, y, yaw))
    return waypoints


def load_tum_file(filepath: Path) -> list:
    """
    Load actual trajectory from TUM format file.

    Format: timestamp tx ty tz qx qy qz qw

    Returns list of (timestamp, x, y, z, qx, qy, qz, qw) tuples.
    """
    poses = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 8:
                timestamp = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                poses.append((timestamp, tx, ty, tz, qx, qy, qz, qw))
    return poses


def interpolate_path(waypoints: list, resolution: float = 0.05) -> list:
    """
    Interpolate between waypoints to create a dense path for CTE computation.

    Args:
        waypoints: List of (x, y, yaw) tuples
        resolution: Distance between interpolated points in meters

    Returns:
        List of (x, y) points along the path, plus segment info for signed CTE
    """
    if len(waypoints) < 2:
        return [(w[0], w[1]) for w in waypoints]

    path = []
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i][0], waypoints[i][1]
        x2, y2 = waypoints[i + 1][0], waypoints[i + 1][1]

        dx = x2 - x1
        dy = y2 - y1
        segment_length = math.sqrt(dx * dx + dy * dy)

        if segment_length < 1e-6:
            continue

        # Number of points for this segment
        n_points = max(2, int(segment_length / resolution))

        for j in range(n_points):
            t = j / (n_points - 1) if n_points > 1 else 0
            x = x1 + t * dx
            y = y1 + t * dy
            # Store point with segment direction for signed CTE
            path.append((x, y, dx, dy))

    # Add final point
    if waypoints:
        last = waypoints[-1]
        if path:
            path.append((last[0], last[1], path[-1][2], path[-1][3]))
        else:
            path.append((last[0], last[1], 0, 0))

    return path


def compute_signed_cte(actual_x: float, actual_y: float,
                       path_x: float, path_y: float,
                       dir_x: float, dir_y: float) -> float:
    """
    Compute signed cross-track error.

    Positive CTE = robot is to the left of the path (in direction of travel)
    Negative CTE = robot is to the right of the path

    Args:
        actual_x, actual_y: Actual robot position
        path_x, path_y: Closest point on path
        dir_x, dir_y: Path direction vector at closest point

    Returns:
        Signed CTE (meters)
    """
    # Vector from path point to actual position
    to_robot_x = actual_x - path_x
    to_robot_y = actual_y - path_y

    # Normalize direction vector
    dir_length = math.sqrt(dir_x * dir_x + dir_y * dir_y)
    if dir_length < 1e-6:
        # No direction info, return unsigned distance
        return math.sqrt(to_robot_x * to_robot_x + to_robot_y * to_robot_y)

    dir_x_norm = dir_x / dir_length
    dir_y_norm = dir_y / dir_length

    # Cross product gives signed perpendicular distance
    # Positive when robot is to the left of path direction
    cte = dir_x_norm * to_robot_y - dir_y_norm * to_robot_x

    return cte


def find_closest_point_on_path(actual_x: float, actual_y: float,
                                path: list) -> tuple:
    """
    Find the closest point on the interpolated path to the actual position.

    Returns:
        (path_x, path_y, dir_x, dir_y, distance)
    """
    min_dist = float('inf')
    closest = None

    for px, py, dx, dy in path:
        dist = math.sqrt((actual_x - px) ** 2 + (actual_y - py) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest = (px, py, dx, dy, dist)

    return closest


def compute_cte_metrics(trajectory_file: Path, actual_file: Path) -> dict:
    """
    Compute cross-track error metrics.

    Args:
        trajectory_file: Path to intended trajectory CSV
        actual_file: Path to actual trajectory TUM file

    Returns:
        Dict with CTE statistics and timeline
    """
    results = {
        'timestamp': datetime.now().isoformat(),
        'trajectory_file': str(trajectory_file),
        'actual_file': str(actual_file),
        'status': 'unknown',
        'errors': []
    }

    # Validate input files
    if not trajectory_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Trajectory file not found: {trajectory_file}')
        return results

    if not actual_file.exists():
        results['status'] = 'failed'
        results['errors'].append(f'Actual trajectory file not found: {actual_file}')
        return results

    try:
        # Load data
        waypoints = load_trajectory_csv(trajectory_file)
        actual_poses = load_tum_file(actual_file)

        if len(waypoints) < 2:
            results['status'] = 'failed'
            results['errors'].append(f'Insufficient waypoints in trajectory: {len(waypoints)}')
            return results

        if len(actual_poses) < 2:
            results['status'] = 'failed'
            results['errors'].append(f'Insufficient poses in actual trajectory: {len(actual_poses)}')
            return results

        # Interpolate intended path
        path = interpolate_path(waypoints, resolution=0.05)

        if len(path) < 2:
            results['status'] = 'failed'
            results['errors'].append('Failed to interpolate path')
            return results

        # Compute CTE for each actual pose
        cte_values = []
        cte_timeline = []

        for pose in actual_poses:
            timestamp, actual_x, actual_y = pose[0], pose[1], pose[2]

            # Find closest point on path
            closest = find_closest_point_on_path(actual_x, actual_y, path)
            if closest is None:
                continue

            path_x, path_y, dir_x, dir_y, dist = closest

            # Compute signed CTE
            cte = compute_signed_cte(actual_x, actual_y, path_x, path_y, dir_x, dir_y)

            cte_values.append(cte)
            cte_timeline.append({
                'timestamp': timestamp,
                'cte': round(cte, 6),
                'actual_x': round(actual_x, 4),
                'actual_y': round(actual_y, 4),
                'path_x': round(path_x, 4),
                'path_y': round(path_y, 4)
            })

        if not cte_values:
            results['status'] = 'failed'
            results['errors'].append('No valid CTE measurements computed')
            return results

        # Compute statistics
        abs_cte = [abs(c) for c in cte_values]

        mean_cte = sum(abs_cte) / len(abs_cte)
        max_cte = max(abs_cte)
        min_cte = min(abs_cte)

        # RMSE of CTE
        rmse_cte = math.sqrt(sum(c * c for c in cte_values) / len(cte_values))

        # Standard deviation
        mean_signed = sum(cte_values) / len(cte_values)
        variance = sum((c - mean_signed) ** 2 for c in cte_values) / len(cte_values)
        std_cte = math.sqrt(variance)

        # Store results
        results['status'] = 'success'
        results['waypoint_count'] = len(waypoints)
        results['path_points'] = len(path)
        results['pose_count'] = len(actual_poses)
        results['cte_count'] = len(cte_values)

        results['metrics'] = {
            'mean_cte': round(mean_cte, 6),
            'max_cte': round(max_cte, 6),
            'min_cte': round(min_cte, 6),
            'rmse_cte': round(rmse_cte, 6),
            'std_cte': round(std_cte, 6),
            'mean_signed_cte': round(mean_signed, 6),  # Shows bias (left/right tendency)
        }

        # Store timeline (can be large, consider subsampling for very long runs)
        if len(cte_timeline) > 1000:
            # Subsample to ~1000 points for storage efficiency
            step = len(cte_timeline) // 1000
            results['cte_timeline'] = cte_timeline[::step]
            results['cte_timeline_subsampled'] = True
        else:
            results['cte_timeline'] = cte_timeline
            results['cte_timeline_subsampled'] = False

    except Exception as e:
        results['status'] = 'failed'
        results['errors'].append(f'Exception during CTE computation: {str(e)}')

    return results


def find_trajectory_name_from_run_info(run_dir: Path) -> str:
    """Try to find the trajectory name from run_info.json."""
    run_info_file = run_dir / 'run_info.json'
    if run_info_file.exists():
        try:
            with open(run_info_file, 'r') as f:
                run_info = json.load(f)
                return run_info.get('trajectory', '')
        except:
            pass
    return ''


def main():
    parser = argparse.ArgumentParser(
        description='Compute Cross-Track Error (CTE) for path following evaluation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Compute CTE for specific files
    python3 compute_cte.py --trajectory ~/thesis/trajectories/traj_01_easy.csv \\
                           --actual ~/thesis/ros2_ws/results/run1/est.tum \\
                           --output ~/thesis/ros2_ws/results/run1/path_deviation.json

    # Compute CTE for a run directory (auto-detect trajectory from run_info.json)
    python3 compute_cte.py --run_dir ~/thesis/ros2_ws/results/run1 \\
                           --trajectory_dir ~/thesis/trajectories

Output:
    path_deviation.json with structure:
    {
        "status": "success",
        "metrics": {
            "mean_cte": 0.05,
            "max_cte": 0.15,
            "rmse_cte": 0.06,
            "std_cte": 0.03
        },
        "cte_timeline": [...]
    }

Notes:
    - CTE is signed: positive = left of path, negative = right of path
    - mean_cte uses absolute values (typical metric)
    - mean_signed_cte shows directional bias
"""
    )

    parser.add_argument(
        '--trajectory',
        type=Path,
        help='Intended trajectory CSV file (x,y,yaw,speed format)'
    )
    parser.add_argument(
        '--actual',
        type=Path,
        help='Actual trajectory file (TUM format, typically est.tum)'
    )
    parser.add_argument(
        '--output', '-o',
        type=Path,
        help='Output path_deviation.json file'
    )
    parser.add_argument(
        '--run_dir',
        type=Path,
        help='Run directory containing est.tum (auto-detect trajectory from run_info.json)'
    )
    parser.add_argument(
        '--trajectory_dir',
        type=Path,
        default=Path.home() / 'thesis' / 'trajectories',
        help='Directory containing trajectory CSV files (default: ~/thesis/trajectories)'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed output'
    )

    args = parser.parse_args()

    # Determine input/output paths
    if args.run_dir:
        actual_file = args.run_dir / 'est.tum'
        output_file = args.run_dir / 'path_deviation.json'

        # Try to find trajectory name from run_info.json
        traj_name = find_trajectory_name_from_run_info(args.run_dir)
        if traj_name:
            trajectory_file = args.trajectory_dir / f'{traj_name}.csv'
        elif args.trajectory:
            trajectory_file = args.trajectory
        else:
            print("Error: Could not determine trajectory file.")
            print("  Either specify --trajectory or ensure run_info.json contains 'trajectory' field")
            sys.exit(1)
    elif args.trajectory and args.actual:
        trajectory_file = args.trajectory
        actual_file = args.actual
        output_file = args.output or Path('path_deviation.json')
    else:
        parser.error('Either --run_dir or both --trajectory and --actual are required')
        sys.exit(1)

    if args.verbose:
        print(f"Trajectory: {trajectory_file}")
        print(f"Actual: {actual_file}")
        print(f"Output: {output_file}")

    # Compute CTE
    results = compute_cte_metrics(trajectory_file, actual_file)

    # Save results
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)

    # Print summary
    if args.verbose or results['status'] != 'success':
        print(f"\nStatus: {results['status']}")
        if results.get('metrics'):
            m = results['metrics']
            print(f"Mean CTE: {m['mean_cte']:.4f} m")
            print(f"Max CTE: {m['max_cte']:.4f} m")
            print(f"RMSE CTE: {m['rmse_cte']:.4f} m")
            print(f"Std CTE: {m['std_cte']:.4f} m")
            print(f"Signed mean: {m['mean_signed_cte']:.4f} m (bias)")
        if results.get('errors'):
            print("Errors:")
            for err in results['errors']:
                print(f"  - {err}")

    print(f"Results saved to: {output_file}")

    # Exit code based on status
    sys.exit(0 if results['status'] == 'success' else 1)


if __name__ == '__main__':
    main()
