#!/usr/bin/env python3
"""
Batch runner for SLAM comparison experiments.

Runs all combinations of (algorithm x goal_set) for N repetitions.
Kills lingering processes between runs to ensure clean state.
Saves a progress log so the batch can be resumed if interrupted.
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

DEFAULT_ALGORITHMS = ['slam_toolbox', 'cartographer']
DEFAULT_GOALS = ['nav_goals_01.yaml', 'nav_goals_02.yaml', 'nav_goals_03.yaml']

# Timeouts tuned per goal set complexity (seconds per goal)
GOAL_TIMEOUTS = {
    'nav_goals_01.yaml': 120,
    'nav_goals_02.yaml': 120,
    'nav_goals_03.yaml': 180,
}

PROCESSES_TO_KILL = [
    'gz sim', 'gzserver', 'gazebo', 'slam_toolbox', 'cartographer_node',
    'cartographer_occupancy_grid', 'lifecycle_manager', 'bt_navigator',
    'controller_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'waypoint_follower', 'parameter_bridge',
    'gt_publisher', 'traj_exporter', 'robot_state_publisher',
    'async_slam_toolbox_node',
]


def cleanup_processes():
    """Kill all lingering ROS/Gazebo processes."""
    for proc_name in PROCESSES_TO_KILL:
        try:
            subprocess.run(
                ['pkill', '-f', proc_name],
                capture_output=True, timeout=5,
            )
        except Exception:
            pass
    time.sleep(3)


def load_progress(log_path):
    """Load previously completed experiments."""
    if not os.path.exists(log_path):
        return []
    try:
        with open(log_path, 'r') as f:
            return json.load(f)
    except Exception:
        return []


def save_progress(log_path, completed):
    """Save progress log."""
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, 'w') as f:
        json.dump(completed, f, indent=2)


def run_single_experiment(algorithm, goals_file, run_num, script_path, output_dir, verbose=False):
    """Run a single experiment and return success status + result summary."""
    timeout_per_goal = GOAL_TIMEOUTS.get(goals_file, 120)

    cmd = [
        'python3', script_path,
        '--algorithm', algorithm,
        '--goals', goals_file,
        '--output_dir', output_dir,
        '--timeout_per_goal', str(timeout_per_goal),
    ]
    if verbose:
        cmd.append('--verbose')

    print(f"\n{'='*70}")
    print(f"Running: {algorithm} + {goals_file} (run {run_num})")
    print(f"Timeout per goal: {timeout_per_goal}s")
    print(f"{'='*70}")

    start = time.time()
    try:
        # Generous overall timeout: ~12 minutes max per experiment
        result = subprocess.run(
            cmd,
            timeout=900,
            capture_output=not verbose,
            text=True,
        )
        elapsed = time.time() - start

        if result.returncode != 0:
            print(f"FAILED (exit code {result.returncode}) after {elapsed:.1f}s")
            if not verbose and result.stderr:
                print("stderr tail:")
                print(result.stderr[-1000:])
            return False, None

        print(f"COMPLETED in {elapsed:.1f}s")
        return True, elapsed

    except subprocess.TimeoutExpired:
        print(f"TIMED OUT after 900s")
        return False, None
    except Exception as e:
        print(f"ERROR: {e}")
        return False, None


def find_latest_result(output_dir, algorithm, goals_file):
    """Find the most recent result directory for a given config."""
    goals_name = goals_file.replace('.yaml', '')
    pattern = f'nav_{algorithm}_{goals_name}_'
    results_path = Path(output_dir)
    matching = sorted(
        [d for d in results_path.iterdir() if d.is_dir() and d.name.startswith(pattern)],
        key=lambda d: d.stat().st_mtime,
        reverse=True,
    )
    return matching[0] if matching else None


def extract_metrics(result_dir):
    """Extract key metrics from nav_results.json."""
    nav_results = result_dir / 'nav_results.json'
    if not nav_results.exists():
        return {}
    try:
        with open(nav_results, 'r') as f:
            data = json.load(f)

        metrics = {
            'success_rate': data.get('success_rate'),
            'total_time_s': data.get('total_time_s'),
            'goals_succeeded': data.get('goals_succeeded'),
            'goals_attempted': data.get('goals_attempted'),
        }
        slam = data.get('slam_accuracy') or {}
        if slam.get('success') and slam.get('ate'):
            metrics['ate_rmse_m'] = slam['ate']['rmse']
            metrics['ate_mean_m'] = slam['ate']['mean']
            metrics['rpe_rmse_m'] = slam['rpe']['rmse']
        return metrics
    except Exception:
        return {}


def main():
    parser = argparse.ArgumentParser(description='Batch SLAM experiment runner.')
    parser.add_argument('--repetitions', '-n', type=int, default=3,
                        help='Number of repetitions per config (default: 3)')
    parser.add_argument('--algorithms', nargs='+', default=DEFAULT_ALGORITHMS,
                        help='Algorithms to test')
    parser.add_argument('--goals', nargs='+', default=DEFAULT_GOALS,
                        help='Goal YAML files to test')
    parser.add_argument('--output_dir', default=os.path.expanduser('~/thesis/ros2_ws/results'),
                        help='Output directory for results')
    parser.add_argument('--resume', action='store_true',
                        help='Resume from previous batch progress log')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output (shows all subprocess output)')
    args = parser.parse_args()

    # Locate run_nav_experiment.py (same directory as this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    nav_script = os.path.join(script_dir, 'run_nav_experiment.py')
    if not os.path.exists(nav_script):
        print(f"ERROR: Cannot find run_nav_experiment.py at {nav_script}", file=sys.stderr)
        sys.exit(1)

    # Setup progress log
    log_path = os.path.join(args.output_dir, 'batch_progress.json')
    completed = load_progress(log_path) if args.resume else []
    completed_keys = {(c['algorithm'], c['goals_file'], c['run_num']) for c in completed}

    # Build job list
    jobs = []
    for algo in args.algorithms:
        for goals in args.goals:
            for run in range(1, args.repetitions + 1):
                jobs.append((algo, goals, run))

    total = len(jobs)
    skipped = sum(1 for j in jobs if j in completed_keys)
    remaining = total - skipped

    print(f"\n{'#'*70}")
    print(f"# BATCH SLAM EXPERIMENTS")
    print(f"#{'-'*69}")
    print(f"# Algorithms: {args.algorithms}")
    print(f"# Goal sets:  {args.goals}")
    print(f"# Repetitions: {args.repetitions}")
    print(f"# Total jobs:  {total}")
    print(f"# Completed:   {skipped}")
    print(f"# Remaining:   {remaining}")
    print(f"# Output:      {args.output_dir}")
    print(f"{'#'*70}\n")

    if remaining == 0:
        print("All jobs already complete. Nothing to do.")
        return 0

    batch_start = time.time()

    for idx, (algo, goals, run) in enumerate(jobs, start=1):
        if (algo, goals, run) in completed_keys:
            print(f"[{idx}/{total}] SKIP (already done): {algo} + {goals} run {run}")
            continue

        # Clean up before each run
        print(f"\n[{idx}/{total}] Cleaning up processes...")
        cleanup_processes()

        # Run experiment
        success, elapsed = run_single_experiment(
            algo, goals, run, nav_script, args.output_dir, args.verbose,
        )

        # Record result
        result_dir = find_latest_result(args.output_dir, algo, goals)
        metrics = extract_metrics(result_dir) if (success and result_dir) else {}

        record = {
            'algorithm': algo,
            'goals_file': goals,
            'run_num': run,
            'success': success,
            'elapsed_s': elapsed,
            'result_dir': str(result_dir) if result_dir else None,
            'metrics': metrics,
            'timestamp': datetime.now().isoformat(),
        }
        completed.append(record)
        save_progress(log_path, completed)

        # Print progress summary
        batch_elapsed = time.time() - batch_start
        done = idx - skipped
        avg_per_job = batch_elapsed / done if done > 0 else 0
        eta = avg_per_job * (total - idx) if done > 0 else 0
        print(f"\n>>> Progress: {done}/{remaining} done, "
              f"batch elapsed: {batch_elapsed/60:.1f} min, "
              f"ETA: {eta/60:.1f} min")

        if metrics:
            ate_cm = metrics.get('ate_rmse_m', 0) * 100 if metrics.get('ate_rmse_m') else None
            ate_str = f"{ate_cm:.2f} cm" if ate_cm else "N/A"
            print(f">>> Result: {metrics.get('goals_succeeded')}/{metrics.get('goals_attempted')} goals, "
                  f"ATE={ate_str}, time={metrics.get('total_time_s', 0):.1f}s")

    # Final cleanup
    cleanup_processes()

    # Print summary
    print(f"\n{'#'*70}")
    print(f"# BATCH COMPLETE")
    print(f"#{'-'*69}")
    print(f"# Total time: {(time.time()-batch_start)/60:.1f} minutes")
    print(f"# Progress log: {log_path}")
    print(f"{'#'*70}\n")

    # Quick summary table
    print("Summary:")
    print(f"{'Algorithm':<15} {'Goal set':<22} {'Run':<5} {'Success':<10} {'ATE (cm)':<10}")
    print('-' * 70)
    for rec in completed:
        ate = rec.get('metrics', {}).get('ate_rmse_m')
        ate_str = f"{ate*100:.2f}" if ate else 'N/A'
        succ_str = 'OK' if rec['success'] else 'FAIL'
        print(f"{rec['algorithm']:<15} {rec['goals_file']:<22} {rec['run_num']:<5} {succ_str:<10} {ate_str:<10}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
