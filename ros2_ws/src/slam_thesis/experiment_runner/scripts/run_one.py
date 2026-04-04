#!/usr/bin/env python3
"""
Single experiment orchestrator for SLAM evaluation.

Runs one SLAM algorithm on one recorded bag, exports trajectories,
and computes evaluation metrics.

Usage:
    python3 run_one.py --bag_path /path/to/bag --algorithm slam_toolbox
    python3 run_one.py --bag_path /path/to/bag --algorithm cartographer --timeout 300

Output:
    Creates in the bag directory:
    - gt_trajectory.tum: Ground truth trajectory (map_gt -> base_footprint_gt, true GT from Gazebo)
    - est_trajectory.tum: SLAM estimated trajectory (map -> base_footprint)
    - results/metrics.json: ATE/RPE evaluation metrics
    - results/ate_plot.png: Trajectory comparison plot (optional)
    - results/rpe_plot.png: RPE distribution plot (optional)
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
from typing import Optional, Tuple

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


def validate_bag_path(bag_path: str) -> str:
    """Validate and resolve bag path."""
    bag_path = os.path.expanduser(bag_path)
    bag_path = os.path.abspath(bag_path)

    if not os.path.exists(bag_path):
        raise ValueError(f"Bag path does not exist: {bag_path}")

    # Check for metadata.yaml (indicates valid rosbag)
    metadata_file = os.path.join(bag_path, 'metadata.yaml')
    if not os.path.exists(metadata_file):
        raise ValueError(f"Not a valid rosbag (missing metadata.yaml): {bag_path}")

    return bag_path


def get_bag_duration(bag_path: str) -> float:
    """Get the duration of a rosbag in seconds."""
    try:
        result = subprocess.run(
            ['ros2', 'bag', 'info', bag_path],
            capture_output=True,
            text=True,
            timeout=30
        )

        # Parse duration from output: "Duration: 123.456s"
        for line in result.stdout.split('\n'):
            if 'Duration:' in line:
                # Extract number before 's'
                parts = line.split(':')
                if len(parts) >= 2:
                    duration_str = parts[1].strip().rstrip('s')
                    return float(duration_str)

        # Default if parsing fails
        return 120.0
    except Exception:
        return 120.0  # Default 2 minutes


def extract_dataset_name(bag_path: str) -> str:
    """Extract dataset name from bag path."""
    return os.path.basename(bag_path.rstrip('/'))


def run_slam_with_export(
    bag_path: str,
    algorithm: str,
    gt_file: str,
    est_file: str,
    rate: float = 1.0,
    timeout: float = 300.0,
    verbose: bool = False
) -> Tuple[bool, str]:
    """
    Run SLAM with trajectory export and wait for completion.

    Args:
        bag_path: Path to the rosbag
        algorithm: SLAM algorithm (slam_toolbox or cartographer)
        gt_file: Output path for ground truth trajectory
        est_file: Output path for estimated trajectory
        rate: Playback rate multiplier
        timeout: Maximum time to wait in seconds
        verbose: Print verbose output

    Returns:
        Tuple of (success, error_message)
    """
    processes = []

    try:
        # Determine launch file
        if algorithm == 'slam_toolbox':
            slam_launch = 'slam_toolbox.launch.py'
            slam_package = 'slam_launch'
        elif algorithm == 'cartographer':
            slam_launch = 'cartographer.launch.py'
            slam_package = 'slam_launch'
        else:
            return False, f"Unknown algorithm: {algorithm}"

        # Build SLAM launch command
        slam_cmd = [
            'ros2', 'launch', slam_package, slam_launch,
            f'bag_path:={bag_path}',
            f'rate:={rate}',
            'use_sim_time:=true',
            'loop:=false',
        ]

        if verbose:
            print(f"[run_one] Starting SLAM: {algorithm}")
            print(f"[run_one] Command: {' '.join(slam_cmd)}")

        # Start SLAM process
        slam_proc = subprocess.Popen(
            slam_cmd,
            stdout=subprocess.PIPE if not verbose else None,
            stderr=subprocess.PIPE if not verbose else None,
            preexec_fn=os.setsid  # Create new process group for cleanup
        )
        processes.append(slam_proc)

        # Wait for SLAM to initialize (matches launch file delay)
        time.sleep(3.0)

        # Build GT exporter command (map_gt -> base_footprint_gt for TRUE ground truth)
        gt_export_cmd = [
            'ros2', 'launch', 'traj_exporter', 'export_trajectory.launch.py',
            f'output_file:={gt_file}',
            'parent_frame:=map_gt',
            'child_frame:=base_footprint_gt',
            'sample_rate:=10.0',
            'use_sim_time:=true',
        ]

        if verbose:
            print(f"[run_one] Starting GT exporter: map_gt -> base_footprint_gt")

        # Start GT exporter
        gt_proc = subprocess.Popen(
            gt_export_cmd,
            stdout=subprocess.PIPE if not verbose else None,
            stderr=subprocess.PIPE if not verbose else None,
            preexec_fn=os.setsid
        )
        processes.append(gt_proc)

        # Build estimated trajectory exporter command (map -> base_footprint for SLAM output)
        est_export_cmd = [
            'ros2', 'launch', 'traj_exporter', 'export_trajectory.launch.py',
            f'output_file:={est_file}',
            'parent_frame:=map',
            'child_frame:=base_footprint',
            'sample_rate:=10.0',
            'use_sim_time:=true',
        ]

        if verbose:
            print(f"[run_one] Starting SLAM exporter: map -> base_footprint")

        # Start EST exporter
        est_proc = subprocess.Popen(
            est_export_cmd,
            stdout=subprocess.PIPE if not verbose else None,
            stderr=subprocess.PIPE if not verbose else None,
            preexec_fn=os.setsid
        )
        processes.append(est_proc)

        # Wait for SLAM process to complete (indicates bag finished)
        start_time = time.time()
        while slam_proc.poll() is None:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                raise TimeoutError(f"SLAM process timed out after {timeout}s")
            time.sleep(1.0)

        # Give exporters a moment to finish writing
        time.sleep(2.0)

        # Check SLAM exit code
        if slam_proc.returncode != 0:
            # Non-zero exit might be normal for bag end, check if outputs exist
            pass

        if verbose:
            print(f"[run_one] SLAM process completed with code: {slam_proc.returncode}")

        return True, ""

    except TimeoutError as e:
        return False, str(e)
    except Exception as e:
        return False, f"Error running SLAM: {e}"
    finally:
        # Clean up all processes
        for proc in processes:
            if proc.poll() is None:
                try:
                    # Kill the process group
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass


def verify_trajectory_files(gt_file: str, est_file: str) -> Tuple[bool, str]:
    """
    Verify that trajectory files exist and have content.

    Returns:
        Tuple of (success, error_message)
    """
    for name, filepath in [('Ground truth', gt_file), ('Estimated', est_file)]:
        if not os.path.exists(filepath):
            return False, f"{name} trajectory file not created: {filepath}"

        # Check file has content (more than just header)
        with open(filepath, 'r') as f:
            lines = [l for l in f.readlines() if l.strip() and not l.startswith('#')]
            if len(lines) < 2:
                return False, f"{name} trajectory file has too few poses: {len(lines)}"

    return True, ""


def run_evaluation(
    gt_file: str,
    est_file: str,
    output_dir: str,
    dataset: str,
    algorithm: str,
    save_plots: bool = True,
    verbose: bool = False
) -> Tuple[bool, str, Optional[dict]]:
    """
    Run the evaluation script.

    Returns:
        Tuple of (success, error_message, metrics_dict)
    """
    try:
        evaluate_script = find_script_path('evaluate_run.py')
    except FileNotFoundError as e:
        return False, str(e), None

    # Build evaluation command
    eval_cmd = [
        sys.executable, evaluate_script,
        '--gt_file', gt_file,
        '--est_file', est_file,
        '--output_dir', output_dir,
        '--dataset', dataset,
        '--algorithm', algorithm,
    ]

    if save_plots:
        eval_cmd.append('--save_plots')

    if verbose:
        eval_cmd.append('--verbose')
        print(f"[run_one] Running evaluation")
        print(f"[run_one] Command: {' '.join(eval_cmd)}")

    try:
        result = subprocess.run(
            eval_cmd,
            capture_output=True,
            text=True,
            timeout=120
        )

        if verbose:
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(result.stderr, file=sys.stderr)

        # Load metrics.json
        metrics_path = os.path.join(output_dir, 'metrics.json')
        if os.path.exists(metrics_path):
            with open(metrics_path, 'r') as f:
                metrics = json.load(f)
            return metrics.get('success', False), metrics.get('error', ''), metrics
        else:
            return False, "metrics.json not created", None

    except subprocess.TimeoutExpired:
        return False, "Evaluation script timed out", None
    except Exception as e:
        return False, f"Evaluation error: {e}", None


def main():
    parser = argparse.ArgumentParser(
        description='Run a single SLAM experiment with evaluation.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --bag_path ~/thesis/ros2_ws/bags/traj_01_easy_baseline --algorithm slam_toolbox
  %(prog)s --bag_path /path/to/bag --algorithm cartographer --rate 0.5 --verbose
  %(prog)s --bag_path /path/to/bag --algorithm slam_toolbox --timeout 600
        """
    )

    parser.add_argument(
        '--bag_path',
        type=str,
        required=True,
        help='Path to the rosbag directory'
    )

    parser.add_argument(
        '--algorithm',
        type=str,
        required=True,
        choices=VALID_ALGORITHMS,
        help='SLAM algorithm to use'
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
        help='Maximum time for SLAM process in seconds (default: 300)'
    )

    parser.add_argument(
        '--output_dir',
        type=str,
        default=None,
        help='Output directory for results (default: bag_path/results)'
    )

    parser.add_argument(
        '--save_plots',
        action='store_true',
        default=True,
        help='Save visualization plots (default: True)'
    )

    parser.add_argument(
        '--no_plots',
        action='store_true',
        help='Disable plot generation'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Validate inputs
    try:
        bag_path = validate_bag_path(args.bag_path)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Set up paths
    dataset_name = extract_dataset_name(bag_path)
    output_dir = args.output_dir or os.path.join(bag_path, 'results', args.algorithm)
    gt_file = os.path.join(bag_path, 'gt_trajectory.tum')
    est_file = os.path.join(bag_path, f'{args.algorithm}_trajectory.tum')
    save_plots = args.save_plots and not args.no_plots

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Print configuration
    if args.verbose:
        print(f"\n{'='*60}")
        print(f"SLAM Experiment: {dataset_name}")
        print(f"{'='*60}")
        print(f"Bag path:   {bag_path}")
        print(f"Algorithm:  {args.algorithm}")
        print(f"Rate:       {args.rate}x")
        print(f"Timeout:    {args.timeout}s")
        print(f"Output:     {output_dir}")
        print(f"{'='*60}\n")

    # Get bag duration for timeout estimation
    bag_duration = get_bag_duration(bag_path)
    effective_timeout = max(args.timeout, (bag_duration / args.rate) + 60)

    if args.verbose:
        print(f"[run_one] Bag duration: {bag_duration:.1f}s")
        print(f"[run_one] Effective timeout: {effective_timeout:.1f}s")

    # Run SLAM with trajectory export
    print(f"[run_one] Running {args.algorithm} on {dataset_name}...")
    success, error = run_slam_with_export(
        bag_path=bag_path,
        algorithm=args.algorithm,
        gt_file=gt_file,
        est_file=est_file,
        rate=args.rate,
        timeout=effective_timeout,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_one] SLAM failed: {error}", file=sys.stderr)
        # Create failure metrics.json
        failure_result = {
            'success': False,
            'dataset': dataset_name,
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"SLAM execution failed: {error}",
            'ate': None,
            'rpe': None,
        }
        with open(os.path.join(output_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    # Verify trajectory files
    success, error = verify_trajectory_files(gt_file, est_file)
    if not success:
        print(f"[run_one] Trajectory verification failed: {error}", file=sys.stderr)
        failure_result = {
            'success': False,
            'dataset': dataset_name,
            'algorithm': args.algorithm,
            'timestamp': datetime.now().isoformat(),
            'error': f"Trajectory export failed: {error}",
            'ate': None,
            'rpe': None,
        }
        with open(os.path.join(output_dir, 'metrics.json'), 'w') as f:
            json.dump(failure_result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"[run_one] Trajectory files verified")
        print(f"[run_one]   GT: {gt_file}")
        print(f"[run_one]   EST: {est_file}")

    # Run evaluation
    print(f"[run_one] Running evaluation...")
    success, error, metrics = run_evaluation(
        gt_file=gt_file,
        est_file=est_file,
        output_dir=output_dir,
        dataset=dataset_name,
        algorithm=args.algorithm,
        save_plots=save_plots,
        verbose=args.verbose
    )

    if not success:
        print(f"[run_one] Evaluation failed: {error}", file=sys.stderr)
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
    print(f"Results saved to: {output_dir}")

    sys.exit(0)


if __name__ == '__main__':
    main()
