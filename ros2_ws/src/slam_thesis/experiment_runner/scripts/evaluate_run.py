#!/usr/bin/env python3
"""
Per-run evaluation script for SLAM trajectory comparison.

Uses the evo library to compute ATE (Absolute Trajectory Error) and
RPE (Relative Pose Error) metrics by comparing estimated trajectory
against ground truth.

Usage:
    python3 evaluate_run.py --gt_file /path/to/gt.tum --est_file /path/to/est.tum --output_dir /path/to/output

Output:
    - metrics.json: JSON file with ATE/RPE statistics
    - ate_plot.png: ATE visualization (optional, if --save_plots)
    - rpe_plot.png: RPE visualization (optional, if --save_plots)
"""

import argparse
import copy
import json
import os
import sys
from datetime import datetime
from typing import Any, Dict, Optional

import numpy as np

try:
    from evo.core import metrics, sync
    from evo.core.trajectory import PoseTrajectory3D
    from evo.tools import file_interface
    from evo.core.metrics import PoseRelation, Unit
except ImportError as e:
    print(f"Error: evo library not installed. Install with: pip3 install evo", file=sys.stderr)
    print(f"Import error: {e}", file=sys.stderr)
    sys.exit(1)


def load_trajectory(filepath: str) -> Optional[PoseTrajectory3D]:
    """
    Load a TUM format trajectory file.

    Args:
        filepath: Path to the TUM format trajectory file

    Returns:
        PoseTrajectory3D object or None if loading fails
    """
    if not os.path.exists(filepath):
        return None

    try:
        traj = file_interface.read_tum_trajectory_file(filepath)
        return traj
    except Exception as e:
        print(f"Error loading trajectory from {filepath}: {e}", file=sys.stderr)
        return None


def compute_metrics(traj_gt: PoseTrajectory3D, traj_est: PoseTrajectory3D) -> Dict[str, Any]:
    """
    Compute ATE and RPE metrics between ground truth and estimated trajectories.

    Args:
        traj_gt: Ground truth trajectory
        traj_est: Estimated trajectory

    Returns:
        Dictionary containing ATE and RPE statistics
    """
    # Associate trajectories by timestamps
    max_diff = 0.05  # Maximum timestamp difference for matching (50ms)
    try:
        traj_gt_sync, traj_est_sync = sync.associate_trajectories(
            traj_gt, traj_est, max_diff=max_diff
        )
    except Exception as e:
        raise RuntimeError(f"Failed to associate trajectories: {e}")

    if len(traj_gt_sync.timestamps) < 2:
        raise RuntimeError(
            f"Not enough synchronized poses: {len(traj_gt_sync.timestamps)} "
            "(need at least 2 for RPE)"
        )

    # Align trajectories using Umeyama alignment (SE3)
    try:
        traj_est_aligned = copy.deepcopy(traj_est_sync)
        traj_est_aligned.align(traj_gt_sync, correct_scale=False, correct_only_scale=False)
    except Exception as e:
        raise RuntimeError(f"Failed to align trajectories: {e}")

    # Compute ATE (Absolute Trajectory Error)
    ate_metric = metrics.APE(PoseRelation.translation_part)
    ate_metric.process_data((traj_gt_sync, traj_est_aligned))
    ate_stats = ate_metric.get_all_statistics()

    # Compute RPE (Relative Pose Error) - translation part
    # Use frame-based delta for robustness with short trajectories
    rpe_metric = metrics.RPE(
        PoseRelation.translation_part,
        delta=1,
        delta_unit=Unit.frames,
        all_pairs=False
    )
    rpe_metric.process_data((traj_gt_sync, traj_est_aligned))
    rpe_stats = rpe_metric.get_all_statistics()

    # Compute trajectory length
    try:
        traj_length = float(traj_gt_sync.path_length)
    except Exception:
        traj_length = 0.0

    # Compute duration
    try:
        duration = float(traj_gt_sync.timestamps[-1] - traj_gt_sync.timestamps[0])
    except Exception:
        duration = 0.0

    return {
        "ate": {
            "rmse": float(ate_stats["rmse"]),
            "mean": float(ate_stats["mean"]),
            "median": float(ate_stats["median"]),
            "std": float(ate_stats["std"]),
            "min": float(ate_stats["min"]),
            "max": float(ate_stats["max"]),
        },
        "rpe": {
            "rmse": float(rpe_stats["rmse"]),
            "mean": float(rpe_stats["mean"]),
            "median": float(rpe_stats["median"]),
            "std": float(rpe_stats["std"]),
            "min": float(rpe_stats["min"]),
            "max": float(rpe_stats["max"]),
        },
        "trajectory_length_m": traj_length,
        "duration_s": duration,
        "num_poses": len(traj_gt_sync.timestamps),
    }


def save_plots(
    traj_gt: PoseTrajectory3D,
    traj_est: PoseTrajectory3D,
    output_dir: str,
    prefix: str = ""
) -> None:
    """
    Save ATE and RPE visualization plots.

    Args:
        traj_gt: Ground truth trajectory
        traj_est: Estimated trajectory
        output_dir: Directory to save plots
        prefix: Optional prefix for filenames
    """
    try:
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend for headless operation
        import matplotlib.pyplot as plt
        from evo.tools import plot
        from evo.tools.settings import SETTINGS

        # Disable interactive mode
        plt.ioff()
        SETTINGS.plot_backend = 'Agg'
    except ImportError as e:
        print(f"Warning: Could not import matplotlib for plotting: {e}", file=sys.stderr)
        return

    # Associate and align for plotting
    max_diff = 0.05
    try:
        traj_gt_sync, traj_est_sync = sync.associate_trajectories(
            traj_gt, traj_est, max_diff=max_diff
        )
        traj_est_aligned = copy.deepcopy(traj_est_sync)
        traj_est_aligned.align(traj_gt_sync, correct_scale=False)
    except Exception as e:
        print(f"Warning: Could not align trajectories for plotting: {e}", file=sys.stderr)
        return

    # ATE plot - trajectory comparison
    try:
        fig, ax = plt.subplots(figsize=(10, 8))

        # Plot ground truth trajectory
        gt_xyz = traj_gt_sync.positions_xyz
        ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], 'b-', linewidth=2, label='Ground Truth')

        # Plot estimated trajectory (aligned)
        est_xyz = traj_est_aligned.positions_xyz
        ax.plot(est_xyz[:, 0], est_xyz[:, 1], 'r--', linewidth=2, label='Estimated (aligned)')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Comparison')
        ax.legend()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        ate_plot_path = os.path.join(output_dir, f"{prefix}ate_plot.png")
        fig.savefig(ate_plot_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
    except Exception as e:
        print(f"Warning: Failed to save ATE plot: {e}", file=sys.stderr)

    # RPE distribution plot
    try:
        rpe_metric = metrics.RPE(
            PoseRelation.translation_part,
            delta=1,
            delta_unit=Unit.frames,
            all_pairs=False
        )
        rpe_metric.process_data((traj_gt_sync, traj_est_aligned))

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.hist(rpe_metric.error, bins=50, edgecolor='black', alpha=0.7)
        ax.set_xlabel('RPE (m)')
        ax.set_ylabel('Count')
        ax.set_title('Relative Pose Error Distribution')
        ax.axvline(
            rpe_metric.get_statistic(metrics.StatisticsType.rmse),
            color='r', linestyle='--', label=f'RMSE: {rpe_metric.get_statistic(metrics.StatisticsType.rmse):.4f} m'
        )
        ax.legend()

        rpe_plot_path = os.path.join(output_dir, f"{prefix}rpe_plot.png")
        fig.savefig(rpe_plot_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
    except Exception as e:
        print(f"Warning: Failed to save RPE plot: {e}", file=sys.stderr)


def create_failure_result(
    dataset: str,
    algorithm: str,
    error_message: str
) -> Dict[str, Any]:
    """
    Create a failure result JSON structure.

    Args:
        dataset: Dataset name
        algorithm: Algorithm name
        error_message: Description of the failure

    Returns:
        Dictionary with failure information
    """
    return {
        "success": False,
        "dataset": dataset,
        "algorithm": algorithm,
        "timestamp": datetime.now().isoformat(),
        "ate": None,
        "rpe": None,
        "trajectory_length_m": None,
        "duration_s": None,
        "num_poses": None,
        "error": error_message,
    }


def create_success_result(
    dataset: str,
    algorithm: str,
    metrics_data: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Create a success result JSON structure.

    Args:
        dataset: Dataset name
        algorithm: Algorithm name
        metrics_data: Computed metrics

    Returns:
        Dictionary with success information and metrics
    """
    return {
        "success": True,
        "dataset": dataset,
        "algorithm": algorithm,
        "timestamp": datetime.now().isoformat(),
        "ate": metrics_data["ate"],
        "rpe": metrics_data["rpe"],
        "trajectory_length_m": metrics_data["trajectory_length_m"],
        "duration_s": metrics_data["duration_s"],
        "num_poses": metrics_data["num_poses"],
        "error": None,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate SLAM trajectory against ground truth using evo metrics."
    )
    parser.add_argument(
        "--gt_file",
        type=str,
        required=True,
        help="Path to ground truth trajectory file (TUM format)"
    )
    parser.add_argument(
        "--est_file",
        type=str,
        required=True,
        help="Path to estimated trajectory file (TUM format)"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        required=True,
        help="Directory to save metrics.json and plots"
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default="unknown",
        help="Dataset name for metadata"
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        default="unknown",
        help="Algorithm name for metadata"
    )
    parser.add_argument(
        "--save_plots",
        action="store_true",
        help="Save ATE and RPE visualization plots"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print detailed output"
    )

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    metrics_path = os.path.join(args.output_dir, "metrics.json")

    if args.verbose:
        print(f"Ground truth file: {args.gt_file}")
        print(f"Estimated file: {args.est_file}")
        print(f"Output directory: {args.output_dir}")

    # Load ground truth trajectory
    traj_gt = load_trajectory(args.gt_file)
    if traj_gt is None:
        error_msg = f"Failed to load ground truth trajectory: {args.gt_file}"
        print(f"Error: {error_msg}", file=sys.stderr)
        result = create_failure_result(args.dataset, args.algorithm, error_msg)
        with open(metrics_path, 'w') as f:
            json.dump(result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"Ground truth: {len(traj_gt.timestamps)} poses")

    # Load estimated trajectory
    traj_est = load_trajectory(args.est_file)
    if traj_est is None:
        error_msg = f"Failed to load estimated trajectory: {args.est_file}"
        print(f"Error: {error_msg}", file=sys.stderr)
        result = create_failure_result(args.dataset, args.algorithm, error_msg)
        with open(metrics_path, 'w') as f:
            json.dump(result, f, indent=2)
        sys.exit(1)

    if args.verbose:
        print(f"Estimated: {len(traj_est.timestamps)} poses")

    # Check for empty trajectories
    if len(traj_gt.timestamps) < 2:
        error_msg = f"Ground truth trajectory too short: {len(traj_gt.timestamps)} poses (need >= 2)"
        print(f"Error: {error_msg}", file=sys.stderr)
        result = create_failure_result(args.dataset, args.algorithm, error_msg)
        with open(metrics_path, 'w') as f:
            json.dump(result, f, indent=2)
        sys.exit(1)

    if len(traj_est.timestamps) < 2:
        error_msg = f"Estimated trajectory too short: {len(traj_est.timestamps)} poses (need >= 2)"
        print(f"Error: {error_msg}", file=sys.stderr)
        result = create_failure_result(args.dataset, args.algorithm, error_msg)
        with open(metrics_path, 'w') as f:
            json.dump(result, f, indent=2)
        sys.exit(1)

    # Compute metrics
    try:
        metrics_data = compute_metrics(traj_gt, traj_est)
        result = create_success_result(args.dataset, args.algorithm, metrics_data)

        if args.verbose:
            print(f"\nResults:")
            print(f"  ATE RMSE: {metrics_data['ate']['rmse']:.4f} m")
            print(f"  ATE Mean: {metrics_data['ate']['mean']:.4f} m")
            print(f"  RPE RMSE: {metrics_data['rpe']['rmse']:.4f} m")
            print(f"  RPE Mean: {metrics_data['rpe']['mean']:.4f} m")
            print(f"  Trajectory length: {metrics_data['trajectory_length_m']:.2f} m")
            print(f"  Duration: {metrics_data['duration_s']:.2f} s")
            print(f"  Synchronized poses: {metrics_data['num_poses']}")

    except Exception as e:
        error_msg = f"Metric computation failed: {e}"
        print(f"Error: {error_msg}", file=sys.stderr)
        result = create_failure_result(args.dataset, args.algorithm, error_msg)
        with open(metrics_path, 'w') as f:
            json.dump(result, f, indent=2)
        sys.exit(1)

    # Save metrics.json
    with open(metrics_path, 'w') as f:
        json.dump(result, f, indent=2)

    if args.verbose:
        print(f"\nMetrics saved to: {metrics_path}")

    # Optionally save plots
    if args.save_plots:
        if args.verbose:
            print("Generating plots...")
        save_plots(traj_gt, traj_est, args.output_dir)
        if args.verbose:
            print(f"Plots saved to: {args.output_dir}")

    print(f"Evaluation complete: ATE RMSE = {metrics_data['ate']['rmse']:.4f} m")
    sys.exit(0)


if __name__ == "__main__":
    main()
