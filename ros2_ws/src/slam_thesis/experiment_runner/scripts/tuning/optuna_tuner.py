#!/usr/bin/env python3
"""
Multi-fidelity Bayesian Optimization for SLAM Algorithm Tuning.

Uses Optuna with TPE sampler and MedianPruner for efficient hyperparameter
optimization. Implements multi-fidelity approach:
  - Phase 1: Quick trials on micro-trajectory (40 trials, ~20s each)
  - Phase 2: Refinement on traj_01_easy with top configs (10 trials)
  - Phase 3: Validation on traj_01_easy with top 2 configs (3x each)

Usage:
    python3 optuna_tuner.py --algorithm slam_toolbox --n_trials 40
    python3 optuna_tuner.py --algorithm cartographer --n_trials 40 --parallel 2
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

try:
    import optuna
    from optuna.pruners import MedianPruner
    from optuna.samplers import TPESampler
except ImportError:
    print("ERROR: Optuna not installed. Run: pip install optuna")
    sys.exit(1)

from config_generators import (
    get_param_space,
    generate_config,
    SLAM_TOOLBOX_PARAM_SPACE,
    CARTOGRAPHER_PARAM_SPACE,
)

# =============================================================================
# CONSTANTS
# =============================================================================

THESIS_DIR = Path.home() / 'thesis'
ROS2_WS = THESIS_DIR / 'ros2_ws'
TRAJECTORIES_DIR = THESIS_DIR / 'trajectories'
RESULTS_BASE = ROS2_WS / 'results' / 'tuning'

# Trajectory configs for multi-fidelity
MICRO_TRAJECTORY = 'traj_micro_tune'  # ~4m, ~15s
TUNING_TRAJECTORY = 'traj_01_easy'    # ~16m, ~60s

# Timeouts (seconds)
MICRO_TIMEOUT = 60      # 1 min for micro trajectory
TUNING_TIMEOUT = 180    # 3 min for tuning trajectory

# Early termination threshold
EARLY_TERM_ATE_THRESHOLD = 0.5  # If ATE > 50cm at 50% progress, prune

# Config file locations (will be overwritten during tuning)
SLAM_TOOLBOX_CONFIG = ROS2_WS / 'src/slam_thesis/slam_launch/config/slam_toolbox.yaml'
CARTOGRAPHER_CONFIG = ROS2_WS / 'src/slam_thesis/slam_launch/config/cartographer_2d.lua'


# =============================================================================
# EXPERIMENT RUNNER
# =============================================================================

def run_experiment(
    algorithm: str,
    trajectory: str,
    output_dir: Path,
    timeout: float,
    config_path: Optional[Path] = None
) -> dict:
    """
    Run a single SLAM experiment and return metrics.

    Args:
        algorithm: 'slam_toolbox' or 'cartographer'
        trajectory: Trajectory name (without .csv)
        output_dir: Directory for output files
        timeout: Maximum time to wait
        config_path: Optional custom config file path

    Returns:
        dict with 'ate_rmse', 'rpe_rmse', 'completion_rate', 'status'
    """
    # Build command
    run_one_script = ROS2_WS / 'src/slam_thesis/experiment_runner/scripts/run_one.py'

    cmd = [
        'python3', str(run_one_script),
        '--algo', algorithm,
        '--trajectory', trajectory,
        '--output_dir', str(output_dir),
        '--timeout', str(timeout),
        '--pose_mode', 'slam',
    ]

    # Run experiment
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 120  # Extra buffer for startup/cleanup
        )
    except subprocess.TimeoutExpired:
        return {
            'status': 'timeout',
            'ate_rmse': float('inf'),
            'rpe_rmse': float('inf'),
            'completion_rate': 0.0,
        }

    # Parse results
    metrics_file = output_dir / 'metrics.json'
    if not metrics_file.exists():
        return {
            'status': 'failed',
            'ate_rmse': float('inf'),
            'rpe_rmse': float('inf'),
            'completion_rate': 0.0,
        }

    with open(metrics_file, 'r') as f:
        metrics = json.load(f)

    # Handle None values safely (metrics['ate'] might be None, not missing)
    ate_dict = metrics.get('ate') or {}
    rpe_dict = metrics.get('rpe') or {}
    completion_dict = metrics.get('completion') or {}

    return {
        'status': metrics.get('status', 'unknown'),
        'ate_rmse': ate_dict.get('rmse', float('inf')) if ate_dict else float('inf'),
        'rpe_rmse': rpe_dict.get('rmse', float('inf')) if rpe_dict else float('inf'),
        'completion_rate': completion_dict.get('completion_rate', 0.0) if completion_dict else 0.0,
    }


# =============================================================================
# OPTUNA OBJECTIVE FUNCTION
# =============================================================================

def create_objective(
    algorithm: str,
    trajectory: str,
    timeout: float,
    results_dir: Path,
    config_backup_dir: Path,
) -> callable:
    """
    Create an Optuna objective function for the given algorithm.

    Args:
        algorithm: 'slam_toolbox' or 'cartographer'
        trajectory: Trajectory to use for evaluation
        timeout: Experiment timeout
        results_dir: Base directory for trial outputs
        config_backup_dir: Directory to save trial configs

    Returns:
        Objective function for Optuna
    """
    param_space = get_param_space(algorithm)

    # Determine config path
    if algorithm == 'slam_toolbox':
        config_path = SLAM_TOOLBOX_CONFIG
    else:
        config_path = CARTOGRAPHER_CONFIG

    def objective(trial: optuna.Trial) -> float:
        """Optuna objective function - returns ATE RMSE to minimize."""
        trial_id = trial.number
        print(f"\n{'='*60}")
        print(f"Trial {trial_id}: Starting...")
        print(f"{'='*60}")

        # Sample parameters
        params = {}
        for name, spec in param_space.items():
            if spec['type'] == 'float':
                if spec.get('log', False):
                    params[name] = trial.suggest_float(name, spec['low'], spec['high'], log=True)
                else:
                    params[name] = trial.suggest_float(name, spec['low'], spec['high'])
            elif spec['type'] == 'int':
                params[name] = trial.suggest_int(name, spec['low'], spec['high'])
            elif spec['type'] == 'categorical':
                params[name] = trial.suggest_categorical(name, spec['choices'])

        # Print sampled parameters
        print(f"Trial {trial_id}: Parameters:")
        for k, v in params.items():
            print(f"  {k}: {v}")

        # Generate config file
        trial_config_path = config_backup_dir / f'trial_{trial_id}_{algorithm}_config'
        if algorithm == 'slam_toolbox':
            trial_config_path = trial_config_path.with_suffix('.yaml')
        else:
            trial_config_path = trial_config_path.with_suffix('.lua')

        generate_config(algorithm, params, trial_config_path)

        # Copy to active config location
        shutil.copy(trial_config_path, config_path)
        print(f"Trial {trial_id}: Config written to {config_path}")

        # Run experiment
        trial_output_dir = results_dir / f'trial_{trial_id:03d}'
        trial_output_dir.mkdir(parents=True, exist_ok=True)

        start_time = time.time()
        result = run_experiment(
            algorithm=algorithm,
            trajectory=trajectory,
            output_dir=trial_output_dir,
            timeout=timeout,
        )
        elapsed = time.time() - start_time

        # Extract metrics
        ate_rmse = result['ate_rmse']
        rpe_rmse = result['rpe_rmse']
        completion_rate = result['completion_rate']
        status = result['status']

        print(f"Trial {trial_id}: Completed in {elapsed:.1f}s")
        print(f"  Status: {status}")
        print(f"  ATE RMSE: {ate_rmse:.4f} m ({ate_rmse*100:.2f} cm)")
        print(f"  RPE RMSE: {rpe_rmse:.4f} m")
        print(f"  Completion: {completion_rate*100:.1f}%")

        # Save trial metadata
        trial_meta = {
            'trial_id': trial_id,
            'algorithm': algorithm,
            'trajectory': trajectory,
            'params': params,
            'result': result,
            'elapsed_seconds': elapsed,
            'timestamp': datetime.now().isoformat(),
        }
        with open(trial_output_dir / 'trial_meta.json', 'w') as f:
            json.dump(trial_meta, f, indent=2)

        # Return objective value (ATE to minimize)
        # Penalize failed runs heavily
        if status == 'failed' or ate_rmse == float('inf'):
            return 10.0  # Large penalty

        # Penalize incomplete trajectories
        if completion_rate < 0.5:
            return ate_rmse + (1.0 - completion_rate)

        return ate_rmse

    return objective


# =============================================================================
# MULTI-FIDELITY TUNING
# =============================================================================

def run_multi_fidelity_tuning(
    algorithm: str,
    n_coarse_trials: int = 40,
    n_refine_trials: int = 10,
    n_validation_runs: int = 3,
    n_parallel: int = 1,
    seed: int = 42,
) -> dict:
    """
    Run multi-fidelity hyperparameter tuning.

    Phase 1: Coarse search on micro-trajectory
    Phase 2: Refinement on traj_01_easy with top configs
    Phase 3: Validation with multiple runs

    Args:
        algorithm: 'slam_toolbox' or 'cartographer'
        n_coarse_trials: Number of trials for coarse search
        n_refine_trials: Number of top configs to refine
        n_validation_runs: Runs per config for validation
        n_parallel: Number of parallel trials (careful with ROS!)
        seed: Random seed for reproducibility

    Returns:
        dict with best parameters and results
    """
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    study_name = f'{algorithm}_tuning_{timestamp}'

    # Create directories
    results_dir = RESULTS_BASE / study_name
    results_dir.mkdir(parents=True, exist_ok=True)

    config_backup_dir = results_dir / 'configs'
    config_backup_dir.mkdir(parents=True, exist_ok=True)

    # Backup original configs
    original_configs_dir = results_dir / 'original_configs'
    original_configs_dir.mkdir(parents=True, exist_ok=True)

    if SLAM_TOOLBOX_CONFIG.exists():
        shutil.copy(SLAM_TOOLBOX_CONFIG, original_configs_dir / 'slam_toolbox.yaml')
    if CARTOGRAPHER_CONFIG.exists():
        shutil.copy(CARTOGRAPHER_CONFIG, original_configs_dir / 'cartographer_2d.lua')

    print(f"\n{'='*70}")
    print(f"MULTI-FIDELITY HYPERPARAMETER TUNING")
    print(f"{'='*70}")
    print(f"Algorithm: {algorithm}")
    print(f"Study name: {study_name}")
    print(f"Results dir: {results_dir}")
    print(f"{'='*70}\n")

    # =========================================================================
    # PHASE 1: Coarse Search on Micro-Trajectory
    # =========================================================================
    print(f"\n{'#'*70}")
    print(f"# PHASE 1: Coarse Search ({n_coarse_trials} trials on {MICRO_TRAJECTORY})")
    print(f"{'#'*70}\n")

    phase1_dir = results_dir / 'phase1_coarse'
    phase1_dir.mkdir(parents=True, exist_ok=True)

    # Create Optuna study with TPE sampler
    sampler = TPESampler(seed=seed, n_startup_trials=5)
    pruner = MedianPruner(n_startup_trials=5, n_warmup_steps=0)

    study_phase1 = optuna.create_study(
        study_name=f'{study_name}_phase1',
        direction='minimize',
        sampler=sampler,
        pruner=pruner,
    )

    # Create objective for micro-trajectory
    objective_phase1 = create_objective(
        algorithm=algorithm,
        trajectory=MICRO_TRAJECTORY,
        timeout=MICRO_TIMEOUT,
        results_dir=phase1_dir,
        config_backup_dir=config_backup_dir,
    )

    # Run optimization
    study_phase1.optimize(
        objective_phase1,
        n_trials=n_coarse_trials,
        n_jobs=n_parallel,
        show_progress_bar=True,
    )

    # Get top trials
    top_trials = sorted(study_phase1.trials, key=lambda t: t.value if t.value else float('inf'))[:n_refine_trials]

    print(f"\nPhase 1 Complete!")
    print(f"Best trial: {study_phase1.best_trial.number} with ATE={study_phase1.best_value:.4f}m")
    print(f"Top {n_refine_trials} trials: {[t.number for t in top_trials]}")

    # Save phase 1 results
    phase1_results = {
        'best_trial': study_phase1.best_trial.number,
        'best_value': study_phase1.best_value,
        'best_params': study_phase1.best_params,
        'top_trials': [{'number': t.number, 'value': t.value, 'params': t.params} for t in top_trials],
    }
    with open(results_dir / 'phase1_results.json', 'w') as f:
        json.dump(phase1_results, f, indent=2)

    # =========================================================================
    # PHASE 2: Refinement on Tuning Trajectory
    # =========================================================================
    print(f"\n{'#'*70}")
    print(f"# PHASE 2: Refinement ({n_refine_trials} configs on {TUNING_TRAJECTORY})")
    print(f"{'#'*70}\n")

    phase2_dir = results_dir / 'phase2_refine'
    phase2_dir.mkdir(parents=True, exist_ok=True)

    phase2_results = []

    for i, trial in enumerate(top_trials):
        print(f"\nRefinement {i+1}/{n_refine_trials}: Testing trial {trial.number} params...")

        # Generate config from trial params
        params = trial.params
        config_path_trial = config_backup_dir / f'refine_{i}_{algorithm}_config'
        if algorithm == 'slam_toolbox':
            config_path_trial = config_path_trial.with_suffix('.yaml')
        else:
            config_path_trial = config_path_trial.with_suffix('.lua')

        generate_config(algorithm, params, config_path_trial)

        # Copy to active location
        if algorithm == 'slam_toolbox':
            shutil.copy(config_path_trial, SLAM_TOOLBOX_CONFIG)
        else:
            shutil.copy(config_path_trial, CARTOGRAPHER_CONFIG)

        # Run on tuning trajectory
        refine_output_dir = phase2_dir / f'config_{i:02d}_trial_{trial.number}'
        result = run_experiment(
            algorithm=algorithm,
            trajectory=TUNING_TRAJECTORY,
            output_dir=refine_output_dir,
            timeout=TUNING_TIMEOUT,
        )

        phase2_results.append({
            'rank': i,
            'original_trial': trial.number,
            'phase1_ate': trial.value,
            'phase2_ate': result['ate_rmse'],
            'phase2_rpe': result['rpe_rmse'],
            'phase2_completion': result['completion_rate'],
            'params': params,
        })

        print(f"  Phase 1 ATE: {trial.value:.4f}m -> Phase 2 ATE: {result['ate_rmse']:.4f}m")

    # Sort by phase 2 ATE
    phase2_results.sort(key=lambda x: x['phase2_ate'])

    # Save phase 2 results
    with open(results_dir / 'phase2_results.json', 'w') as f:
        json.dump(phase2_results, f, indent=2)

    print(f"\nPhase 2 Complete!")
    print(f"Best config: rank {phase2_results[0]['rank']} (trial {phase2_results[0]['original_trial']})")
    print(f"Phase 2 ATE: {phase2_results[0]['phase2_ate']:.4f}m ({phase2_results[0]['phase2_ate']*100:.2f}cm)")

    # =========================================================================
    # PHASE 3: Validation
    # =========================================================================
    print(f"\n{'#'*70}")
    print(f"# PHASE 3: Validation ({n_validation_runs} runs for top 2 configs)")
    print(f"{'#'*70}\n")

    phase3_dir = results_dir / 'phase3_validation'
    phase3_dir.mkdir(parents=True, exist_ok=True)

    validation_results = []
    top_2_configs = phase2_results[:2]

    for config in top_2_configs:
        config_ates = []
        config_rank = config['rank']
        params = config['params']

        print(f"\nValidating config {config_rank} (trial {config['original_trial']})...")

        # Generate and apply config
        config_path_val = config_backup_dir / f'validation_{config_rank}_{algorithm}_config'
        if algorithm == 'slam_toolbox':
            config_path_val = config_path_val.with_suffix('.yaml')
        else:
            config_path_val = config_path_val.with_suffix('.lua')

        generate_config(algorithm, params, config_path_val)

        if algorithm == 'slam_toolbox':
            shutil.copy(config_path_val, SLAM_TOOLBOX_CONFIG)
        else:
            shutil.copy(config_path_val, CARTOGRAPHER_CONFIG)

        for run in range(n_validation_runs):
            print(f"  Run {run+1}/{n_validation_runs}...")
            val_output_dir = phase3_dir / f'config_{config_rank}_run_{run}'

            result = run_experiment(
                algorithm=algorithm,
                trajectory=TUNING_TRAJECTORY,
                output_dir=val_output_dir,
                timeout=TUNING_TIMEOUT,
            )

            config_ates.append(result['ate_rmse'])
            print(f"    ATE: {result['ate_rmse']:.4f}m")

        # Calculate statistics
        import statistics
        mean_ate = statistics.mean(config_ates)
        std_ate = statistics.stdev(config_ates) if len(config_ates) > 1 else 0

        validation_results.append({
            'config_rank': config_rank,
            'original_trial': config['original_trial'],
            'params': params,
            'validation_ates': config_ates,
            'mean_ate': mean_ate,
            'std_ate': std_ate,
        })

        print(f"  Mean ATE: {mean_ate:.4f}m +/- {std_ate:.4f}m")

    # Sort by mean ATE
    validation_results.sort(key=lambda x: x['mean_ate'])

    # Save validation results
    with open(results_dir / 'phase3_validation_results.json', 'w') as f:
        json.dump(validation_results, f, indent=2)

    # =========================================================================
    # FINAL RESULTS
    # =========================================================================
    best_config = validation_results[0]

    print(f"\n{'='*70}")
    print(f"TUNING COMPLETE!")
    print(f"{'='*70}")
    print(f"\nBest configuration:")
    print(f"  Original trial: {best_config['original_trial']}")
    print(f"  Mean ATE: {best_config['mean_ate']:.4f}m ({best_config['mean_ate']*100:.2f}cm)")
    print(f"  Std ATE: {best_config['std_ate']:.4f}m")
    print(f"\nParameters:")
    for k, v in best_config['params'].items():
        print(f"  {k}: {v}")

    # Generate and save final best config
    final_config_path = results_dir / f'best_{algorithm}_config'
    if algorithm == 'slam_toolbox':
        final_config_path = final_config_path.with_suffix('.yaml')
    else:
        final_config_path = final_config_path.with_suffix('.lua')

    generate_config(algorithm, best_config['params'], final_config_path)
    print(f"\nBest config saved to: {final_config_path}")

    # Apply best config as the active one
    if algorithm == 'slam_toolbox':
        shutil.copy(final_config_path, SLAM_TOOLBOX_CONFIG)
    else:
        shutil.copy(final_config_path, CARTOGRAPHER_CONFIG)
    print(f"Best config applied to: {SLAM_TOOLBOX_CONFIG if algorithm == 'slam_toolbox' else CARTOGRAPHER_CONFIG}")

    # Save final summary
    final_summary = {
        'algorithm': algorithm,
        'study_name': study_name,
        'best_params': best_config['params'],
        'validation_mean_ate': best_config['mean_ate'],
        'validation_std_ate': best_config['std_ate'],
        'phase1_trials': n_coarse_trials,
        'phase2_configs': n_refine_trials,
        'phase3_runs_per_config': n_validation_runs,
        'timestamp': datetime.now().isoformat(),
    }
    with open(results_dir / 'final_summary.json', 'w') as f:
        json.dump(final_summary, f, indent=2)

    return final_summary


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Multi-fidelity Bayesian optimization for SLAM tuning',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Tune SLAM Toolbox with default settings
    python3 optuna_tuner.py --algorithm slam_toolbox

    # Tune Cartographer with more trials
    python3 optuna_tuner.py --algorithm cartographer --n_coarse 50

    # Quick test run
    python3 optuna_tuner.py --algorithm slam_toolbox --n_coarse 5 --n_refine 2 --n_validation 2
"""
    )

    parser.add_argument(
        '--algorithm', '-a',
        choices=['slam_toolbox', 'cartographer'],
        required=True,
        help='SLAM algorithm to tune'
    )

    parser.add_argument(
        '--n_coarse',
        type=int,
        default=40,
        help='Number of coarse search trials on micro-trajectory (default: 40)'
    )

    parser.add_argument(
        '--n_refine',
        type=int,
        default=10,
        help='Number of top configs to refine on full trajectory (default: 10)'
    )

    parser.add_argument(
        '--n_validation',
        type=int,
        default=3,
        help='Number of validation runs per top config (default: 3)'
    )

    parser.add_argument(
        '--parallel',
        type=int,
        default=1,
        help='Number of parallel trials (WARNING: may cause ROS conflicts, default: 1)'
    )

    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Random seed for reproducibility (default: 42)'
    )

    args = parser.parse_args()

    # Run tuning
    result = run_multi_fidelity_tuning(
        algorithm=args.algorithm,
        n_coarse_trials=args.n_coarse,
        n_refine_trials=args.n_refine,
        n_validation_runs=args.n_validation,
        n_parallel=args.parallel,
        seed=args.seed,
    )

    print(f"\n\nFinal result: {result['validation_mean_ate']*100:.2f}cm +/- {result['validation_std_ate']*100:.2f}cm")


if __name__ == '__main__':
    main()
