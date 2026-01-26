#!/usr/bin/env python3
"""
Visualize SLAM Hyperparameter Tuning Results.

Generates thesis-quality figures showing:
1. Optimization progression (ATE/RPE over trials)
2. Best-so-far convergence curve
3. Multi-fidelity phase transitions
4. Final validation results with error bars

Usage:
    python3 visualize_tuning.py                    # Auto-find latest results
    python3 visualize_tuning.py --output_dir figs  # Custom output directory
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# Style settings for thesis
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Colors
COLORS = {
    'slam_toolbox': '#2ecc71',      # Green
    'cartographer': '#3498db',       # Blue
    'best_so_far': '#e74c3c',        # Red
    'phase1': '#95a5a6',             # Gray
    'phase2': '#f39c12',             # Orange
    'phase3': '#9b59b6',             # Purple
}

RESULTS_BASE = Path.home() / 'thesis' / 'ros2_ws' / 'results' / 'tuning'


def find_latest_tuning_dirs() -> Dict[str, Path]:
    """Find the latest tuning result directories for each algorithm."""
    dirs = {}
    for algo in ['slam_toolbox', 'cartographer']:
        algo_dirs = sorted(RESULTS_BASE.glob(f'{algo}_tuning_*'))
        if algo_dirs:
            dirs[algo] = algo_dirs[-1]  # Latest
    return dirs


def load_phase1_trials(tuning_dir: Path) -> List[Dict]:
    """Load all Phase 1 trial data."""
    trials = []
    phase1_dir = tuning_dir / 'phase1_coarse'

    if not phase1_dir.exists():
        return trials

    for trial_dir in sorted(phase1_dir.glob('trial_*')):
        metrics_file = trial_dir / 'metrics.json'
        meta_file = trial_dir / 'trial_meta.json'

        if metrics_file.exists():
            with open(metrics_file) as f:
                metrics = json.load(f)

            trial_num = int(trial_dir.name.split('_')[1])

            # Handle potential None values
            ate_data = metrics.get('ate') or {}
            rpe_data = metrics.get('rpe') or {}

            trials.append({
                'trial': trial_num,
                'ate_rmse': ate_data.get('rmse', float('inf')),
                'rpe_rmse': rpe_data.get('rmse', float('inf')),
                'status': metrics.get('status', 'unknown'),
            })

    return sorted(trials, key=lambda x: x['trial'])


def load_phase2_results(tuning_dir: Path) -> List[Dict]:
    """Load Phase 2 refinement results."""
    results_file = tuning_dir / 'phase2_results.json'
    if results_file.exists():
        with open(results_file) as f:
            return json.load(f)
    return []


def load_phase3_results(tuning_dir: Path) -> List[Dict]:
    """Load Phase 3 validation results."""
    results_file = tuning_dir / 'phase3_validation_results.json'
    if results_file.exists():
        with open(results_file) as f:
            return json.load(f)
    return []


def load_final_summary(tuning_dir: Path) -> Dict:
    """Load final summary."""
    summary_file = tuning_dir / 'final_summary.json'
    if summary_file.exists():
        with open(summary_file) as f:
            return json.load(f)
    return {}


def compute_best_so_far(trials: List[Dict]) -> List[float]:
    """Compute cumulative best ATE at each trial."""
    best_so_far = []
    current_best = float('inf')
    for t in trials:
        if t['ate_rmse'] < current_best:
            current_best = t['ate_rmse']
        best_so_far.append(current_best)
    return best_so_far


def plot_optimization_progression(
    algo_data: Dict[str, Dict],
    output_dir: Path
):
    """
    Plot the full optimization progression for both algorithms.

    Shows: Phase 1 trials, best-so-far line, Phase 2/3 progression.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for idx, (algo, data) in enumerate(algo_data.items()):
        ax = axes[idx]

        phase1 = data['phase1']
        phase2 = data['phase2']
        phase3 = data['phase3']
        summary = data['summary']

        if not phase1:
            ax.text(0.5, 0.5, f'No data for {algo}', ha='center', va='center')
            continue

        # Extract trial data
        trials = [t['trial'] for t in phase1]
        ates = [t['ate_rmse'] * 100 for t in phase1]  # Convert to cm

        # Best so far
        best_so_far = [b * 100 for b in compute_best_so_far(phase1)]

        # Plot Phase 1 trials
        ax.scatter(trials, ates, c=COLORS['phase1'], s=80, alpha=0.7,
                   label='Phase 1 trials', zorder=2, edgecolors='white', linewidth=0.5)

        # Plot best-so-far line
        ax.plot(trials, best_so_far, c=COLORS['best_so_far'], linewidth=2.5,
                label='Best so far', zorder=3)

        # Mark the best trial
        best_idx = np.argmin(ates)
        ax.scatter([trials[best_idx]], [ates[best_idx]], c=COLORS['best_so_far'],
                   s=200, marker='*', zorder=4, edgecolors='black', linewidth=1,
                   label=f'Best: {ates[best_idx]:.2f} cm')

        # Add Phase 2/3 annotation
        if phase3:
            final_ate = summary.get('validation_mean_ate', 0) * 100
            final_std = summary.get('validation_std_ate', 0) * 100

            # Add text box with final result
            textstr = f'Final (validated):\n{final_ate:.2f} ± {final_std:.2f} cm'
            props = dict(boxstyle='round', facecolor=COLORS['phase3'], alpha=0.3)
            ax.text(0.97, 0.97, textstr, transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', horizontalalignment='right', bbox=props)

        # Labels and formatting
        algo_display = 'SLAM Toolbox' if algo == 'slam_toolbox' else 'Cartographer'
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('ATE RMSE (cm)')
        ax.set_title(f'{algo_display} - Bayesian Optimization')
        ax.legend(loc='upper right' if idx == 0 else 'upper left')

        # Set y-axis to start at 0
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=-0.5)

    plt.tight_layout()

    output_path = output_dir / 'optimization_progression.pdf'
    plt.savefig(output_path)
    plt.savefig(output_dir / 'optimization_progression.png')
    print(f"Saved: {output_path}")

    return fig


def plot_multi_fidelity_phases(
    algo_data: Dict[str, Dict],
    output_dir: Path
):
    """
    Plot the multi-fidelity progression: Phase 1 → Phase 2 → Phase 3.

    Shows how performance transfers across trajectory complexities.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for idx, (algo, data) in enumerate(algo_data.items()):
        ax = axes[idx]

        phase1 = data['phase1']
        phase2 = data['phase2']
        phase3 = data['phase3']

        if not phase1 or not phase2:
            ax.text(0.5, 0.5, f'Insufficient data for {algo}', ha='center', va='center')
            continue

        # Get best Phase 1 ATE
        best_phase1_ate = min(t['ate_rmse'] for t in phase1) * 100

        # Phase 2 results (sorted by phase2_ate)
        phase2_sorted = sorted(phase2, key=lambda x: x['phase2_ate'])

        # Prepare data for grouped bar chart
        labels = ['Phase 1\n(micro-traj)', 'Phase 2\n(traj_01_easy)']

        if phase3:
            labels.append('Phase 3\n(validated)')

        # Get values for best config
        best_p1 = phase2_sorted[0]['phase1_ate'] * 100
        best_p2 = phase2_sorted[0]['phase2_ate'] * 100

        values = [best_p1, best_p2]
        errors = [0, 0]  # No error bars for P1/P2

        if phase3:
            best_p3_mean = phase3[0]['mean_ate'] * 100
            best_p3_std = phase3[0]['std_ate'] * 100
            values.append(best_p3_mean)
            errors.append(best_p3_std)

        # Create bar chart
        x = np.arange(len(labels))
        colors = [COLORS['phase1'], COLORS['phase2']]
        if phase3:
            colors.append(COLORS['phase3'])

        bars = ax.bar(x, values, yerr=errors, capsize=5, color=colors,
                      edgecolor='black', linewidth=1, alpha=0.8)

        # Add value labels on bars
        for bar, val, err in zip(bars, values, errors):
            height = bar.get_height()
            label = f'{val:.2f}'
            if err > 0:
                label += f'\n±{err:.2f}'
            ax.annotate(label,
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=9)

        # Labels
        algo_display = 'SLAM Toolbox' if algo == 'slam_toolbox' else 'Cartographer'
        ax.set_ylabel('ATE RMSE (cm)')
        ax.set_title(f'{algo_display} - Multi-Fidelity Progression')
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_ylim(bottom=0)

    plt.tight_layout()

    output_path = output_dir / 'multi_fidelity_phases.pdf'
    plt.savefig(output_path)
    plt.savefig(output_dir / 'multi_fidelity_phases.png')
    print(f"Saved: {output_path}")

    return fig


def plot_combined_comparison(
    algo_data: Dict[str, Dict],
    output_dir: Path
):
    """
    Single figure comparing both algorithms side-by-side.

    Shows final tuned performance with validation error bars.
    """
    fig, ax = plt.subplots(figsize=(8, 5))

    algorithms = []
    means = []
    stds = []
    colors = []

    for algo, data in algo_data.items():
        summary = data['summary']
        if summary:
            algo_display = 'SLAM Toolbox' if algo == 'slam_toolbox' else 'Cartographer'
            algorithms.append(algo_display)
            means.append(summary.get('validation_mean_ate', 0) * 100)
            stds.append(summary.get('validation_std_ate', 0) * 100)
            colors.append(COLORS[algo])

    x = np.arange(len(algorithms))
    bars = ax.bar(x, means, yerr=stds, capsize=8, color=colors,
                  edgecolor='black', linewidth=1.5, alpha=0.85, width=0.5)

    # Add value labels
    for bar, mean, std in zip(bars, means, stds):
        height = bar.get_height()
        ax.annotate(f'{mean:.2f} ± {std:.2f} cm',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 5),
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=11, fontweight='bold')

    ax.set_ylabel('ATE RMSE (cm)')
    ax.set_title('Tuned SLAM Algorithm Comparison (Validated)')
    ax.set_xticks(x)
    ax.set_xticklabels(algorithms)
    ax.set_ylim(bottom=0, top=max(means) * 1.4)

    plt.tight_layout()

    output_path = output_dir / 'algorithm_comparison.pdf'
    plt.savefig(output_path)
    plt.savefig(output_dir / 'algorithm_comparison.png')
    print(f"Saved: {output_path}")

    return fig


def plot_ate_rpe_scatter(
    algo_data: Dict[str, Dict],
    output_dir: Path
):
    """
    Scatter plot of ATE vs RPE for all trials.

    Shows if optimizing ATE also improved RPE.
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    for algo, data in algo_data.items():
        phase1 = data['phase1']
        if not phase1:
            continue

        ates = [t['ate_rmse'] * 100 for t in phase1]
        rpes = [t['rpe_rmse'] * 100 for t in phase1]

        algo_display = 'SLAM Toolbox' if algo == 'slam_toolbox' else 'Cartographer'
        ax.scatter(ates, rpes, c=COLORS[algo], s=100, alpha=0.7,
                   label=algo_display, edgecolors='white', linewidth=0.5)

        # Mark best ATE trial
        best_idx = np.argmin(ates)
        ax.scatter([ates[best_idx]], [rpes[best_idx]], c=COLORS[algo],
                   s=250, marker='*', edgecolors='black', linewidth=1.5,
                   zorder=5)

    ax.set_xlabel('ATE RMSE (cm)')
    ax.set_ylabel('RPE RMSE (cm)')
    ax.set_title('ATE vs RPE Trade-off (★ = best ATE trial)')
    ax.legend()

    plt.tight_layout()

    output_path = output_dir / 'ate_rpe_scatter.pdf'
    plt.savefig(output_path)
    plt.savefig(output_dir / 'ate_rpe_scatter.png')
    print(f"Saved: {output_path}")

    return fig


def print_summary_table(algo_data: Dict[str, Dict]):
    """Print a summary table to console."""
    print("\n" + "="*70)
    print("TUNING RESULTS SUMMARY")
    print("="*70)

    for algo, data in algo_data.items():
        summary = data['summary']
        phase1 = data['phase1']

        algo_display = 'SLAM Toolbox' if algo == 'slam_toolbox' else 'Cartographer'
        print(f"\n{algo_display}:")
        print("-" * 40)

        if phase1:
            best_p1 = min(t['ate_rmse'] for t in phase1) * 100
            print(f"  Phase 1 best ATE:     {best_p1:.2f} cm")

        if summary:
            mean = summary.get('validation_mean_ate', 0) * 100
            std = summary.get('validation_std_ate', 0) * 100
            print(f"  Validated ATE:        {mean:.2f} ± {std:.2f} cm")
            print(f"  Phase 1 trials:       {summary.get('phase1_trials', 'N/A')}")
            print(f"  Phase 2 configs:      {summary.get('phase2_configs', 'N/A')}")
            print(f"  Phase 3 runs/config:  {summary.get('phase3_runs_per_config', 'N/A')}")

    print("\n" + "="*70)


def main():
    parser = argparse.ArgumentParser(description='Visualize SLAM tuning results')
    parser.add_argument('--output_dir', '-o', type=str, default=None,
                        help='Output directory for figures (default: results/tuning/figures)')
    parser.add_argument('--show', action='store_true',
                        help='Display plots interactively')
    args = parser.parse_args()

    # Find tuning directories
    tuning_dirs = find_latest_tuning_dirs()

    if not tuning_dirs:
        print("ERROR: No tuning results found in", RESULTS_BASE)
        return

    print(f"Found tuning results:")
    for algo, path in tuning_dirs.items():
        print(f"  {algo}: {path.name}")

    # Load all data
    algo_data = {}
    for algo, tuning_dir in tuning_dirs.items():
        algo_data[algo] = {
            'phase1': load_phase1_trials(tuning_dir),
            'phase2': load_phase2_results(tuning_dir),
            'phase3': load_phase3_results(tuning_dir),
            'summary': load_final_summary(tuning_dir),
        }

    # Create output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = RESULTS_BASE / 'figures'
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nOutput directory: {output_dir}")

    # Generate plots
    print("\nGenerating figures...")
    plot_optimization_progression(algo_data, output_dir)
    plot_multi_fidelity_phases(algo_data, output_dir)
    plot_combined_comparison(algo_data, output_dir)
    plot_ate_rpe_scatter(algo_data, output_dir)

    # Print summary
    print_summary_table(algo_data)

    if args.show:
        plt.show()

    print(f"\nDone! Figures saved to: {output_dir}")


if __name__ == '__main__':
    main()
