#!/usr/bin/env python3
"""
Generate thesis comparison plots from batch experiment results.

Reads batch_progress.json and individual nav_results.json files
to produce publication-ready figures.
"""

import json
import os
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = os.path.expanduser('~/thesis/ros2_ws/results')
PLOTS_DIR = os.path.expanduser('~/thesis/ros2_ws/results/plots')

GOAL_LABELS = {
    'nav_goals_01.yaml': 'Simple\n(4 goals, ~12m)',
    'nav_goals_02.yaml': 'Moderate\n(7 goals, ~15m)',
    'nav_goals_03.yaml': 'Complex\n(11 goals, ~35m)',
}

GOAL_ORDER = ['nav_goals_01.yaml', 'nav_goals_02.yaml', 'nav_goals_03.yaml']
GOAL_DISTANCES = {'nav_goals_01.yaml': 12, 'nav_goals_02.yaml': 15, 'nav_goals_03.yaml': 35}

ALGO_COLORS = {
    'slam_toolbox': '#2196F3',
    'cartographer': '#FF5722',
}

ALGO_LABELS = {
    'slam_toolbox': 'SLAM Toolbox',
    'cartographer': 'Cartographer',
}


def load_results():
    """Load batch results and group by (algorithm, goal_set)."""
    path = os.path.join(RESULTS_DIR, 'batch_progress.json')
    with open(path) as f:
        data = json.load(f)

    groups = defaultdict(list)
    for r in data:
        if not r['success']:
            continue
        m = r.get('metrics', {})
        ate = m.get('ate_rmse_m')
        rpe = m.get('rpe_rmse_m')
        t = m.get('total_time_s')
        sr = m.get('success_rate')
        groups[(r['algorithm'], r['goals_file'])].append({
            'ate_m': ate,
            'rpe_m': rpe,
            'time_s': t,
            'success_rate': sr,
            'result_dir': r.get('result_dir'),
        })
    return groups


def plot_ate_comparison(groups):
    """Bar chart: ATE RMSE comparison across goal sets."""
    fig, ax = plt.subplots(figsize=(10, 6))

    x = np.arange(len(GOAL_ORDER))
    width = 0.35

    for i, algo in enumerate(['slam_toolbox', 'cartographer']):
        means = []
        stds = []
        for goals in GOAL_ORDER:
            runs = groups.get((algo, goals), [])
            ates = [r['ate_m'] * 100 for r in runs if r['ate_m']]
            means.append(statistics.mean(ates) if ates else 0)
            stds.append(statistics.stdev(ates) if len(ates) > 1 else 0)

        offset = (i - 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds,
                      label=ALGO_LABELS[algo], color=ALGO_COLORS[algo],
                      alpha=0.85, capsize=5, edgecolor='white', linewidth=0.5)

        # Add value labels on bars
        for bar, m, s in zip(bars, means, stds):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + s + 0.3,
                    f'{m:.1f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_xlabel('Trajectory Complexity', fontsize=12)
    ax.set_ylabel('ATE RMSE (cm)', fontsize=12)
    ax.set_title('Absolute Trajectory Error: SLAM Toolbox vs Cartographer', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels([GOAL_LABELS[g] for g in GOAL_ORDER])
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3)
    ax.set_ylim(0, max(20, ax.get_ylim()[1] * 1.15))

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'ate_comparison.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_rpe_comparison(groups):
    """Bar chart: RPE RMSE comparison across goal sets."""
    fig, ax = plt.subplots(figsize=(10, 6))

    x = np.arange(len(GOAL_ORDER))
    width = 0.35

    for i, algo in enumerate(['slam_toolbox', 'cartographer']):
        means = []
        stds = []
        for goals in GOAL_ORDER:
            runs = groups.get((algo, goals), [])
            rpes = [r['rpe_m'] * 100 for r in runs if r['rpe_m']]
            means.append(statistics.mean(rpes) if rpes else 0)
            stds.append(statistics.stdev(rpes) if len(rpes) > 1 else 0)

        offset = (i - 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds,
                      label=ALGO_LABELS[algo], color=ALGO_COLORS[algo],
                      alpha=0.85, capsize=5, edgecolor='white', linewidth=0.5)

        for bar, m, s in zip(bars, means, stds):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + s + 0.02,
                    f'{m:.2f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_xlabel('Trajectory Complexity', fontsize=12)
    ax.set_ylabel('RPE RMSE (cm)', fontsize=12)
    ax.set_title('Relative Pose Error: SLAM Toolbox vs Cartographer', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels([GOAL_LABELS[g] for g in GOAL_ORDER])
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'rpe_comparison.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_comparison(groups):
    """Bar chart: Navigation time comparison."""
    fig, ax = plt.subplots(figsize=(10, 6))

    x = np.arange(len(GOAL_ORDER))
    width = 0.35

    for i, algo in enumerate(['slam_toolbox', 'cartographer']):
        means = []
        stds = []
        for goals in GOAL_ORDER:
            runs = groups.get((algo, goals), [])
            times = [r['time_s'] for r in runs if r['time_s']]
            means.append(statistics.mean(times) if times else 0)
            stds.append(statistics.stdev(times) if len(times) > 1 else 0)

        offset = (i - 0.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds,
                      label=ALGO_LABELS[algo], color=ALGO_COLORS[algo],
                      alpha=0.85, capsize=5, edgecolor='white', linewidth=0.5)

        for bar, m, s in zip(bars, means, stds):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + s + 1,
                    f'{m:.0f}s', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_xlabel('Trajectory Complexity', fontsize=12)
    ax.set_ylabel('Total Navigation Time (s)', fontsize=12)
    ax.set_title('Navigation Time: SLAM Toolbox vs Cartographer', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels([GOAL_LABELS[g] for g in GOAL_ORDER])
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'time_comparison.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_ate_scaling(groups):
    """Line plot: ATE vs trajectory complexity (path length)."""
    fig, ax = plt.subplots(figsize=(10, 6))

    for algo in ['slam_toolbox', 'cartographer']:
        distances = []
        means = []
        stds = []
        for goals in GOAL_ORDER:
            runs = groups.get((algo, goals), [])
            ates = [r['ate_m'] * 100 for r in runs if r['ate_m']]
            if ates:
                distances.append(GOAL_DISTANCES[goals])
                means.append(statistics.mean(ates))
                stds.append(statistics.stdev(ates) if len(ates) > 1 else 0)

        ax.errorbar(distances, means, yerr=stds,
                     label=ALGO_LABELS[algo], color=ALGO_COLORS[algo],
                     marker='o', markersize=10, linewidth=2.5, capsize=6)

        # Add value annotations
        for d, m, s in zip(distances, means, stds):
            ax.annotate(f'{m:.1f}cm', (d, m), textcoords='offset points',
                        xytext=(12, 8), fontsize=9, fontweight='bold',
                        color=ALGO_COLORS[algo])

    ax.set_xlabel('Approximate Trajectory Length (m)', fontsize=12)
    ax.set_ylabel('ATE RMSE (cm)', fontsize=12)
    ax.set_title('SLAM Accuracy Scaling with Trajectory Complexity', fontsize=14)
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_xlim(8, 40)
    ax.set_ylim(0, max(22, ax.get_ylim()[1] * 1.15))

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'ate_scaling.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_accuracy_ratio(groups):
    """Bar chart: ATE ratio (cartographer / slam_toolbox) per goal set."""
    fig, ax = plt.subplots(figsize=(8, 5))

    ratios = []
    labels = []
    for goals in GOAL_ORDER:
        st_runs = groups.get(('slam_toolbox', goals), [])
        cg_runs = groups.get(('cartographer', goals), [])
        st_ates = [r['ate_m'] * 100 for r in st_runs if r['ate_m']]
        cg_ates = [r['ate_m'] * 100 for r in cg_runs if r['ate_m']]
        if st_ates and cg_ates:
            ratio = statistics.mean(cg_ates) / statistics.mean(st_ates)
            ratios.append(ratio)
            labels.append(GOAL_LABELS[goals])

    bars = ax.bar(range(len(ratios)), ratios, color='#9C27B0', alpha=0.85,
                  edgecolor='white', linewidth=0.5)

    for bar, r in zip(bars, ratios):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                f'{r:.1f}x', ha='center', va='bottom', fontsize=12, fontweight='bold')

    ax.set_xlabel('Trajectory Complexity', fontsize=12)
    ax.set_ylabel('ATE Ratio (Cartographer / SLAM Toolbox)', fontsize=12)
    ax.set_title('How Much More Accurate is SLAM Toolbox?', fontsize=14)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels)
    ax.axhline(y=1, color='gray', linestyle='--', alpha=0.5, label='Equal performance')
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=10)

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'accuracy_ratio.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_per_run_scatter(groups):
    """Scatter plot: all individual ATE measurements."""
    fig, ax = plt.subplots(figsize=(10, 6))

    for algo in ['slam_toolbox', 'cartographer']:
        all_x = []
        all_y = []
        for goals in GOAL_ORDER:
            runs = groups.get((algo, goals), [])
            for r in runs:
                if r['ate_m']:
                    all_x.append(GOAL_DISTANCES[goals])
                    all_y.append(r['ate_m'] * 100)

        # Jitter x for visibility
        jitter = -0.3 if algo == 'slam_toolbox' else 0.3
        jittered_x = [x + jitter for x in all_x]

        ax.scatter(jittered_x, all_y, label=ALGO_LABELS[algo],
                   color=ALGO_COLORS[algo], s=80, alpha=0.7, edgecolors='white',
                   linewidth=0.5, zorder=5)

    ax.set_xlabel('Approximate Trajectory Length (m)', fontsize=12)
    ax.set_ylabel('ATE RMSE (cm)', fontsize=12)
    ax.set_title('Individual Run Results (All 18 Experiments)', fontsize=14)
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_xlim(8, 40)

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'individual_runs_scatter.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_box_plots(groups):
    """Box plots: ATE distribution per (algorithm x goal_set)."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=True)

    for col, goals in enumerate(GOAL_ORDER):
        ax = axes[col]
        data = []
        labels = []
        colors = []
        for algo in ['slam_toolbox', 'cartographer']:
            runs = groups.get((algo, goals), [])
            ates = [r['ate_m'] * 100 for r in runs if r['ate_m']]
            data.append(ates)
            labels.append(ALGO_LABELS[algo])
            colors.append(ALGO_COLORS[algo])

        bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.6)
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        # Overlay individual points
        for i, (d, color) in enumerate(zip(data, colors)):
            jitter = np.random.normal(0, 0.04, len(d))
            ax.scatter([i + 1 + j for j in jitter], d, color=color,
                       s=50, zorder=5, alpha=0.8, edgecolors='white', linewidth=0.5)

        ax.set_title(GOAL_LABELS[goals].replace('\n', ' '), fontsize=11)
        ax.grid(axis='y', alpha=0.3)

    axes[0].set_ylabel('ATE RMSE (cm)', fontsize=12)
    fig.suptitle('ATE Distribution Across Runs', fontsize=14, y=1.02)
    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, 'ate_boxplots.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def main():
    os.makedirs(PLOTS_DIR, exist_ok=True)
    groups = load_results()

    print(f'\nGenerating plots from {sum(len(v) for v in groups.values())} runs...\n')

    plot_ate_comparison(groups)
    plot_rpe_comparison(groups)
    plot_time_comparison(groups)
    plot_ate_scaling(groups)
    plot_accuracy_ratio(groups)
    plot_per_run_scatter(groups)
    plot_box_plots(groups)

    print(f'\nAll plots saved to: {PLOTS_DIR}')
    print(f'Open in File Explorer: \\\\wsl.localhost\\Ubuntu-24.04{PLOTS_DIR}')


if __name__ == '__main__':
    main()
