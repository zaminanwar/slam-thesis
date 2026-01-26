#!/bin/bash
# =============================================================================
# SLAM Hyperparameter Tuning Runner
# =============================================================================
#
# This script runs multi-fidelity Bayesian optimization for SLAM algorithms.
#
# Usage:
#   ./run_tuning.sh slam_toolbox    # Tune SLAM Toolbox (~60-90 min)
#   ./run_tuning.sh cartographer    # Tune Cartographer (~60-90 min)
#   ./run_tuning.sh both            # Tune both sequentially (~2-3 hours)
#   ./run_tuning.sh medium          # Medium run, good for thesis (~30-45 min at 5x speed)
#   ./run_tuning.sh test            # Quick test run (~15 min)
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/thesis/ros2_ws/install/setup.bash

# Set display for Gazebo (WSL2)
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Check for optuna
if ! python3 -c "import optuna" 2>/dev/null; then
    echo "ERROR: Optuna not installed. Installing..."
    pip install optuna
fi

# Parse arguments
MODE="${1:-help}"

case "$MODE" in
    slam_toolbox)
        echo "=============================================="
        echo "Tuning SLAM Toolbox"
        echo "Estimated time: 60-90 minutes"
        echo "=============================================="
        python3 optuna_tuner.py \
            --algorithm slam_toolbox \
            --n_coarse 40 \
            --n_refine 10 \
            --n_validation 3
        ;;

    cartographer)
        echo "=============================================="
        echo "Tuning Cartographer"
        echo "Estimated time: 60-90 minutes"
        echo "=============================================="
        python3 optuna_tuner.py \
            --algorithm cartographer \
            --n_coarse 40 \
            --n_refine 10 \
            --n_validation 3
        ;;

    both)
        echo "=============================================="
        echo "Tuning BOTH algorithms"
        echo "Estimated time: 2-3 hours"
        echo "=============================================="

        echo ""
        echo ">>> Starting SLAM Toolbox tuning..."
        python3 optuna_tuner.py \
            --algorithm slam_toolbox \
            --n_coarse 40 \
            --n_refine 10 \
            --n_validation 3

        echo ""
        echo ">>> Starting Cartographer tuning..."
        python3 optuna_tuner.py \
            --algorithm cartographer \
            --n_coarse 40 \
            --n_refine 10 \
            --n_validation 3

        echo ""
        echo "=============================================="
        echo "BOTH algorithms tuned successfully!"
        echo "=============================================="
        ;;

    medium)
        echo "=============================================="
        echo "MEDIUM tuning run (both algorithms)"
        echo "20 trials - good balance for thesis figures"
        echo "Estimated time: 30-45 minutes (at 5x sim speed)"
        echo "=============================================="

        echo ""
        echo ">>> Starting SLAM Toolbox tuning (medium)..."
        python3 optuna_tuner.py \
            --algorithm slam_toolbox \
            --n_coarse 20 \
            --n_refine 5 \
            --n_validation 3

        echo ""
        echo ">>> Starting Cartographer tuning (medium)..."
        python3 optuna_tuner.py \
            --algorithm cartographer \
            --n_coarse 20 \
            --n_refine 5 \
            --n_validation 3

        echo ""
        echo "=============================================="
        echo "MEDIUM tuning complete!"
        echo "=============================================="
        ;;

    test)
        echo "=============================================="
        echo "Quick TEST run (both algorithms)"
        echo "Estimated time: 15-20 minutes"
        echo "=============================================="

        echo ""
        echo ">>> Testing SLAM Toolbox tuning..."
        python3 optuna_tuner.py \
            --algorithm slam_toolbox \
            --n_coarse 5 \
            --n_refine 2 \
            --n_validation 2

        echo ""
        echo ">>> Testing Cartographer tuning..."
        python3 optuna_tuner.py \
            --algorithm cartographer \
            --n_coarse 5 \
            --n_refine 2 \
            --n_validation 2

        echo ""
        echo "Test complete!"
        ;;

    help|*)
        echo "SLAM Hyperparameter Tuning"
        echo ""
        echo "Usage: $0 <mode>"
        echo ""
        echo "Modes:"
        echo "  slam_toolbox  - Tune SLAM Toolbox (~60-90 min)"
        echo "  cartographer  - Tune Cartographer (~60-90 min)"
        echo "  both          - Tune both algorithms (~2-3 hours)"
        echo "  medium        - Medium run, thesis-quality (~30-45 min at 5x speed)"
        echo "  test          - Quick test run (~15 min)"
        echo "  help          - Show this help"
        echo ""
        echo "Example:"
        echo "  $0 slam_toolbox"
        echo ""
        echo "Output will be saved to:"
        echo "  ~/thesis/ros2_ws/results/tuning/<algorithm>_tuning_<timestamp>/"
        ;;
esac
