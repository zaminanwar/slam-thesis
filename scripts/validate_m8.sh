#!/bin/bash
# Milestone 8 Validation: Experiment Runner
# Validates run_one.py, run_all.py, and aggregate_results.py scripts

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"
EXPERIMENT_RUNNER_PKG="$WORKSPACE_DIR/src/slam_thesis/experiment_runner"
RESULTS_DIR="$WORKSPACE_DIR/results"

echo "=========================================="
echo "Milestone 8 Validation: Experiment Runner"
echo "=========================================="
echo ""

# Detect ROS distro
if [ -f "$SCRIPT_DIR/../.ros_distro" ]; then
    ROS_DISTRO=$(cat "$SCRIPT_DIR/../.ros_distro")
else
    ROS_DISTRO="jazzy"
fi
echo "ROS Distro: $ROS_DISTRO"

PASS_COUNT=0
FAIL_COUNT=0

pass() {
    echo "[PASS] $1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
    echo "[FAIL] $1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

warn() {
    echo "[WARN] $1"
}

# ============================================
# Test 1: Check run_one.py
# ============================================
echo ""
echo "=== Test 1: run_one.py Script ==="

RUN_ONE_SCRIPT="$EXPERIMENT_RUNNER_PKG/scripts/run_one.py"

if [ -f "$RUN_ONE_SCRIPT" ]; then
    pass "run_one.py exists"
else
    fail "run_one.py missing at $RUN_ONE_SCRIPT"
fi

if [ -x "$RUN_ONE_SCRIPT" ]; then
    pass "run_one.py is executable"
else
    fail "run_one.py is not executable"
fi

if python3 -m py_compile "$RUN_ONE_SCRIPT" 2>/dev/null; then
    pass "run_one.py is valid Python"
else
    fail "run_one.py has syntax errors"
fi

# Check for key functionality
if grep -q "slam_toolbox\|cartographer" "$RUN_ONE_SCRIPT"; then
    pass "run_one.py supports SLAM algorithms"
else
    fail "SLAM algorithm support not found"
fi

if grep -q "evaluate_run.py\|evaluate_run" "$RUN_ONE_SCRIPT"; then
    pass "run_one.py calls evaluation script"
else
    fail "Evaluation call not found"
fi

if grep -q "ProcessManager\|subprocess" "$RUN_ONE_SCRIPT"; then
    pass "run_one.py manages subprocesses"
else
    fail "Process management not found"
fi

if grep -q "signal\|SIGINT\|SIGTERM" "$RUN_ONE_SCRIPT"; then
    pass "run_one.py handles signals for cleanup"
else
    fail "Signal handling not found"
fi

if grep -q "trajectory_done\|wait_for" "$RUN_ONE_SCRIPT"; then
    pass "run_one.py waits for trajectory completion"
else
    fail "Trajectory completion wait not found"
fi

# Check CLI interface
if python3 "$RUN_ONE_SCRIPT" --help 2>&1 | grep -q "\-\-algo"; then
    pass "run_one.py has --algo argument"
else
    fail "run_one.py missing --algo argument"
fi

if python3 "$RUN_ONE_SCRIPT" --help 2>&1 | grep -q "\-\-trajectory"; then
    pass "run_one.py has --trajectory argument"
else
    fail "run_one.py missing --trajectory argument"
fi

if python3 "$RUN_ONE_SCRIPT" --help 2>&1 | grep -q "\-\-timeout"; then
    pass "run_one.py has --timeout argument"
else
    fail "run_one.py missing --timeout argument"
fi

# ============================================
# Test 2: Check run_all.py
# ============================================
echo ""
echo "=== Test 2: run_all.py Script ==="

RUN_ALL_SCRIPT="$EXPERIMENT_RUNNER_PKG/scripts/run_all.py"

if [ -f "$RUN_ALL_SCRIPT" ]; then
    pass "run_all.py exists"
else
    fail "run_all.py missing at $RUN_ALL_SCRIPT"
fi

if [ -x "$RUN_ALL_SCRIPT" ]; then
    pass "run_all.py is executable"
else
    fail "run_all.py is not executable"
fi

if python3 -m py_compile "$RUN_ALL_SCRIPT" 2>/dev/null; then
    pass "run_all.py is valid Python"
else
    fail "run_all.py has syntax errors"
fi

# Check for key functionality
if grep -q "discover_trajectories\|glob" "$RUN_ALL_SCRIPT"; then
    pass "run_all.py discovers trajectories"
else
    fail "Trajectory discovery not found"
fi

if grep -q "run_one.py\|run_single_experiment" "$RUN_ALL_SCRIPT"; then
    pass "run_all.py calls run_one.py"
else
    fail "run_one.py call not found"
fi

if grep -q "batch_summary\|summary" "$RUN_ALL_SCRIPT"; then
    pass "run_all.py generates batch summary"
else
    fail "Batch summary generation not found"
fi

if grep -q "dry.run\|dry_run" "$RUN_ALL_SCRIPT"; then
    pass "run_all.py supports dry-run mode"
else
    fail "Dry-run mode not found"
fi

# Check CLI interface
if python3 "$RUN_ALL_SCRIPT" --help 2>&1 | grep -q "\-\-algorithms"; then
    pass "run_all.py has --algorithms argument"
else
    fail "run_all.py missing --algorithms argument"
fi

if python3 "$RUN_ALL_SCRIPT" --help 2>&1 | grep -q "\-\-trajectories"; then
    pass "run_all.py has --trajectories argument"
else
    fail "run_all.py missing --trajectories argument"
fi

# Test dry-run mode
if python3 "$RUN_ALL_SCRIPT" --dry-run 2>&1 | grep -q "DRY RUN\|Would run"; then
    pass "run_all.py dry-run works"
else
    fail "run_all.py dry-run failed"
fi

# ============================================
# Test 3: Check aggregate_results.py
# ============================================
echo ""
echo "=== Test 3: aggregate_results.py Script ==="

AGGREGATE_SCRIPT="$EXPERIMENT_RUNNER_PKG/scripts/aggregate_results.py"

if [ -f "$AGGREGATE_SCRIPT" ]; then
    pass "aggregate_results.py exists"
else
    fail "aggregate_results.py missing at $AGGREGATE_SCRIPT"
fi

if [ -x "$AGGREGATE_SCRIPT" ]; then
    pass "aggregate_results.py is executable"
else
    fail "aggregate_results.py is not executable"
fi

if python3 -m py_compile "$AGGREGATE_SCRIPT" 2>/dev/null; then
    pass "aggregate_results.py is valid Python"
else
    fail "aggregate_results.py has syntax errors"
fi

# Check for key functionality
if grep -q "metrics.json" "$AGGREGATE_SCRIPT"; then
    pass "aggregate_results.py reads metrics.json"
else
    fail "metrics.json reading not found"
fi

if grep -q "csv\|CSV" "$AGGREGATE_SCRIPT"; then
    pass "aggregate_results.py outputs CSV"
else
    fail "CSV output not found"
fi

if grep -q "ate_rmse\|rpe_rmse" "$AGGREGATE_SCRIPT"; then
    pass "aggregate_results.py extracts key metrics"
else
    fail "Metric extraction not found"
fi

# Check CLI interface
if python3 "$AGGREGATE_SCRIPT" --help 2>&1 | grep -q "\-\-batch_dir\|\-\-results_dir"; then
    pass "aggregate_results.py has input directory argument"
else
    fail "aggregate_results.py missing input directory argument"
fi

if python3 "$AGGREGATE_SCRIPT" --help 2>&1 | grep -q "\-\-output"; then
    pass "aggregate_results.py has --output argument"
else
    fail "aggregate_results.py missing --output argument"
fi

# ============================================
# Test 4: Functional Test with Sample Data
# ============================================
echo ""
echo "=== Test 4: Aggregation Functional Test ==="

TEST_DIR="$RESULTS_DIR/m8_test_$$"
mkdir -p "$TEST_DIR/slam_toolbox_traj_01_easy"
mkdir -p "$TEST_DIR/cartographer_traj_01_easy"

# Create sample metrics files
cat > "$TEST_DIR/slam_toolbox_traj_01_easy/metrics.json" << 'EOF'
{
    "timestamp": "2026-01-25T12:00:00",
    "status": "success",
    "ate": {"rmse": 0.045, "mean": 0.040, "median": 0.042, "std": 0.015, "min": 0.010, "max": 0.080},
    "rpe": {"rmse": 0.022, "mean": 0.018, "median": 0.019, "std": 0.008, "min": 0.005, "max": 0.040, "delta_m": 1.0},
    "gt_poses": 150,
    "est_poses": 148,
    "errors": []
}
EOF

cat > "$TEST_DIR/cartographer_traj_01_easy/metrics.json" << 'EOF'
{
    "timestamp": "2026-01-25T12:05:00",
    "status": "success",
    "ate": {"rmse": 0.052, "mean": 0.048, "median": 0.050, "std": 0.018, "min": 0.012, "max": 0.095},
    "rpe": {"rmse": 0.028, "mean": 0.024, "median": 0.025, "std": 0.010, "min": 0.008, "max": 0.052, "delta_m": 1.0},
    "gt_poses": 150,
    "est_poses": 145,
    "errors": []
}
EOF

# Run aggregation
if python3 "$AGGREGATE_SCRIPT" --batch_dir "$TEST_DIR" --quiet 2>/dev/null; then
    pass "aggregate_results.py executed successfully"
else
    fail "aggregate_results.py execution failed"
fi

# Check output
if [ -f "$TEST_DIR/summary.csv" ]; then
    pass "summary.csv generated"
else
    fail "summary.csv not generated"
fi

# Check CSV content
if grep -q "slam_toolbox" "$TEST_DIR/summary.csv" 2>/dev/null; then
    pass "CSV contains slam_toolbox results"
else
    fail "CSV missing slam_toolbox results"
fi

if grep -q "cartographer" "$TEST_DIR/summary.csv" 2>/dev/null; then
    pass "CSV contains cartographer results"
else
    fail "CSV missing cartographer results"
fi

if grep -q "ate_rmse\|0.045" "$TEST_DIR/summary.csv" 2>/dev/null; then
    pass "CSV contains ATE RMSE values"
else
    fail "CSV missing ATE RMSE values"
fi

if grep -q "rpe_rmse\|0.022" "$TEST_DIR/summary.csv" 2>/dev/null; then
    pass "CSV contains RPE RMSE values"
else
    fail "CSV missing RPE RMSE values"
fi

# Clean up
rm -rf "$TEST_DIR"

# ============================================
# Test 5: Check trajectories exist
# ============================================
echo ""
echo "=== Test 5: Trajectory Files ==="

TRAJ_DIR="$SCRIPT_DIR/../trajectories"

if [ -d "$TRAJ_DIR" ]; then
    pass "Trajectories directory exists"
else
    fail "Trajectories directory missing"
fi

TRAJ_COUNT=$(ls -1 "$TRAJ_DIR"/*.csv 2>/dev/null | wc -l)
if [ "$TRAJ_COUNT" -gt 0 ]; then
    pass "Found $TRAJ_COUNT trajectory files"
else
    fail "No trajectory CSV files found"
fi

# Check specific trajectories
for traj in traj_01_easy traj_02_loop traj_03_complex; do
    if [ -f "$TRAJ_DIR/${traj}.csv" ]; then
        pass "$traj.csv exists"
    else
        warn "$traj.csv not found"
    fi
done

# ============================================
# Test 6: Check ROS launch files
# ============================================
echo ""
echo "=== Test 6: Required Launch Files ==="

SLAM_LAUNCH_PKG="$WORKSPACE_DIR/src/slam_thesis/slam_launch"

for launch_file in slam_toolbox_eval.launch.py cartographer_eval.launch.py; do
    if [ -f "$SLAM_LAUNCH_PKG/launch/$launch_file" ]; then
        pass "$launch_file exists"
    else
        fail "$launch_file missing"
    fi
done

# ============================================
# Summary
# ============================================
echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="
echo "Passed: $PASS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo "[SUCCESS] All M8 validation tests passed!"
    echo ""
    echo "Experiment Runner is ready. Usage:"
    echo ""
    echo "  # Run single experiment:"
    echo "  python3 $RUN_ONE_SCRIPT --algo slam_toolbox --trajectory traj_01_easy"
    echo ""
    echo "  # Run all experiments:"
    echo "  python3 $RUN_ALL_SCRIPT"
    echo ""
    echo "  # Dry-run (see what would be executed):"
    echo "  python3 $RUN_ALL_SCRIPT --dry-run"
    echo ""
    echo "  # Aggregate results:"
    echo "  python3 $AGGREGATE_SCRIPT --batch_dir ~/thesis/ros2_ws/results/batch_xxx"
    echo ""
    exit 0
else
    echo "[FAILURE] $FAIL_COUNT tests failed. Please fix issues above."
    exit 1
fi
