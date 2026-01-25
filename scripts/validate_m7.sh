#!/bin/bash
# Milestone 7 Validation: Evaluation (ATE/RPE Metrics)
# Validates the evaluate_run.py script for trajectory evaluation using evo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"
EXPERIMENT_RUNNER_PKG="$WORKSPACE_DIR/src/slam_thesis/experiment_runner"
RESULTS_DIR="$WORKSPACE_DIR/results"

echo "=========================================="
echo "Milestone 7 Validation: Evaluation (ATE/RPE)"
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

# Test 1: Check evo installation
echo ""
echo "=== Test 1: evo Installation ==="

if command -v evo_ape &> /dev/null || [ -f "$HOME/.local/bin/evo_ape" ]; then
    pass "evo_ape command available"
else
    fail "evo_ape not found (install with: pip install evo)"
fi

if command -v evo_rpe &> /dev/null || [ -f "$HOME/.local/bin/evo_rpe" ]; then
    pass "evo_rpe command available"
else
    fail "evo_rpe not found (install with: pip install evo)"
fi

# Test 2: Check evaluate_run.py exists and is valid
echo ""
echo "=== Test 2: evaluate_run.py Script ==="

EVALUATE_SCRIPT="$EXPERIMENT_RUNNER_PKG/scripts/evaluate_run.py"

if [ -f "$EVALUATE_SCRIPT" ]; then
    pass "evaluate_run.py exists"
else
    fail "evaluate_run.py missing at $EVALUATE_SCRIPT"
fi

if [ -x "$EVALUATE_SCRIPT" ]; then
    pass "evaluate_run.py is executable"
else
    fail "evaluate_run.py is not executable (run: chmod +x $EVALUATE_SCRIPT)"
fi

if python3 -m py_compile "$EVALUATE_SCRIPT" 2>/dev/null; then
    pass "evaluate_run.py is valid Python"
else
    fail "evaluate_run.py has syntax errors"
fi

# Test 3: Check script functionality
echo ""
echo "=== Test 3: Script Functionality ==="

# Check for key functionality in the script
if grep -q "evo_ape" "$EVALUATE_SCRIPT"; then
    pass "Uses evo_ape for ATE computation"
else
    fail "evo_ape usage not found"
fi

if grep -q "evo_rpe" "$EVALUATE_SCRIPT"; then
    pass "Uses evo_rpe for RPE computation"
else
    fail "evo_rpe usage not found"
fi

if grep -q "metrics.json\|save_results" "$EVALUATE_SCRIPT"; then
    pass "Saves results to JSON"
else
    fail "JSON output not implemented"
fi

if grep -q "error\|Error\|failed\|Failed" "$EVALUATE_SCRIPT"; then
    pass "Error handling implemented"
else
    fail "Error handling not found"
fi

if grep -q "rmse\|RMSE" "$EVALUATE_SCRIPT"; then
    pass "RMSE metric extraction implemented"
else
    fail "RMSE metric extraction not found"
fi

# Test 4: Create test data and run evaluation
echo ""
echo "=== Test 4: Functional Test with Sample Data ==="

TEST_DIR="$RESULTS_DIR/m7_test_$$"
mkdir -p "$TEST_DIR"

# Create sample ground truth trajectory (L-shaped path)
cat > "$TEST_DIR/gt.tum" << 'EOF'
# Ground truth trajectory (TUM format)
# timestamp tx ty tz qx qy qz qw
0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0
0.1 0.1 0.0 0.0 0.0 0.0 0.0 1.0
0.2 0.2 0.0 0.0 0.0 0.0 0.0 1.0
0.3 0.3 0.0 0.0 0.0 0.0 0.0 1.0
0.4 0.4 0.0 0.0 0.0 0.0 0.0 1.0
0.5 0.5 0.0 0.0 0.0 0.0 0.0 1.0
0.6 0.6 0.0 0.0 0.0 0.0 0.0 1.0
0.7 0.7 0.0 0.0 0.0 0.0 0.0 1.0
0.8 0.8 0.0 0.0 0.0 0.0 0.0 1.0
0.9 0.9 0.0 0.0 0.0 0.0 0.0 1.0
1.0 1.0 0.0 0.0 0.0 0.0 0.0 1.0
1.1 1.0 0.1 0.0 0.0 0.0 0.0 1.0
1.2 1.0 0.2 0.0 0.0 0.0 0.0 1.0
1.3 1.0 0.3 0.0 0.0 0.0 0.0 1.0
1.4 1.0 0.4 0.0 0.0 0.0 0.0 1.0
1.5 1.0 0.5 0.0 0.0 0.0 0.0 1.0
1.6 1.0 0.6 0.0 0.0 0.0 0.0 1.0
1.7 1.0 0.7 0.0 0.0 0.0 0.0 1.0
1.8 1.0 0.8 0.0 0.0 0.0 0.0 1.0
1.9 1.0 0.9 0.0 0.0 0.0 0.0 1.0
2.0 1.0 1.0 0.0 0.0 0.0 0.0 1.0
EOF

# Create sample estimate (with small errors)
cat > "$TEST_DIR/est.tum" << 'EOF'
# SLAM estimate trajectory (TUM format)
# timestamp tx ty tz qx qy qz qw
0.0 0.01 0.01 0.0 0.0 0.0 0.0 1.0
0.1 0.11 0.02 0.0 0.0 0.0 0.0 1.0
0.2 0.19 -0.01 0.0 0.0 0.0 0.0 1.0
0.3 0.31 0.01 0.0 0.0 0.0 0.0 1.0
0.4 0.39 0.02 0.0 0.0 0.0 0.0 1.0
0.5 0.51 -0.01 0.0 0.0 0.0 0.0 1.0
0.6 0.59 0.01 0.0 0.0 0.0 0.0 1.0
0.7 0.71 0.02 0.0 0.0 0.0 0.0 1.0
0.8 0.79 -0.01 0.0 0.0 0.0 0.0 1.0
0.9 0.91 0.01 0.0 0.0 0.0 0.0 1.0
1.0 0.99 0.02 0.0 0.0 0.0 0.0 1.0
1.1 1.01 0.11 0.0 0.0 0.0 0.0 1.0
1.2 0.99 0.19 0.0 0.0 0.0 0.0 1.0
1.3 1.01 0.31 0.0 0.0 0.0 0.0 1.0
1.4 0.99 0.39 0.0 0.0 0.0 0.0 1.0
1.5 1.01 0.51 0.0 0.0 0.0 0.0 1.0
1.6 0.99 0.59 0.0 0.0 0.0 0.0 1.0
1.7 1.01 0.71 0.0 0.0 0.0 0.0 1.0
1.8 0.99 0.79 0.0 0.0 0.0 0.0 1.0
1.9 1.01 0.91 0.0 0.0 0.0 0.0 1.0
2.0 0.99 0.99 0.0 0.0 0.0 0.0 1.0
EOF

# Run evaluation
if python3 "$EVALUATE_SCRIPT" --run_dir "$TEST_DIR" 2>/dev/null; then
    pass "evaluate_run.py executed successfully"
else
    fail "evaluate_run.py execution failed"
fi

# Check output file
if [ -f "$TEST_DIR/metrics.json" ]; then
    pass "metrics.json generated"
else
    fail "metrics.json not generated"
fi

# Check output content
if grep -q '"status": "success"' "$TEST_DIR/metrics.json" 2>/dev/null; then
    pass "Evaluation status is success"
elif grep -q '"status": "partial"' "$TEST_DIR/metrics.json" 2>/dev/null; then
    warn "Evaluation status is partial (some metrics may have failed)"
    pass "Evaluation produced results"
else
    fail "Evaluation failed or produced no results"
fi

if grep -q '"ate":' "$TEST_DIR/metrics.json" 2>/dev/null; then
    pass "ATE metrics present in output"
else
    fail "ATE metrics missing from output"
fi

if grep -q '"rpe":' "$TEST_DIR/metrics.json" 2>/dev/null; then
    pass "RPE metrics present in output"
else
    fail "RPE metrics missing from output"
fi

if grep -q '"rmse":' "$TEST_DIR/metrics.json" 2>/dev/null; then
    pass "RMSE values present in output"
else
    fail "RMSE values missing from output"
fi

# Check ATE RMSE is reasonable (should be small for our test data)
ATE_RMSE=$(python3 -c "import json; d=json.load(open('$TEST_DIR/metrics.json')); print(d.get('ate',{}).get('rmse',999))" 2>/dev/null || echo "999")
if python3 -c "exit(0 if float($ATE_RMSE) < 0.1 else 1)" 2>/dev/null; then
    pass "ATE RMSE is reasonable (< 0.1m): $ATE_RMSE m"
else
    fail "ATE RMSE is unexpectedly large: $ATE_RMSE m"
fi

# Test 5: Test error handling
echo ""
echo "=== Test 5: Error Handling ==="

# Test with missing files
BAD_DIR="$RESULTS_DIR/m7_test_bad_$$"
mkdir -p "$BAD_DIR"

if python3 "$EVALUATE_SCRIPT" --run_dir "$BAD_DIR" 2>/dev/null; then
    # Script should still run but report failure
    if grep -q '"status": "failed"' "$BAD_DIR/metrics.json" 2>/dev/null; then
        pass "Correctly reports failure for missing files"
    else
        fail "Should report failure status for missing files"
    fi
else
    # Non-zero exit code is also acceptable for failures
    pass "Returns non-zero exit code for missing files"
fi

if grep -q '"errors":' "$BAD_DIR/metrics.json" 2>/dev/null; then
    pass "Error messages included in output"
else
    fail "Error messages should be included in output"
fi

# Clean up test directories
rm -rf "$TEST_DIR" "$BAD_DIR"

# Test 6: Check CLI arguments
echo ""
echo "=== Test 6: CLI Interface ==="

if python3 "$EVALUATE_SCRIPT" --help 2>&1 | grep -q "run_dir\|--gt\|--est"; then
    pass "CLI help shows expected arguments"
else
    fail "CLI help missing expected arguments"
fi

if python3 "$EVALUATE_SCRIPT" --help 2>&1 | grep -q "metrics"; then
    pass "CLI help mentions metrics output"
else
    fail "CLI help should mention metrics output"
fi

# Summary
echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="
echo "Passed: $PASS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo "[SUCCESS] All M7 validation tests passed!"
    echo ""
    echo "Evaluation script is ready. Usage:"
    echo ""
    echo "  # After running SLAM and generating trajectory files:"
    echo "  python3 $EVALUATE_SCRIPT --run_dir ~/thesis/ros2_ws/results/run1"
    echo ""
    echo "  # Or with explicit paths:"
    echo "  python3 $EVALUATE_SCRIPT --gt gt.tum --est est.tum --output metrics.json"
    echo ""
    echo "  # Output format (metrics.json):"
    echo "  {"
    echo '    "status": "success",'
    echo '    "ate": {"rmse": 0.05, "mean": 0.04, ...},'
    echo '    "rpe": {"rmse": 0.02, "mean": 0.015, ...}'
    echo "  }"
    echo ""
    exit 0
else
    echo "[FAILURE] $FAIL_COUNT tests failed. Please fix issues above."
    exit 1
fi
