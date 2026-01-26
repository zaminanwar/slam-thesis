#!/bin/bash
#
# M7 Validation Script - Evaluation
#
# Validates:
# 1. evaluate_run.py script exists and is executable
# 2. evo library is installed
# 3. evaluate_run.py help works
# 4. evaluate_run.py can compute metrics on test data
# 5. metrics.json output format is correct
# 6. Failure handling works correctly
# 7. Plot generation works
#
# This is the per-run evaluation script using evo library.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$THESIS_DIR/ros2_ws"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M7 Validation - Evaluation"
echo "=========================================="
echo ""

# Track failures
FAILURES=0

# Helper functions
check_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

check_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAILURES++))
}

check_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Create temp directory for tests
TEST_DIR=$(mktemp -d)
trap "rm -rf $TEST_DIR" EXIT

# 1. Check evaluate_run.py exists and is executable
echo "Checking evaluate_run.py script..."

EVAL_SCRIPT="$WS_DIR/src/slam_thesis/experiment_runner/scripts/evaluate_run.py"

if [[ -f "$EVAL_SCRIPT" ]]; then
    check_pass "evaluate_run.py exists"
else
    check_fail "evaluate_run.py missing: $EVAL_SCRIPT"
fi

if [[ -x "$EVAL_SCRIPT" ]]; then
    check_pass "evaluate_run.py is executable"
else
    check_warn "evaluate_run.py is not executable (will try with python3)"
fi

echo ""

# 2. Check evo library is installed
echo "Checking evo library..."

if python3 -c "import evo" 2>/dev/null; then
    check_pass "evo library is installed"
    EVO_VERSION=$(python3 -c "import evo; print(evo.__version__)" 2>/dev/null || echo "unknown")
    echo "        evo version: $EVO_VERSION"
else
    check_fail "evo library is not installed"
    echo "        Install with: pip3 install evo"
fi

echo ""

# 3. Check evaluate_run.py help works
echo "Checking evaluate_run.py help..."

if python3 "$EVAL_SCRIPT" --help > /tmp/eval_help.log 2>&1; then
    check_pass "evaluate_run.py --help works"
    if grep -q "gt_file" /tmp/eval_help.log && grep -q "est_file" /tmp/eval_help.log; then
        check_pass "evaluate_run.py has required arguments"
    else
        check_fail "evaluate_run.py missing required arguments"
    fi
else
    check_fail "evaluate_run.py --help failed"
fi

echo ""

# 4. Create test trajectory files
echo "Creating test trajectory files..."

cat > "$TEST_DIR/gt.tum" << 'EOF'
# TUM trajectory format: timestamp tx ty tz qx qy qz qw
1000.000000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
1000.100000000 0.100000 0.050000 0.000000 0.000000 0.000000 0.025000 0.999688
1000.200000000 0.200000 0.150000 0.000000 0.000000 0.000000 0.050000 0.998750
1000.300000000 0.250000 0.300000 0.000000 0.000000 0.000000 0.100000 0.995004
1000.400000000 0.200000 0.400000 0.000000 0.000000 0.000000 0.150000 0.988771
1000.500000000 0.100000 0.450000 0.000000 0.000000 0.000000 0.200000 0.979796
1000.600000000 0.000000 0.400000 0.000000 0.000000 0.000000 0.250000 0.968246
1000.700000000 -0.050000 0.300000 0.000000 0.000000 0.000000 0.300000 0.953939
1000.800000000 0.000000 0.200000 0.000000 0.000000 0.000000 0.350000 0.936750
1000.900000000 0.050000 0.100000 0.000000 0.000000 0.000000 0.400000 0.916516
1001.000000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.450000 0.893029
EOF

cat > "$TEST_DIR/est.tum" << 'EOF'
# TUM trajectory format: timestamp tx ty tz qx qy qz qw
1000.000000000 0.010000 0.005000 0.000000 0.000000 0.000000 0.000000 1.000000
1000.100000000 0.105000 0.055000 0.000000 0.000000 0.000000 0.025000 0.999688
1000.200000000 0.208000 0.148000 0.000000 0.000000 0.000000 0.050000 0.998750
1000.300000000 0.255000 0.295000 0.000000 0.000000 0.000000 0.100000 0.995004
1000.400000000 0.195000 0.408000 0.000000 0.000000 0.000000 0.150000 0.988771
1000.500000000 0.108000 0.445000 0.000000 0.000000 0.000000 0.200000 0.979796
1000.600000000 -0.005000 0.398000 0.000000 0.000000 0.000000 0.250000 0.968246
1000.700000000 -0.052000 0.305000 0.000000 0.000000 0.000000 0.300000 0.953939
1000.800000000 0.005000 0.195000 0.000000 0.000000 0.000000 0.350000 0.936750
1000.900000000 0.055000 0.098000 0.000000 0.000000 0.000000 0.400000 0.916516
1001.000000000 0.008000 0.005000 0.000000 0.000000 0.000000 0.450000 0.893029
EOF

check_pass "Test trajectory files created"
echo ""

# 5. Test metric computation
echo "Testing metric computation..."

OUTPUT_DIR="$TEST_DIR/output"
if python3 "$EVAL_SCRIPT" \
    --gt_file "$TEST_DIR/gt.tum" \
    --est_file "$TEST_DIR/est.tum" \
    --output_dir "$OUTPUT_DIR" \
    --dataset "test_dataset" \
    --algorithm "test_algo" \
    --verbose > /tmp/eval_output.log 2>&1; then
    check_pass "evaluate_run.py completed successfully"
else
    check_fail "evaluate_run.py failed"
    cat /tmp/eval_output.log
fi

echo ""

# 6. Check metrics.json output format
echo "Checking metrics.json output format..."

METRICS_FILE="$OUTPUT_DIR/metrics.json"

if [[ -f "$METRICS_FILE" ]]; then
    check_pass "metrics.json exists"

    # Check required fields
    for field in success dataset algorithm timestamp ate rpe; do
        if python3 -c "import json; d=json.load(open('$METRICS_FILE')); assert '$field' in d" 2>/dev/null; then
            check_pass "metrics.json has '$field' field"
        else
            check_fail "metrics.json missing '$field' field"
        fi
    done

    # Check success is true
    if python3 -c "import json; d=json.load(open('$METRICS_FILE')); assert d['success'] == True" 2>/dev/null; then
        check_pass "metrics.json shows success=true"
    else
        check_fail "metrics.json shows success=false"
    fi

    # Check ATE structure
    if python3 -c "
import json
d = json.load(open('$METRICS_FILE'))
ate = d['ate']
assert 'rmse' in ate
assert 'mean' in ate
assert 'median' in ate
assert 'std' in ate
assert 'min' in ate
assert 'max' in ate
" 2>/dev/null; then
        check_pass "ATE has all required statistics"
    else
        check_fail "ATE missing required statistics"
    fi

    # Check RPE structure
    if python3 -c "
import json
d = json.load(open('$METRICS_FILE'))
rpe = d['rpe']
assert 'rmse' in rpe
assert 'mean' in rpe
assert 'median' in rpe
assert 'std' in rpe
assert 'min' in rpe
assert 'max' in rpe
" 2>/dev/null; then
        check_pass "RPE has all required statistics"
    else
        check_fail "RPE missing required statistics"
    fi

    # Print computed metrics
    echo ""
    echo "Computed metrics:"
    python3 -c "
import json
d = json.load(open('$METRICS_FILE'))
print(f\"  ATE RMSE: {d['ate']['rmse']:.4f} m\")
print(f\"  RPE RMSE: {d['rpe']['rmse']:.4f} m\")
print(f\"  Trajectory length: {d['trajectory_length_m']:.2f} m\")
print(f\"  Duration: {d['duration_s']:.2f} s\")
print(f\"  Poses: {d['num_poses']}\")
"
else
    check_fail "metrics.json not created"
fi

echo ""

# 7. Test failure handling
echo "Testing failure handling..."

FAIL_OUTPUT_DIR="$TEST_DIR/fail_output"
if python3 "$EVAL_SCRIPT" \
    --gt_file "$TEST_DIR/nonexistent.tum" \
    --est_file "$TEST_DIR/est.tum" \
    --output_dir "$FAIL_OUTPUT_DIR" \
    --dataset "fail_test" \
    --algorithm "test" 2>/dev/null; then
    check_fail "Script should have failed for missing file"
else
    check_pass "Script correctly fails for missing file"
fi

FAIL_METRICS="$FAIL_OUTPUT_DIR/metrics.json"
if [[ -f "$FAIL_METRICS" ]]; then
    if python3 -c "import json; d=json.load(open('$FAIL_METRICS')); assert d['success'] == False" 2>/dev/null; then
        check_pass "Failure recorded with success=false"
    else
        check_fail "Failure not recorded correctly"
    fi

    if python3 -c "import json; d=json.load(open('$FAIL_METRICS')); assert d['error'] is not None" 2>/dev/null; then
        check_pass "Error message recorded"
    else
        check_fail "Error message not recorded"
    fi
else
    check_fail "Failure metrics.json not created"
fi

echo ""

# 8. Test plot generation
echo "Testing plot generation..."

PLOT_OUTPUT_DIR="$TEST_DIR/plot_output"
if python3 "$EVAL_SCRIPT" \
    --gt_file "$TEST_DIR/gt.tum" \
    --est_file "$TEST_DIR/est.tum" \
    --output_dir "$PLOT_OUTPUT_DIR" \
    --dataset "plot_test" \
    --algorithm "test" \
    --save_plots 2>/dev/null; then

    if [[ -f "$PLOT_OUTPUT_DIR/ate_plot.png" ]]; then
        check_pass "ATE plot generated"
    else
        check_warn "ATE plot not generated (non-critical)"
    fi

    if [[ -f "$PLOT_OUTPUT_DIR/rpe_plot.png" ]]; then
        check_pass "RPE plot generated"
    else
        check_warn "RPE plot not generated (non-critical)"
    fi
else
    check_warn "Plot generation test failed (non-critical)"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M7 VALIDATION PASSED${NC}"
    echo ""
    echo "The evaluation script is ready to use."
    echo ""
    echo "Usage examples:"
    echo "  # Evaluate a SLAM run"
    echo "  python3 $EVAL_SCRIPT \\"
    echo "    --gt_file /path/to/gt.tum \\"
    echo "    --est_file /path/to/est.tum \\"
    echo "    --output_dir /path/to/results \\"
    echo "    --dataset traj_01_easy__baseline \\"
    echo "    --algorithm slam_toolbox \\"
    echo "    --save_plots --verbose"
    echo ""
    echo "Output files:"
    echo "  - metrics.json: ATE/RPE statistics"
    echo "  - ate_plot.png: Trajectory comparison plot"
    echo "  - rpe_plot.png: RPE distribution histogram"
    echo ""
    exit 0
else
    echo -e "${RED}M7 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
