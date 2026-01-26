#!/bin/bash
#
# M8 Validation Script - Experiment Runner
#
# Validates:
# 1. run_one.py script exists and has required arguments
# 2. run_all.py script exists and has required arguments
# 3. aggregate_results.py script exists and has required arguments
# 4. Scripts have correct help output
# 5. Package builds successfully
#
# Note: Full end-to-end testing requires recorded bags and ROS2 runtime.
# This validation checks structure and basic functionality only.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$THESIS_DIR/ros2_ws"
SCRIPTS_DIR="$WS_DIR/src/slam_thesis/experiment_runner/scripts"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M8 Validation - Experiment Runner"
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

# 1. Check run_one.py exists
echo "Checking run_one.py..."

RUN_ONE_SCRIPT="$SCRIPTS_DIR/run_one.py"

if [[ -f "$RUN_ONE_SCRIPT" ]]; then
    check_pass "run_one.py exists"
else
    check_fail "run_one.py missing: $RUN_ONE_SCRIPT"
fi

if [[ -x "$RUN_ONE_SCRIPT" ]]; then
    check_pass "run_one.py is executable"
else
    check_warn "run_one.py is not executable (will use python3)"
fi

# Check run_one.py help
if python3 "$RUN_ONE_SCRIPT" --help > /tmp/run_one_help.log 2>&1; then
    check_pass "run_one.py --help works"

    # Check required arguments
    if grep -q "bag_path" /tmp/run_one_help.log && grep -q "algorithm" /tmp/run_one_help.log; then
        check_pass "run_one.py has required arguments (bag_path, algorithm)"
    else
        check_fail "run_one.py missing required arguments"
    fi

    # Check algorithm choices
    if grep -q "slam_toolbox" /tmp/run_one_help.log && grep -q "cartographer" /tmp/run_one_help.log; then
        check_pass "run_one.py supports slam_toolbox and cartographer"
    else
        check_fail "run_one.py missing algorithm choices"
    fi
else
    check_fail "run_one.py --help failed"
fi

echo ""

# 2. Check run_all.py exists
echo "Checking run_all.py..."

RUN_ALL_SCRIPT="$SCRIPTS_DIR/run_all.py"

if [[ -f "$RUN_ALL_SCRIPT" ]]; then
    check_pass "run_all.py exists"
else
    check_fail "run_all.py missing: $RUN_ALL_SCRIPT"
fi

if [[ -x "$RUN_ALL_SCRIPT" ]]; then
    check_pass "run_all.py is executable"
else
    check_warn "run_all.py is not executable (will use python3)"
fi

# Check run_all.py help
if python3 "$RUN_ALL_SCRIPT" --help > /tmp/run_all_help.log 2>&1; then
    check_pass "run_all.py --help works"

    # Check required arguments
    if grep -q "bags_dir" /tmp/run_all_help.log; then
        check_pass "run_all.py has required argument (bags_dir)"
    else
        check_fail "run_all.py missing required arguments"
    fi

    # Check parallel support
    if grep -q "parallel" /tmp/run_all_help.log; then
        check_pass "run_all.py supports parallel execution"
    else
        check_warn "run_all.py missing parallel support"
    fi

    # Check filter support
    if grep -q "filter" /tmp/run_all_help.log; then
        check_pass "run_all.py supports bag filtering"
    else
        check_warn "run_all.py missing filter support"
    fi
else
    check_fail "run_all.py --help failed"
fi

echo ""

# 3. Check aggregate_results.py exists
echo "Checking aggregate_results.py..."

AGGREGATE_SCRIPT="$SCRIPTS_DIR/aggregate_results.py"

if [[ -f "$AGGREGATE_SCRIPT" ]]; then
    check_pass "aggregate_results.py exists"
else
    check_fail "aggregate_results.py missing: $AGGREGATE_SCRIPT"
fi

if [[ -x "$AGGREGATE_SCRIPT" ]]; then
    check_pass "aggregate_results.py is executable"
else
    check_warn "aggregate_results.py is not executable (will use python3)"
fi

# Check aggregate_results.py help
if python3 "$AGGREGATE_SCRIPT" --help > /tmp/aggregate_help.log 2>&1; then
    check_pass "aggregate_results.py --help works"

    # Check required arguments
    if grep -q "bags_dir" /tmp/aggregate_help.log; then
        check_pass "aggregate_results.py has required argument (bags_dir)"
    else
        check_fail "aggregate_results.py missing required arguments"
    fi

    # Check output format support
    if grep -q "csv" /tmp/aggregate_help.log && grep -q "json" /tmp/aggregate_help.log; then
        check_pass "aggregate_results.py supports CSV and JSON output"
    else
        check_warn "aggregate_results.py missing output format options"
    fi
else
    check_fail "aggregate_results.py --help failed"
fi

echo ""

# 4. Check evaluate_run.py exists (dependency from M7)
echo "Checking M7 dependency (evaluate_run.py)..."

EVAL_SCRIPT="$SCRIPTS_DIR/evaluate_run.py"

if [[ -f "$EVAL_SCRIPT" ]]; then
    check_pass "evaluate_run.py exists (M7 dependency)"
else
    check_fail "evaluate_run.py missing (M7 dependency): $EVAL_SCRIPT"
fi

echo ""

# 5. Check package structure
echo "Checking package structure..."

# Check setup.py includes scripts
SETUP_PY="$WS_DIR/src/slam_thesis/experiment_runner/setup.py"
if [[ -f "$SETUP_PY" ]]; then
    if grep -q "scripts" "$SETUP_PY"; then
        check_pass "setup.py includes scripts directory"
    else
        check_fail "setup.py does not include scripts"
    fi
else
    check_fail "setup.py not found"
fi

echo ""

# 6. Test aggregate_results.py with mock data
echo "Testing aggregate_results.py with mock data..."

# Create mock directory structure
MOCK_BAGS_DIR="$TEST_DIR/bags"
mkdir -p "$MOCK_BAGS_DIR/bag_test_01/results/slam_toolbox"
mkdir -p "$MOCK_BAGS_DIR/bag_test_02/results/cartographer"

# Create mock metadata.yaml files
echo "rosbag2_bagfile_information:" > "$MOCK_BAGS_DIR/bag_test_01/metadata.yaml"
echo "rosbag2_bagfile_information:" > "$MOCK_BAGS_DIR/bag_test_02/metadata.yaml"

# Create mock metrics.json files
cat > "$MOCK_BAGS_DIR/bag_test_01/results/slam_toolbox/metrics.json" << 'EOF'
{
  "success": true,
  "dataset": "traj_01_easy_baseline",
  "algorithm": "slam_toolbox",
  "timestamp": "2026-01-26T15:00:00",
  "ate": {"rmse": 0.05, "mean": 0.04, "median": 0.03, "std": 0.02, "min": 0.01, "max": 0.10},
  "rpe": {"rmse": 0.02, "mean": 0.015, "median": 0.012, "std": 0.008, "min": 0.005, "max": 0.05},
  "trajectory_length_m": 45.0,
  "duration_s": 90.0,
  "num_poses": 900,
  "error": null
}
EOF

cat > "$MOCK_BAGS_DIR/bag_test_02/results/cartographer/metrics.json" << 'EOF'
{
  "success": true,
  "dataset": "traj_01_easy_baseline",
  "algorithm": "cartographer",
  "timestamp": "2026-01-26T15:30:00",
  "ate": {"rmse": 0.06, "mean": 0.05, "median": 0.04, "std": 0.025, "min": 0.01, "max": 0.12},
  "rpe": {"rmse": 0.025, "mean": 0.02, "median": 0.015, "std": 0.01, "min": 0.006, "max": 0.06},
  "trajectory_length_m": 45.0,
  "duration_s": 90.0,
  "num_poses": 890,
  "error": null
}
EOF

check_pass "Mock data created"

# Run aggregate_results.py
if python3 "$AGGREGATE_SCRIPT" \
    --bags_dir "$MOCK_BAGS_DIR" \
    --output "$TEST_DIR/test_summary" \
    --format both > /tmp/aggregate_output.log 2>&1; then
    check_pass "aggregate_results.py completed successfully"

    # Check output files
    if [[ -f "$TEST_DIR/test_summary.csv" ]]; then
        check_pass "CSV output created"

        # Check CSV has expected columns
        if head -1 "$TEST_DIR/test_summary.csv" | grep -q "algorithm" && \
           head -1 "$TEST_DIR/test_summary.csv" | grep -q "ate_rmse"; then
            check_pass "CSV has expected columns"
        else
            check_fail "CSV missing expected columns"
        fi
    else
        check_fail "CSV output not created"
    fi

    if [[ -f "$TEST_DIR/test_summary.json" ]]; then
        check_pass "JSON output created"

        # Check JSON structure
        if python3 -c "
import json
d = json.load(open('$TEST_DIR/test_summary.json'))
assert 'aggregations' in d
assert 'individual_results' in d
assert d['successful'] == 2
" 2>/dev/null; then
            check_pass "JSON has expected structure"
        else
            check_fail "JSON missing expected structure"
        fi
    else
        check_fail "JSON output not created"
    fi
else
    check_fail "aggregate_results.py failed"
    cat /tmp/aggregate_output.log
fi

echo ""

# 7. Build test
echo "Testing package build..."

cd "$WS_DIR"

# Source ROS2 if available
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi

# Build experiment_runner package
if colcon build --packages-select experiment_runner > /tmp/build_output.log 2>&1; then
    check_pass "experiment_runner package builds successfully"
else
    check_fail "experiment_runner package build failed"
    tail -20 /tmp/build_output.log
fi

# Check scripts are installed
INSTALL_SCRIPTS_DIR="$WS_DIR/install/experiment_runner/share/experiment_runner/scripts"
if [[ -d "$INSTALL_SCRIPTS_DIR" ]]; then
    if [[ -f "$INSTALL_SCRIPTS_DIR/run_one.py" ]]; then
        check_pass "run_one.py installed to share directory"
    else
        check_fail "run_one.py not installed"
    fi

    if [[ -f "$INSTALL_SCRIPTS_DIR/run_all.py" ]]; then
        check_pass "run_all.py installed to share directory"
    else
        check_fail "run_all.py not installed"
    fi

    if [[ -f "$INSTALL_SCRIPTS_DIR/aggregate_results.py" ]]; then
        check_pass "aggregate_results.py installed to share directory"
    else
        check_fail "aggregate_results.py not installed"
    fi
else
    check_warn "Scripts directory not found in install (may require workspace rebuild)"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M8 VALIDATION PASSED${NC}"
    echo ""
    echo "The experiment runner scripts are ready to use."
    echo ""
    echo "Usage examples:"
    echo ""
    echo "  # Run a single experiment"
    echo "  python3 $RUN_ONE_SCRIPT \\"
    echo "    --bag_path /path/to/bag \\"
    echo "    --algorithm slam_toolbox \\"
    echo "    --verbose"
    echo ""
    echo "  # Run all experiments"
    echo "  python3 $RUN_ALL_SCRIPT \\"
    echo "    --bags_dir ~/thesis/ros2_ws/bags \\"
    echo "    --algorithms slam_toolbox cartographer \\"
    echo "    --parallel 2"
    echo ""
    echo "  # Aggregate results"
    echo "  python3 $AGGREGATE_SCRIPT \\"
    echo "    --bags_dir ~/thesis/ros2_ws/bags \\"
    echo "    --output results_summary"
    echo ""
    echo "Output files:"
    echo "  - bag_path/results/{algorithm}/metrics.json"
    echo "  - bag_path/{algorithm}_trajectory.tum"
    echo "  - results_summary.csv / .json"
    echo ""
    exit 0
else
    echo -e "${RED}M8 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
