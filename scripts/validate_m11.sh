#!/bin/bash
#
# M11 Validation Script - Navigation Experiments
#
# Validates:
# 1. run_nav_experiment.py exists and is valid
# 2. Goal configuration files exist
# 3. Documentation updated for Nav2 integration
# 4. Integration with existing experiment infrastructure
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$THESIS_DIR/ros2_ws"
EXP_RUNNER_DIR="$WS_DIR/src/slam_thesis/experiment_runner"
CLAUDE_DIR="$THESIS_DIR/.claude"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M11 Validation - Navigation Experiments"
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

# 1. Check run_nav_experiment.py
echo "Checking navigation experiment script..."

NAV_EXP_SCRIPT="$EXP_RUNNER_DIR/scripts/run_nav_experiment.py"

if [[ -f "$NAV_EXP_SCRIPT" ]]; then
    check_pass "run_nav_experiment.py exists"

    # Check Python syntax
    if python3 -m py_compile "$NAV_EXP_SCRIPT" 2>/dev/null; then
        check_pass "run_nav_experiment.py has valid Python syntax"
    else
        check_fail "run_nav_experiment.py has Python syntax errors"
    fi

    # Check for required arguments
    if grep -q "algorithm" "$NAV_EXP_SCRIPT"; then
        check_pass "run_nav_experiment.py has algorithm argument"
    else
        check_fail "run_nav_experiment.py missing algorithm argument"
    fi

    if grep -q "goals" "$NAV_EXP_SCRIPT"; then
        check_pass "run_nav_experiment.py has goals argument"
    else
        check_fail "run_nav_experiment.py missing goals argument"
    fi

    # Check for output generation
    if grep -q "nav_results" "$NAV_EXP_SCRIPT" || grep -q "json" "$NAV_EXP_SCRIPT"; then
        check_pass "run_nav_experiment.py generates results output"
    else
        check_warn "run_nav_experiment.py may not generate results output"
    fi

    # Check for NavigateToPose action client
    if grep -q "NavigateToPose\|navigate_to_pose" "$NAV_EXP_SCRIPT"; then
        check_pass "run_nav_experiment.py uses NavigateToPose action"
    else
        check_fail "run_nav_experiment.py missing NavigateToPose action"
    fi

    # Check it's executable
    if [[ -x "$NAV_EXP_SCRIPT" ]]; then
        check_pass "run_nav_experiment.py is executable"
    else
        check_warn "run_nav_experiment.py is not executable (chmod +x recommended)"
    fi
else
    check_fail "run_nav_experiment.py not found: $NAV_EXP_SCRIPT"
fi

echo ""

# 2. Check goal configuration files
echo "Checking goal configuration files..."

CONFIG_DIR="$EXP_RUNNER_DIR/config"

if [[ -d "$CONFIG_DIR" ]]; then
    check_pass "config/ directory exists"
else
    check_fail "config/ directory not found"
fi

# Check nav_goals_01.yaml (simple square path)
GOALS_01="$CONFIG_DIR/nav_goals_01.yaml"
if [[ -f "$GOALS_01" ]]; then
    check_pass "nav_goals_01.yaml exists"

    # Check YAML structure
    if grep -q "goals:" "$GOALS_01"; then
        check_pass "nav_goals_01.yaml has goals key"
    else
        check_fail "nav_goals_01.yaml missing goals key"
    fi

    # Count goals (should have at least 3 for a meaningful path)
    GOAL_COUNT=$(grep -c "^\s*- x:" "$GOALS_01" 2>/dev/null || echo "0")
    if [[ $GOAL_COUNT -ge 3 ]]; then
        check_pass "nav_goals_01.yaml has $GOAL_COUNT goals (sufficient)"
    else
        check_warn "nav_goals_01.yaml has only $GOAL_COUNT goals (recommend 3+)"
    fi

    # Check for required fields
    if grep -q "x:" "$GOALS_01" && grep -q "y:" "$GOALS_01"; then
        check_pass "nav_goals_01.yaml has x/y coordinates"
    else
        check_fail "nav_goals_01.yaml missing x/y coordinates"
    fi
else
    check_fail "nav_goals_01.yaml not found"
fi

# Check nav_goals_02.yaml (complex exploration)
GOALS_02="$CONFIG_DIR/nav_goals_02.yaml"
if [[ -f "$GOALS_02" ]]; then
    check_pass "nav_goals_02.yaml exists"

    # Check YAML structure
    if grep -q "goals:" "$GOALS_02"; then
        check_pass "nav_goals_02.yaml has goals key"
    else
        check_fail "nav_goals_02.yaml missing goals key"
    fi

    # Count goals (complex should have more)
    GOAL_COUNT=$(grep -c "^\s*- x:" "$GOALS_02" 2>/dev/null || echo "0")
    if [[ $GOAL_COUNT -ge 5 ]]; then
        check_pass "nav_goals_02.yaml has $GOAL_COUNT goals (complex path)"
    else
        check_warn "nav_goals_02.yaml has only $GOAL_COUNT goals (recommend 5+ for complex)"
    fi
else
    check_fail "nav_goals_02.yaml not found"
fi

echo ""

# 3. Check documentation updates
echo "Checking documentation updates..."

# Check STATE.md has Nav2 milestones
if [[ -f "$CLAUDE_DIR/STATE.md" ]]; then
    if grep -q "EPIC 10\|M10\|Nav2" "$CLAUDE_DIR/STATE.md"; then
        check_pass "STATE.md has Nav2/M10 documentation"
    else
        check_fail "STATE.md missing Nav2/M10 documentation"
    fi

    if grep -q "EPIC 11\|M11\|Navigation Experiment" "$CLAUDE_DIR/STATE.md"; then
        check_pass "STATE.md has M11 documentation"
    else
        check_fail "STATE.md missing M11 documentation"
    fi
else
    check_fail "STATE.md not found"
fi

# Check INTERFACES.md has Nav2 interfaces
if [[ -f "$CLAUDE_DIR/INTERFACES.md" ]]; then
    if grep -q "navigate_to_pose\|NavigateToPose" "$CLAUDE_DIR/INTERFACES.md"; then
        check_pass "INTERFACES.md has Nav2 action documentation"
    else
        check_warn "INTERFACES.md may be missing Nav2 action documentation"
    fi

    if grep -q "nav_results" "$CLAUDE_DIR/INTERFACES.md"; then
        check_pass "INTERFACES.md has nav_results format"
    else
        check_warn "INTERFACES.md may be missing nav_results format"
    fi
else
    check_fail "INTERFACES.md not found"
fi

# Check DECISIONS.md has Nav2 decisions
if [[ -f "$CLAUDE_DIR/DECISIONS.md" ]]; then
    if grep -q "AD-011\|No AMCL\|AMCL" "$CLAUDE_DIR/DECISIONS.md"; then
        check_pass "DECISIONS.md has AD-011 (No AMCL decision)"
    else
        check_fail "DECISIONS.md missing AD-011 (No AMCL decision)"
    fi

    if grep -q "AD-012\|DWB\|Local Planner" "$CLAUDE_DIR/DECISIONS.md"; then
        check_pass "DECISIONS.md has AD-012 (DWB planner decision)"
    else
        check_fail "DECISIONS.md missing AD-012 (DWB planner decision)"
    fi
else
    check_fail "DECISIONS.md not found"
fi

# Check ORIENTATION.md has Nav2 info
if [[ -f "$CLAUDE_DIR/ORIENTATION.md" ]]; then
    if grep -q "nav_launch" "$CLAUDE_DIR/ORIENTATION.md"; then
        check_pass "ORIENTATION.md mentions nav_launch package"
    else
        check_fail "ORIENTATION.md missing nav_launch package reference"
    fi

    if grep -q "run_nav_experiment" "$CLAUDE_DIR/ORIENTATION.md"; then
        check_pass "ORIENTATION.md has run_nav_experiment command examples"
    else
        check_warn "ORIENTATION.md may be missing run_nav_experiment examples"
    fi
else
    check_fail "ORIENTATION.md not found"
fi

echo ""

# 4. Check integration with experiment infrastructure
echo "Checking integration with experiment infrastructure..."

# Check results directory can be created
RESULTS_DIR="$WS_DIR/results"
if [[ -d "$RESULTS_DIR" ]] || mkdir -p "$RESULTS_DIR" 2>/dev/null; then
    check_pass "Results directory accessible: $RESULTS_DIR"
else
    check_fail "Cannot access/create results directory"
fi

# Check experiment_runner package exists
if [[ -d "$EXP_RUNNER_DIR" ]]; then
    check_pass "experiment_runner package exists"

    # Check it has required scripts
    if [[ -f "$EXP_RUNNER_DIR/scripts/run_one.py" ]]; then
        check_pass "run_one.py exists (bag-based experiments)"
    else
        check_fail "run_one.py not found"
    fi

    if [[ -f "$EXP_RUNNER_DIR/scripts/run_all.py" ]]; then
        check_pass "run_all.py exists (batch experiments)"
    else
        check_fail "run_all.py not found"
    fi
else
    check_fail "experiment_runner package not found"
fi

echo ""

# 5. Check nav2_integration_plan.md exists
echo "Checking integration plan documentation..."

PLAN_FILE="$CLAUDE_DIR/plans/nav2_integration_plan.md"
if [[ -f "$PLAN_FILE" ]]; then
    check_pass "nav2_integration_plan.md exists"

    # Check all phases documented
    for phase in 1 2 3 4 5; do
        if grep -q "Phase $phase" "$PLAN_FILE"; then
            check_pass "Plan documents Phase $phase"
        else
            check_warn "Plan missing Phase $phase documentation"
        fi
    done
else
    check_warn "nav2_integration_plan.md not found (optional)"
fi

echo ""

# 6. Verify M10 passed (dependency)
echo "Checking M10 dependency..."

if [[ -f "$SCRIPT_DIR/validate_m10.sh" ]]; then
    check_pass "validate_m10.sh exists"

    # Run M10 validation silently
    if bash "$SCRIPT_DIR/validate_m10.sh" >/dev/null 2>&1; then
        check_pass "M10 validation passes (Nav2 package ready)"
    else
        check_warn "M10 validation has issues (run validate_m10.sh for details)"
    fi
else
    check_warn "validate_m10.sh not found (cannot verify M10)"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M11 VALIDATION PASSED${NC}"
    echo ""
    echo "Navigation Experiments infrastructure is complete:"
    echo "  - Script: run_nav_experiment.py"
    echo "  - Goals: nav_goals_01.yaml (simple), nav_goals_02.yaml (complex)"
    echo "  - Documentation: Updated STATE, INTERFACES, DECISIONS, ORIENTATION"
    echo ""
    echo "Usage:"
    echo "  python3 $NAV_EXP_SCRIPT \\"
    echo "    --algorithm slam_toolbox \\"
    echo "    --goals $GOALS_01"
    echo ""
    echo "Next steps:"
    echo "  1. Launch Nav2: ros2 launch nav_launch nav2_slam.launch.py"
    echo "  2. Run experiment: python3 run_nav_experiment.py --algorithm slam_toolbox --goals nav_goals_01.yaml"
    echo "  3. Compare algorithms: Run with --algorithm cartographer"
    echo ""
    exit 0
else
    echo -e "${RED}M11 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
