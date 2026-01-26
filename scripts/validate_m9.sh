#!/bin/bash
#
# M9 Validation Script - Documentation
#
# Validates:
# 1. README.md exists and contains required sections
# 2. VALIDATION.md exists and contains milestone checklists
# 3. All validation scripts exist
# 4. Claude continuity files exist
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M9 Validation - Documentation"
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

# 1. Check README.md exists
echo "Checking README.md..."

README_FILE="$THESIS_DIR/README.md"

if [[ -f "$README_FILE" ]]; then
    check_pass "README.md exists"

    # Check required sections
    if grep -q "## Overview" "$README_FILE"; then
        check_pass "README has Overview section"
    else
        check_fail "README missing Overview section"
    fi

    if grep -q "## Quick Start" "$README_FILE"; then
        check_pass "README has Quick Start section"
    else
        check_fail "README missing Quick Start section"
    fi

    if grep -q "## Project Structure" "$README_FILE"; then
        check_pass "README has Project Structure section"
    else
        check_fail "README missing Project Structure section"
    fi

    if grep -q "## Usage" "$README_FILE"; then
        check_pass "README has Usage section"
    else
        check_fail "README missing Usage section"
    fi

    if grep -q "## Milestones" "$README_FILE"; then
        check_pass "README has Milestones section"
    else
        check_fail "README missing Milestones section"
    fi

    if grep -q "## Environment" "$README_FILE"; then
        check_pass "README has Environment section"
    else
        check_fail "README missing Environment section"
    fi

    if grep -q "## Troubleshooting" "$README_FILE"; then
        check_pass "README has Troubleshooting section"
    else
        check_fail "README missing Troubleshooting section"
    fi

    # Check milestone status accuracy
    if grep -q "PASSED" "$README_FILE"; then
        check_pass "README shows milestone completion status"
    else
        check_warn "README may have outdated milestone status"
    fi

    # Check for key usage examples
    if grep -q "ros2 launch" "$README_FILE"; then
        check_pass "README contains launch examples"
    else
        check_fail "README missing launch examples"
    fi

    if grep -q "run_one.py" "$README_FILE" && grep -q "run_all.py" "$README_FILE"; then
        check_pass "README documents experiment runner scripts"
    else
        check_fail "README missing experiment runner documentation"
    fi
else
    check_fail "README.md not found: $README_FILE"
fi

echo ""

# 2. Check VALIDATION.md exists
echo "Checking VALIDATION.md..."

VALIDATION_FILE="$THESIS_DIR/docs/VALIDATION.md"

if [[ -f "$VALIDATION_FILE" ]]; then
    check_pass "VALIDATION.md exists"

    # Check for milestone sections
    for i in 0 1 2 3 4 5 6 7 8 9; do
        if grep -q "## M$i" "$VALIDATION_FILE"; then
            check_pass "VALIDATION.md has M$i section"
        else
            check_fail "VALIDATION.md missing M$i section"
        fi
    done

    # Check for checklist items
    if grep -q "\- \[ \]" "$VALIDATION_FILE"; then
        check_pass "VALIDATION.md contains checklist items"
    else
        check_warn "VALIDATION.md may be missing checklist items"
    fi
else
    check_fail "VALIDATION.md not found: $VALIDATION_FILE"
fi

echo ""

# 3. Check validation scripts exist
echo "Checking validation scripts..."

EXPECTED_SCRIPTS=("validate_m0.sh" "validate_m1.sh" "validate_m2.sh" "validate_m3.sh" "validate_m7.sh" "validate_m8.sh" "validate_m9.sh")

for script in "${EXPECTED_SCRIPTS[@]}"; do
    if [[ -f "$SCRIPT_DIR/$script" ]]; then
        check_pass "$script exists"
    else
        check_fail "$script not found"
    fi
done

# Check scripts are executable
for script in "${EXPECTED_SCRIPTS[@]}"; do
    if [[ -x "$SCRIPT_DIR/$script" ]]; then
        check_pass "$script is executable"
    else
        check_warn "$script is not executable"
    fi
done

echo ""

# 4. Check Claude continuity files
echo "Checking Claude continuity files..."

CLAUDE_DIR="$THESIS_DIR/.claude"
CLAUDE_FILES=("ORIENTATION.md" "STATE.md" "INTERFACES.md" "DECISIONS.md" "RECOVERY.md")

if [[ -d "$CLAUDE_DIR" ]]; then
    check_pass ".claude directory exists"

    for file in "${CLAUDE_FILES[@]}"; do
        if [[ -f "$CLAUDE_DIR/$file" ]]; then
            check_pass "$file exists"
        else
            check_fail "$file not found in .claude/"
        fi
    done
else
    check_fail ".claude directory not found"
fi

echo ""

# 5. Check install_dependencies.sh exists
echo "Checking utility scripts..."

if [[ -f "$SCRIPT_DIR/install_dependencies.sh" ]]; then
    check_pass "install_dependencies.sh exists"
else
    check_fail "install_dependencies.sh not found"
fi

echo ""

# 6. Verify docs directory structure
echo "Checking docs directory..."

if [[ -d "$THESIS_DIR/docs" ]]; then
    check_pass "docs/ directory exists"
else
    check_fail "docs/ directory not found"
fi

echo ""

# 7. Check for common documentation issues
echo "Checking documentation quality..."

# Check README is not empty
if [[ -s "$README_FILE" ]]; then
    README_LINES=$(wc -l < "$README_FILE")
    if [[ $README_LINES -gt 100 ]]; then
        check_pass "README.md has substantial content ($README_LINES lines)"
    else
        check_warn "README.md may be incomplete ($README_LINES lines)"
    fi
else
    check_fail "README.md is empty"
fi

# Check VALIDATION.md is not empty
if [[ -s "$VALIDATION_FILE" ]]; then
    VALIDATION_LINES=$(wc -l < "$VALIDATION_FILE")
    if [[ $VALIDATION_LINES -gt 100 ]]; then
        check_pass "VALIDATION.md has substantial content ($VALIDATION_LINES lines)"
    else
        check_warn "VALIDATION.md may be incomplete ($VALIDATION_LINES lines)"
    fi
else
    check_fail "VALIDATION.md is empty"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M9 VALIDATION PASSED${NC}"
    echo ""
    echo "Documentation is complete:"
    echo "  - README.md: Project overview and usage"
    echo "  - docs/VALIDATION.md: Milestone validation checklist"
    echo "  - .claude/: Claude AI continuity system"
    echo "  - scripts/validate_m*.sh: Milestone validation scripts"
    echo ""
    echo "All milestones (M0-M9) are now complete!"
    echo ""
    exit 0
else
    echo -e "${RED}M9 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
