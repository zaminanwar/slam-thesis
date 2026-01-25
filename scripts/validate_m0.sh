#!/bin/bash
#
# SLAM Thesis - Milestone 0 Validation
# Validates: Workspace structure and colcon build
#
# Usage: bash ~/thesis/scripts/validate_m0.sh
#

echo "=============================================="
echo "Milestone 0 Validation: Foundation"
echo "=============================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS_COUNT=0
FAIL_COUNT=0

check_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Detect ROS distro from saved file or try both
if [[ -f ~/thesis/.ros_distro ]]; then
    ROS_DISTRO=$(cat ~/thesis/.ros_distro)
else
    # Try to detect
    if [[ -d /opt/ros/jazzy ]]; then
        ROS_DISTRO="jazzy"
    elif [[ -d /opt/ros/humble ]]; then
        ROS_DISTRO="humble"
    else
        ROS_DISTRO=""
    fi
fi

# Check 1: ROS2 installed
echo "Check 1: ROS2 installation..."
if [[ -n "$ROS_DISTRO" ]] && source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null; then
    check_pass "ROS2 ${ROS_DISTRO^} is installed"
else
    check_fail "ROS2 is NOT installed - run install_dependencies.sh"
fi

# Check 2: Workspace directory exists
echo "Check 2: Workspace directory structure..."
if [[ -d ~/thesis/ros2_ws/src/slam_thesis ]]; then
    check_pass "Workspace directory exists"
else
    check_fail "Workspace directory ~/thesis/ros2_ws/src/slam_thesis not found"
fi

# Check 3: All 7 packages exist
echo "Check 3: Package directories..."
PACKAGES=("rover_description" "rover_sim" "rover_control" "gt_publisher" "slam_launch" "traj_exporter" "experiment_runner")
for pkg in "${PACKAGES[@]}"; do
    if [[ -d ~/thesis/ros2_ws/src/slam_thesis/$pkg ]]; then
        check_pass "Package $pkg exists"
    else
        check_fail "Package $pkg NOT found"
    fi
done

# Check 4: package.xml files exist
echo "Check 4: package.xml files..."
for pkg in "${PACKAGES[@]}"; do
    if [[ -f ~/thesis/ros2_ws/src/slam_thesis/$pkg/package.xml ]]; then
        check_pass "$pkg/package.xml exists"
    else
        check_fail "$pkg/package.xml NOT found"
    fi
done

# Check 5: .claude directory exists
echo "Check 5: Claude continuity system..."
if [[ -d ~/thesis/.claude ]]; then
    check_pass ".claude directory exists"

    for file in ORIENTATION.md STATE.md DECISIONS.md INTERFACES.md RECOVERY.md; do
        if [[ -f ~/thesis/.claude/$file ]]; then
            check_pass ".claude/$file exists"
        else
            check_fail ".claude/$file NOT found"
        fi
    done
else
    check_fail ".claude directory NOT found"
fi

# Check 6: colcon build (only if ROS2 is installed)
echo "Check 6: colcon build..."
if command -v colcon &> /dev/null && [[ -n "$ROS_DISTRO" ]]; then
    cd ~/thesis/ros2_ws

    # Try to build
    if colcon build 2>&1 | tee /tmp/colcon_build_output.txt | tail -10; then
        if grep -q "packages finished" /tmp/colcon_build_output.txt; then
            check_pass "colcon build succeeded"
        else
            check_fail "colcon build had issues - check output above"
        fi
    else
        check_fail "colcon build failed"
    fi
else
    check_warn "Skipping colcon build - ROS2 not sourced or not installed"
fi

# Summary
echo ""
echo "=============================================="
echo "Validation Summary"
echo "=============================================="
echo -e "ROS2 Distro: ${ROS_DISTRO:-unknown}"
echo -e "Passed: ${GREEN}$PASS_COUNT${NC}"
echo -e "Failed: ${RED}$FAIL_COUNT${NC}"
echo ""

if [[ $FAIL_COUNT -eq 0 ]]; then
    echo -e "${GREEN}Milestone 0: PASSED${NC}"
    echo ""
    echo "Next: Claude will implement M1 (Simulation)"
    exit 0
else
    echo -e "${RED}Milestone 0: FAILED${NC}"
    echo ""
    echo "Please fix the issues above before proceeding."
    exit 1
fi
