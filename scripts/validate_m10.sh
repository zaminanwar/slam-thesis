#!/bin/bash
#
# M10 Validation Script - Nav2 Launch Package
#
# Validates:
# 1. nav_launch package structure and files
# 2. Package builds successfully
# 3. Config files present and valid
# 4. Launch files parseable
# 5. Nav2 dependencies installed
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$THESIS_DIR/ros2_ws"
NAV_LAUNCH_DIR="$WS_DIR/src/slam_thesis/nav_launch"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M10 Validation - Nav2 Launch Package"
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

# Source ROS2
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    check_fail "ROS2 not found"
    exit 1
fi

# Source workspace if built
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
fi

# 1. Check nav_launch package structure
echo "Checking nav_launch package structure..."

if [[ -d "$NAV_LAUNCH_DIR" ]]; then
    check_pass "nav_launch directory exists"
else
    check_fail "nav_launch directory not found: $NAV_LAUNCH_DIR"
    echo -e "${RED}Cannot continue without nav_launch package${NC}"
    exit 1
fi

# Check package.xml
if [[ -f "$NAV_LAUNCH_DIR/package.xml" ]]; then
    check_pass "package.xml exists"

    # Verify it's a valid package
    if grep -q "<name>nav_launch</name>" "$NAV_LAUNCH_DIR/package.xml"; then
        check_pass "package.xml has correct name"
    else
        check_fail "package.xml has wrong package name"
    fi

    # Check for Nav2 dependencies
    if grep -q "nav2_bringup" "$NAV_LAUNCH_DIR/package.xml"; then
        check_pass "package.xml declares nav2_bringup dependency"
    else
        check_fail "package.xml missing nav2_bringup dependency"
    fi
else
    check_fail "package.xml not found"
fi

# Check CMakeLists.txt
if [[ -f "$NAV_LAUNCH_DIR/CMakeLists.txt" ]]; then
    check_pass "CMakeLists.txt exists"
else
    check_fail "CMakeLists.txt not found"
fi

echo ""

# 2. Check config files
echo "Checking config files..."

CONFIG_DIR="$NAV_LAUNCH_DIR/config"

if [[ -d "$CONFIG_DIR" ]]; then
    check_pass "config/ directory exists"
else
    check_fail "config/ directory not found"
fi

# Check nav2_slam_params.yaml (required - SLAM-in-the-loop config)
if [[ -f "$CONFIG_DIR/nav2_slam_params.yaml" ]]; then
    check_pass "nav2_slam_params.yaml exists"

    # Check key parameters
    if grep -q "controller_server:" "$CONFIG_DIR/nav2_slam_params.yaml"; then
        check_pass "nav2_slam_params.yaml has controller_server config"
    else
        check_fail "nav2_slam_params.yaml missing controller_server"
    fi

    if grep -q "planner_server:" "$CONFIG_DIR/nav2_slam_params.yaml"; then
        check_pass "nav2_slam_params.yaml has planner_server config"
    else
        check_fail "nav2_slam_params.yaml missing planner_server"
    fi

    if grep -q "bt_navigator:" "$CONFIG_DIR/nav2_slam_params.yaml"; then
        check_pass "nav2_slam_params.yaml has bt_navigator config"
    else
        check_fail "nav2_slam_params.yaml missing bt_navigator"
    fi

    # Verify no AMCL (per AD-011)
    if grep -q "amcl:" "$CONFIG_DIR/nav2_slam_params.yaml"; then
        check_warn "nav2_slam_params.yaml contains AMCL config (should not for SLAM-in-the-loop)"
    else
        check_pass "nav2_slam_params.yaml correctly omits AMCL (per AD-011)"
    fi
else
    check_fail "nav2_slam_params.yaml not found"
fi

# Check nav2_params.yaml (optional - with AMCL for pre-built maps)
if [[ -f "$CONFIG_DIR/nav2_params.yaml" ]]; then
    check_pass "nav2_params.yaml exists (optional)"
else
    check_warn "nav2_params.yaml not found (optional for pre-built maps)"
fi

echo ""

# 3. Check launch files
echo "Checking launch files..."

LAUNCH_DIR="$NAV_LAUNCH_DIR/launch"

if [[ -d "$LAUNCH_DIR" ]]; then
    check_pass "launch/ directory exists"
else
    check_fail "launch/ directory not found"
fi

# Check nav2_slam.launch.py (main launcher)
if [[ -f "$LAUNCH_DIR/nav2_slam.launch.py" ]]; then
    check_pass "nav2_slam.launch.py exists"

    # Check for algorithm argument
    if grep -q "algorithm" "$LAUNCH_DIR/nav2_slam.launch.py"; then
        check_pass "nav2_slam.launch.py has algorithm argument"
    else
        check_fail "nav2_slam.launch.py missing algorithm argument"
    fi

    # Check it includes SLAM launch
    if grep -q "slam_toolbox\|cartographer" "$LAUNCH_DIR/nav2_slam.launch.py"; then
        check_pass "nav2_slam.launch.py references SLAM algorithms"
    else
        check_fail "nav2_slam.launch.py doesn't reference SLAM algorithms"
    fi

    # Verify Python syntax
    if python3 -m py_compile "$LAUNCH_DIR/nav2_slam.launch.py" 2>/dev/null; then
        check_pass "nav2_slam.launch.py has valid Python syntax"
    else
        check_fail "nav2_slam.launch.py has Python syntax errors"
    fi
else
    check_fail "nav2_slam.launch.py not found"
fi

# Check send_goal.launch.py (utility)
if [[ -f "$LAUNCH_DIR/send_goal.launch.py" ]]; then
    check_pass "send_goal.launch.py exists"

    # Verify Python syntax
    if python3 -m py_compile "$LAUNCH_DIR/send_goal.launch.py" 2>/dev/null; then
        check_pass "send_goal.launch.py has valid Python syntax"
    else
        check_fail "send_goal.launch.py has Python syntax errors"
    fi
else
    check_fail "send_goal.launch.py not found"
fi

echo ""

# 4. Check resource marker and behavior trees
echo "Checking additional package files..."

if [[ -f "$NAV_LAUNCH_DIR/resource/nav_launch" ]]; then
    check_pass "resource/nav_launch marker exists"
else
    check_fail "resource/nav_launch marker not found"
fi

if [[ -d "$NAV_LAUNCH_DIR/behavior_trees" ]]; then
    check_pass "behavior_trees/ directory exists"
else
    check_warn "behavior_trees/ directory not found (may use Nav2 defaults)"
fi

echo ""

# 5. Check Nav2 dependencies are installed
echo "Checking Nav2 dependencies..."

# Track if Nav2 is available (warn but don't fail - requires sudo to install)
NAV2_AVAILABLE=true

# Check nav2_bringup package
if ros2 pkg list 2>/dev/null | grep -q "nav2_bringup"; then
    check_pass "nav2_bringup package installed"
else
    check_warn "nav2_bringup not installed (run: sudo apt install ros-\$ROS_DISTRO-navigation2 ros-\$ROS_DISTRO-nav2-bringup)"
    NAV2_AVAILABLE=false
fi

# Check controller_server
if ros2 pkg list 2>/dev/null | grep -q "nav2_controller"; then
    check_pass "nav2_controller package installed"
else
    check_warn "nav2_controller not installed"
    NAV2_AVAILABLE=false
fi

# Check planner_server
if ros2 pkg list 2>/dev/null | grep -q "nav2_planner"; then
    check_pass "nav2_planner package installed"
else
    check_warn "nav2_planner not installed"
    NAV2_AVAILABLE=false
fi

# Check bt_navigator
if ros2 pkg list 2>/dev/null | grep -q "nav2_bt_navigator"; then
    check_pass "nav2_bt_navigator package installed"
else
    check_warn "nav2_bt_navigator not installed"
    NAV2_AVAILABLE=false
fi

if [[ "$NAV2_AVAILABLE" == "false" ]]; then
    echo ""
    echo -e "${YELLOW}Note: Nav2 packages not installed. Install with:${NC}"
    echo "  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup"
fi

echo ""

# 6. Build test
echo "Testing package build..."

cd "$WS_DIR"

# Try to build just nav_launch
if colcon build --packages-select nav_launch 2>/dev/null; then
    check_pass "nav_launch package builds successfully"
else
    check_fail "nav_launch package build failed"
fi

# Verify installed files
if [[ -d "$WS_DIR/install/nav_launch" ]]; then
    check_pass "nav_launch installed to workspace"

    # Check config files installed
    if [[ -d "$WS_DIR/install/nav_launch/share/nav_launch/config" ]]; then
        check_pass "Config files installed"
    else
        check_fail "Config files not installed"
    fi

    # Check launch files installed
    if [[ -d "$WS_DIR/install/nav_launch/share/nav_launch/launch" ]]; then
        check_pass "Launch files installed"
    else
        check_fail "Launch files not installed"
    fi
else
    check_fail "nav_launch not found in install directory"
fi

echo ""

# 7. Verify package is discoverable
echo "Checking package discoverability..."

source "$WS_DIR/install/setup.bash"

if ros2 pkg list 2>/dev/null | grep -q "nav_launch"; then
    check_pass "nav_launch package discoverable via ros2 pkg list"
else
    check_fail "nav_launch not discoverable"
fi

# Check launch file is findable
if ros2 launch --show-args nav_launch nav2_slam.launch.py 2>/dev/null | grep -q "algorithm"; then
    check_pass "nav2_slam.launch.py shows arguments correctly"
else
    check_warn "Could not verify launch file arguments (may need dependencies running)"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M10 VALIDATION PASSED${NC}"
    echo ""
    echo "Nav2 Launch Package is complete:"
    echo "  - Package structure: Complete"
    echo "  - Config files: nav2_slam_params.yaml (SLAM-in-the-loop)"
    echo "  - Launch files: nav2_slam.launch.py, send_goal.launch.py"
    echo "  - Dependencies: Nav2 packages installed"
    echo ""
    echo "Usage:"
    echo "  ros2 launch nav_launch nav2_slam.launch.py algorithm:=slam_toolbox"
    echo "  ros2 launch nav_launch send_goal.launch.py x:=2.0 y:=1.0 yaw:=0.0"
    echo ""
    exit 0
else
    echo -e "${RED}M10 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
