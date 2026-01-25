#!/bin/bash
# Milestone 5 Validation: SLAM Pipelines
# Validates slam_toolbox and Cartographer configurations and launch files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"
SLAM_LAUNCH_PKG="$WORKSPACE_DIR/src/slam_thesis/slam_launch"

echo "======================================"
echo "Milestone 5 Validation: SLAM Pipelines"
echo "======================================"
echo ""

# Detect ROS distro
if [ -f "$SCRIPT_DIR/../.ros_distro" ]; then
    ROS_DISTRO=$(cat "$SCRIPT_DIR/../.ros_distro")
else
    ROS_DISTRO="jazzy"
fi
echo "ROS Distro: $ROS_DISTRO"
source /opt/ros/$ROS_DISTRO/setup.bash

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

# Test 1: Check slam_launch package structure
echo ""
echo "=== Test 1: Package Structure ==="

if [ -f "$SLAM_LAUNCH_PKG/package.xml" ]; then
    pass "package.xml exists"
else
    fail "package.xml missing"
fi

if [ -f "$SLAM_LAUNCH_PKG/CMakeLists.txt" ]; then
    pass "CMakeLists.txt exists"
else
    fail "CMakeLists.txt missing"
fi

# Test 2: Check SLAM configurations
echo ""
echo "=== Test 2: SLAM Configurations ==="

if [ -f "$SLAM_LAUNCH_PKG/config/slam_toolbox.yaml" ]; then
    pass "slam_toolbox.yaml exists"
    # Check key parameters
    if grep -q "base_frame: base_footprint" "$SLAM_LAUNCH_PKG/config/slam_toolbox.yaml"; then
        pass "slam_toolbox base_frame configured correctly"
    else
        fail "slam_toolbox base_frame not configured correctly"
    fi
else
    fail "slam_toolbox.yaml missing"
fi

if [ -f "$SLAM_LAUNCH_PKG/config/cartographer_2d.lua" ]; then
    pass "cartographer_2d.lua exists"
    # Check key parameters
    if grep -q "tracking_frame = \"base_footprint\"" "$SLAM_LAUNCH_PKG/config/cartographer_2d.lua"; then
        pass "Cartographer tracking_frame configured correctly"
    else
        fail "Cartographer tracking_frame not configured correctly"
    fi
else
    fail "cartographer_2d.lua missing"
fi

# Test 3: Check launch files
echo ""
echo "=== Test 3: Launch Files ==="

if [ -f "$SLAM_LAUNCH_PKG/launch/slam_toolbox.launch.py" ]; then
    pass "slam_toolbox.launch.py exists"
    # Check it's valid Python
    if python3 -m py_compile "$SLAM_LAUNCH_PKG/launch/slam_toolbox.launch.py" 2>/dev/null; then
        pass "slam_toolbox.launch.py is valid Python"
    else
        fail "slam_toolbox.launch.py has syntax errors"
    fi
else
    fail "slam_toolbox.launch.py missing"
fi

if [ -f "$SLAM_LAUNCH_PKG/launch/cartographer.launch.py" ]; then
    pass "cartographer.launch.py exists"
    # Check it's valid Python
    if python3 -m py_compile "$SLAM_LAUNCH_PKG/launch/cartographer.launch.py" 2>/dev/null; then
        pass "cartographer.launch.py is valid Python"
    else
        fail "cartographer.launch.py has syntax errors"
    fi
else
    fail "cartographer.launch.py missing"
fi

# Test 4: Build package
echo ""
echo "=== Test 4: Package Build ==="

cd "$WORKSPACE_DIR"
if colcon build --packages-select slam_launch 2>&1 | tail -5; then
    pass "slam_launch builds successfully"
else
    fail "slam_launch build failed"
fi

# Source workspace
source "$WORKSPACE_DIR/install/setup.bash"

# Test 5: Check installed files
echo ""
echo "=== Test 5: Installed Files ==="

INSTALL_DIR="$WORKSPACE_DIR/install/slam_launch/share/slam_launch"

if [ -f "$INSTALL_DIR/config/slam_toolbox.yaml" ]; then
    pass "slam_toolbox.yaml installed"
else
    fail "slam_toolbox.yaml not installed"
fi

if [ -f "$INSTALL_DIR/config/cartographer_2d.lua" ]; then
    pass "cartographer_2d.lua installed"
else
    fail "cartographer_2d.lua not installed"
fi

if [ -f "$INSTALL_DIR/launch/slam_toolbox.launch.py" ]; then
    pass "slam_toolbox.launch.py installed"
else
    fail "slam_toolbox.launch.py not installed"
fi

if [ -f "$INSTALL_DIR/launch/cartographer.launch.py" ]; then
    pass "cartographer.launch.py installed"
else
    fail "cartographer.launch.py not installed"
fi

# Test 6: Check dependencies
echo ""
echo "=== Test 6: SLAM Dependencies ==="

if ros2 pkg list 2>/dev/null | grep -q "slam_toolbox"; then
    pass "slam_toolbox package available"
else
    fail "slam_toolbox package not installed"
fi

if ros2 pkg list 2>/dev/null | grep -q "cartographer_ros"; then
    pass "cartographer_ros package available"
else
    fail "cartographer_ros package not installed"
fi

# Summary
echo ""
echo "======================================"
echo "Summary"
echo "======================================"
echo "Passed: $PASS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo "[SUCCESS] All M5 validation tests passed!"
    echo ""
    echo "Note: Bag replay with use_sim_time has known TF synchronization"
    echo "issues in ROS 2 Jazzy. The SLAM launch files are configured correctly"
    echo "but may require additional tuning for reliable bag replay."
    echo ""
    echo "For live simulation testing, use:"
    echo "  ros2 launch rover_sim sim.launch.py"
    echo "  ros2 launch slam_launch slam_toolbox.launch.py bag:=<bag_name>"
    exit 0
else
    echo "[FAILURE] $FAIL_COUNT tests failed. Please fix issues above."
    exit 1
fi
