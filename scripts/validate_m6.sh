#!/bin/bash
# Milestone 6 Validation: Trajectory Export
# Validates trajectory exporter node and TUM format output

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"
TRAJ_EXPORTER_PKG="$WORKSPACE_DIR/src/slam_thesis/traj_exporter"
SLAM_LAUNCH_PKG="$WORKSPACE_DIR/src/slam_thesis/slam_launch"
GT_PUBLISHER_PKG="$WORKSPACE_DIR/src/slam_thesis/gt_publisher"

echo "=========================================="
echo "Milestone 6 Validation: Trajectory Export"
echo "=========================================="
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

# Test 1: Check traj_exporter package structure
echo ""
echo "=== Test 1: Package Structure ==="

if [ -f "$TRAJ_EXPORTER_PKG/package.xml" ]; then
    pass "traj_exporter package.xml exists"
else
    fail "traj_exporter package.xml missing"
fi

if [ -f "$TRAJ_EXPORTER_PKG/setup.py" ]; then
    pass "traj_exporter setup.py exists"
else
    fail "traj_exporter setup.py missing"
fi

if [ -f "$TRAJ_EXPORTER_PKG/traj_exporter/traj_exporter_node.py" ]; then
    pass "traj_exporter_node.py exists"
else
    fail "traj_exporter_node.py missing"
fi

# Test 2: Check trajectory exporter node implementation
echo ""
echo "=== Test 2: Node Implementation ==="

if python3 -m py_compile "$TRAJ_EXPORTER_PKG/traj_exporter/traj_exporter_node.py" 2>/dev/null; then
    pass "traj_exporter_node.py is valid Python"
else
    fail "traj_exporter_node.py has syntax errors"
fi

# Check for key functionality
if grep -q "class TrajectoryExporter" "$TRAJ_EXPORTER_PKG/traj_exporter/traj_exporter_node.py"; then
    pass "TrajectoryExporter class defined"
else
    fail "TrajectoryExporter class not found"
fi

if grep -q "lookup_transform" "$TRAJ_EXPORTER_PKG/traj_exporter/traj_exporter_node.py"; then
    pass "TF lookup implemented"
else
    fail "TF lookup not implemented"
fi

if grep -q "timestamp tx ty tz qx qy qz qw" "$TRAJ_EXPORTER_PKG/traj_exporter/traj_exporter_node.py"; then
    pass "TUM format header present"
else
    fail "TUM format header not found"
fi

# Test 3: Check launch files
echo ""
echo "=== Test 3: Launch Files ==="

if [ -f "$TRAJ_EXPORTER_PKG/launch/traj_exporter.launch.py" ]; then
    pass "traj_exporter.launch.py exists"
    if python3 -m py_compile "$TRAJ_EXPORTER_PKG/launch/traj_exporter.launch.py" 2>/dev/null; then
        pass "traj_exporter.launch.py is valid Python"
    else
        fail "traj_exporter.launch.py has syntax errors"
    fi
else
    fail "traj_exporter.launch.py missing"
fi

if [ -f "$SLAM_LAUNCH_PKG/launch/slam_toolbox_eval.launch.py" ]; then
    pass "slam_toolbox_eval.launch.py exists"
    if python3 -m py_compile "$SLAM_LAUNCH_PKG/launch/slam_toolbox_eval.launch.py" 2>/dev/null; then
        pass "slam_toolbox_eval.launch.py is valid Python"
    else
        fail "slam_toolbox_eval.launch.py has syntax errors"
    fi
else
    fail "slam_toolbox_eval.launch.py missing"
fi

if [ -f "$SLAM_LAUNCH_PKG/launch/cartographer_eval.launch.py" ]; then
    pass "cartographer_eval.launch.py exists"
    if python3 -m py_compile "$SLAM_LAUNCH_PKG/launch/cartographer_eval.launch.py" 2>/dev/null; then
        pass "cartographer_eval.launch.py is valid Python"
    else
        fail "cartographer_eval.launch.py has syntax errors"
    fi
else
    fail "cartographer_eval.launch.py missing"
fi

# Test 4: Check gt_publisher uses base_footprint
echo ""
echo "=== Test 4: Frame Configuration ==="

if grep -q "child_frame_id = 'base_footprint'" "$GT_PUBLISHER_PKG/gt_publisher/gt_publisher_node.py"; then
    pass "gt_publisher uses base_footprint frame"
else
    fail "gt_publisher should use base_footprint (not base_link)"
fi

if grep -q "map_gt" "$GT_PUBLISHER_PKG/gt_publisher/gt_publisher_node.py"; then
    pass "gt_publisher uses map_gt parent frame"
else
    fail "gt_publisher should use map_gt parent frame"
fi

# Test 5: Build packages
echo ""
echo "=== Test 5: Package Build ==="

cd "$WORKSPACE_DIR"
if colcon build --packages-select traj_exporter gt_publisher slam_launch 2>&1 | tail -5; then
    pass "All packages build successfully"
else
    fail "Package build failed"
fi

# Source workspace
source "$WORKSPACE_DIR/install/setup.bash"

# Test 6: Check installed files
echo ""
echo "=== Test 6: Installed Files ==="

TRAJ_INSTALL_DIR="$WORKSPACE_DIR/install/traj_exporter"
SLAM_INSTALL_DIR="$WORKSPACE_DIR/install/slam_launch/share/slam_launch"

if [ -f "$TRAJ_INSTALL_DIR/lib/traj_exporter/traj_exporter" ]; then
    pass "traj_exporter executable installed"
else
    fail "traj_exporter executable not installed"
fi

if [ -f "$TRAJ_INSTALL_DIR/share/traj_exporter/launch/traj_exporter.launch.py" ]; then
    pass "traj_exporter.launch.py installed"
else
    fail "traj_exporter.launch.py not installed"
fi

if [ -f "$SLAM_INSTALL_DIR/launch/slam_toolbox_eval.launch.py" ]; then
    pass "slam_toolbox_eval.launch.py installed"
else
    fail "slam_toolbox_eval.launch.py not installed"
fi

if [ -f "$SLAM_INSTALL_DIR/launch/cartographer_eval.launch.py" ]; then
    pass "cartographer_eval.launch.py installed"
else
    fail "cartographer_eval.launch.py not installed"
fi

# Test 7: Check dependencies
echo ""
echo "=== Test 7: Dependencies ==="

if ros2 pkg list 2>/dev/null | grep -q "tf2_ros"; then
    pass "tf2_ros package available"
else
    fail "tf2_ros package not available"
fi

if ros2 pkg list 2>/dev/null | grep -q "traj_exporter"; then
    pass "traj_exporter package registered"
else
    fail "traj_exporter package not registered"
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
    echo "[SUCCESS] All M6 validation tests passed!"
    echo ""
    echo "Trajectory export is ready. Usage:"
    echo ""
    echo "  # 1. Start simulation with ground truth:"
    echo "  ros2 launch rover_sim sim.launch.py"
    echo "  ros2 launch gt_publisher gt_publisher.launch.py"
    echo ""
    echo "  # 2. Start SLAM with trajectory export:"
    echo "  ros2 launch slam_launch slam_toolbox_eval.launch.py output_dir:=~/thesis/ros2_ws/results/test"
    echo "  # OR"
    echo "  ros2 launch slam_launch cartographer_eval.launch.py output_dir:=~/thesis/ros2_ws/results/test"
    echo ""
    echo "  # 3. (Optional) Follow a trajectory:"
    echo "  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy"
    echo ""
    echo "  # 4. When done, Ctrl+C to save trajectory files (gt.tum, est.tum)"
    echo ""
    exit 0
else
    echo "[FAILURE] $FAIL_COUNT tests failed. Please fix issues above."
    exit 1
fi
