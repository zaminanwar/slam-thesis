#!/bin/bash
#
# M3 Validation Script - Ground Truth Publisher
#
# Validates:
# 1. gt_publisher package files exist
# 2. gt_publisher package builds
# 3. gt_publisher_node.py is valid Python
# 4. gt_publisher.launch.py is valid
# 5. pose bridge added to sim.launch.py
# 6. Required imports work
#
# Note: Full ground truth testing requires Gazebo running

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
echo "M3 Validation - Ground Truth Publisher"
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

# 1. Check gt_publisher package files
echo "Checking gt_publisher package files..."

GT_FILES=(
    "$WS_DIR/src/slam_thesis/gt_publisher/gt_publisher/gt_publisher_node.py"
    "$WS_DIR/src/slam_thesis/gt_publisher/launch/gt_publisher.launch.py"
    "$WS_DIR/src/slam_thesis/gt_publisher/package.xml"
    "$WS_DIR/src/slam_thesis/gt_publisher/setup.py"
)

for f in "${GT_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        check_pass "$(basename "$f") exists"
    else
        check_fail "$(basename "$f") missing: $f"
    fi
done

echo ""

# 2. Check pose bridge in sim.launch.py
echo "Checking pose bridge in sim.launch.py..."

SIM_LAUNCH="$WS_DIR/src/slam_thesis/rover_sim/launch/sim.launch.py"
if grep -q "/model/rover/pose" "$SIM_LAUNCH"; then
    check_pass "Pose bridge found in sim.launch.py"
else
    check_fail "Pose bridge missing in sim.launch.py"
fi

echo ""

# 3. Source ROS and build
echo "Building gt_publisher package..."

# Detect ROS distro
if [[ -f "$THESIS_DIR/.ros_distro" ]]; then
    ROS_DISTRO=$(cat "$THESIS_DIR/.ros_distro")
else
    ROS_DISTRO="jazzy"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_DIR"

if colcon build --packages-select gt_publisher 2>&1 | tee /tmp/m3_build.log | tail -10; then
    if grep -q "Finished <<< gt_publisher" /tmp/m3_build.log; then
        check_pass "gt_publisher built successfully"
    elif grep -q "packages finished" /tmp/m3_build.log; then
        check_pass "gt_publisher built successfully"
    else
        check_fail "gt_publisher build may have issues"
    fi
else
    check_fail "gt_publisher build failed"
fi

echo ""

# 4. Validate Python syntax
echo "Validating Python syntax..."

source "$WS_DIR/install/setup.bash"

if python3 -m py_compile "$WS_DIR/src/slam_thesis/gt_publisher/gt_publisher/gt_publisher_node.py" 2>/tmp/py_check.log; then
    check_pass "gt_publisher_node.py syntax OK"
else
    check_fail "gt_publisher_node.py has syntax errors"
    cat /tmp/py_check.log
fi

echo ""

# 5. Validate launch file
echo "Validating launch files..."

if python3 -c "
import sys
import os
sys.path.insert(0, '$WS_DIR/install/gt_publisher/share/gt_publisher/launch')
os.environ['HOME'] = os.path.expanduser('~')
from launch import LaunchDescription
exec(open('$WS_DIR/src/slam_thesis/gt_publisher/launch/gt_publisher.launch.py').read())
ld = generate_launch_description()
assert isinstance(ld, LaunchDescription), 'Not a LaunchDescription'
" 2>/tmp/launch_check.log; then
    check_pass "gt_publisher.launch.py is valid"
else
    check_fail "gt_publisher.launch.py has errors"
    cat /tmp/launch_check.log
fi

echo ""

# 6. Check imports work
echo "Checking Python imports..."

if python3 -c "
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
" 2>/tmp/import_check.log; then
    check_pass "All ROS2 imports work"
else
    check_fail "ROS2 import errors"
    cat /tmp/import_check.log
fi

echo ""

# 7. Check node implementation has required elements
echo "Checking node implementation..."

NODE_FILE="$WS_DIR/src/slam_thesis/gt_publisher/gt_publisher/gt_publisher_node.py"

if grep -q "map_gt" "$NODE_FILE"; then
    check_pass "Node uses map_gt frame"
else
    check_fail "Node does not use map_gt frame"
fi

if grep -q "TransformBroadcaster" "$NODE_FILE"; then
    check_pass "Node uses TF broadcaster"
else
    check_fail "Node does not use TF broadcaster"
fi

if grep -q "/gt_pose" "$NODE_FILE"; then
    check_pass "Node publishes to /gt_pose"
else
    check_fail "Node does not publish to /gt_pose"
fi

if grep -q "publish_rate" "$NODE_FILE"; then
    check_pass "Node has publish_rate parameter"
else
    check_fail "Node missing publish_rate parameter"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M3 VALIDATION PASSED${NC}"
    echo ""
    echo "Next steps for manual verification:"
    echo "  1. Start simulation:"
    echo "     ros2 launch rover_sim sim.launch.py"
    echo ""
    echo "  2. In another terminal, start gt_publisher:"
    echo "     source ~/thesis/ros2_ws/install/setup.bash"
    echo "     ros2 launch gt_publisher gt_publisher.launch.py"
    echo ""
    echo "  3. Verify /gt_pose is publishing:"
    echo "     ros2 topic echo /gt_pose --once"
    echo ""
    echo "  4. Verify TF map_gt->base_link:"
    echo "     ros2 run tf2_ros tf2_echo map_gt base_link"
    echo ""
    echo "  5. Move the robot and verify pose updates:"
    echo "     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once"
    echo ""
    exit 0
else
    echo -e "${RED}M3 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
