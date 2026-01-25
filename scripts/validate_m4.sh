#!/bin/bash
#
# M4 Validation Script - Dataset Recording
#
# Validates:
# 1. experiment_runner package files exist
# 2. experiment_runner package builds
# 3. Launch files are valid Python
# 4. odom_noise_node.py is valid Python
# 5. topics_to_record.yaml is valid YAML
# 6. Required imports work
# 7. bags directory exists
#
# Note: Full recording testing requires Gazebo running

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
echo "M4 Validation - Dataset Recording"
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

# 1. Check experiment_runner package files
echo "Checking experiment_runner package files..."

EXP_FILES=(
    "$WS_DIR/src/slam_thesis/experiment_runner/launch/record_dataset.launch.py"
    "$WS_DIR/src/slam_thesis/experiment_runner/launch/replay_bag.launch.py"
    "$WS_DIR/src/slam_thesis/experiment_runner/config/topics_to_record.yaml"
    "$WS_DIR/src/slam_thesis/experiment_runner/experiment_runner/odom_noise_node.py"
    "$WS_DIR/src/slam_thesis/experiment_runner/package.xml"
    "$WS_DIR/src/slam_thesis/experiment_runner/setup.py"
)

for f in "${EXP_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        check_pass "$(basename "$f") exists"
    else
        check_fail "$(basename "$f") missing: $f"
    fi
done

echo ""

# 2. Check bags directory
echo "Checking bags directory..."

BAGS_DIR="$WS_DIR/bags"
if [[ -d "$BAGS_DIR" ]]; then
    check_pass "bags directory exists: $BAGS_DIR"
else
    mkdir -p "$BAGS_DIR"
    check_warn "Created bags directory: $BAGS_DIR"
fi

echo ""

# 3. Source ROS and build
echo "Building experiment_runner package..."

# Detect ROS distro
if [[ -f "$THESIS_DIR/.ros_distro" ]]; then
    ROS_DISTRO=$(cat "$THESIS_DIR/.ros_distro")
else
    ROS_DISTRO="jazzy"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_DIR"

if colcon build --packages-select experiment_runner 2>&1 | tee /tmp/m4_build.log | tail -10; then
    if grep -q "Finished <<< experiment_runner" /tmp/m4_build.log; then
        check_pass "experiment_runner built successfully"
    elif grep -q "packages finished" /tmp/m4_build.log; then
        check_pass "experiment_runner built successfully"
    else
        check_fail "experiment_runner build may have issues"
    fi
else
    check_fail "experiment_runner build failed"
fi

echo ""

# 4. Validate Python syntax
echo "Validating Python syntax..."

source "$WS_DIR/install/setup.bash"

if python3 -m py_compile "$WS_DIR/src/slam_thesis/experiment_runner/experiment_runner/odom_noise_node.py" 2>/tmp/py_check.log; then
    check_pass "odom_noise_node.py syntax OK"
else
    check_fail "odom_noise_node.py has syntax errors"
    cat /tmp/py_check.log
fi

echo ""

# 5. Validate YAML config
echo "Validating topics_to_record.yaml..."

if python3 -c "
import yaml
with open('$WS_DIR/src/slam_thesis/experiment_runner/config/topics_to_record.yaml') as f:
    config = yaml.safe_load(f)
    assert 'topics' in config, 'Missing topics key'
    assert '/scan' in config['topics'], 'Missing /scan topic'
    assert '/odom' in config['topics'], 'Missing /odom topic'
    assert '/gt_pose' in config['topics'], 'Missing /gt_pose topic'
    assert '/clock' in config['topics'], 'Missing /clock topic'
    assert '/tf' in config['topics'], 'Missing /tf topic'
print('Topics found:', len(config['topics']))
" 2>/tmp/yaml_check.log; then
    check_pass "topics_to_record.yaml is valid"
else
    check_fail "topics_to_record.yaml has errors"
    cat /tmp/yaml_check.log
fi

echo ""

# 6. Validate launch files
echo "Validating launch files..."

for launch_file in record_dataset.launch.py replay_bag.launch.py; do
    LAUNCH_PATH="$WS_DIR/src/slam_thesis/experiment_runner/launch/$launch_file"
    if python3 -c "
import sys
import os
os.environ['HOME'] = os.path.expanduser('~')
# Create dummy trajectory for record launch
if 'record' in '$launch_file':
    import tempfile
    os.makedirs('/tmp/trajectories', exist_ok=True)
    with open('/tmp/trajectories/test.csv', 'w') as f:
        f.write('x,y,yaw,speed\n0,0,0,0.5\n')
from launch import LaunchDescription
exec(open('$LAUNCH_PATH').read())
ld = generate_launch_description()
assert isinstance(ld, LaunchDescription), 'Not a LaunchDescription'
" 2>/tmp/launch_check.log; then
        check_pass "$launch_file is valid"
    else
        check_fail "$launch_file has errors"
        cat /tmp/launch_check.log
    fi
done

echo ""

# 7. Check imports work
echo "Checking Python imports..."

if python3 -c "
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
" 2>/tmp/import_check.log; then
    check_pass "Core ROS2 imports work"
else
    check_fail "ROS2 import errors"
    cat /tmp/import_check.log
fi

# Check tf_transformations (may not be available)
if python3 -c "import tf_transformations" 2>/dev/null; then
    check_pass "tf_transformations available"
else
    check_warn "tf_transformations not installed (optional)"
fi

echo ""

# 8. Check odom_noise_node has required elements
echo "Checking odom_noise_node implementation..."

NODE_FILE="$WS_DIR/src/slam_thesis/experiment_runner/experiment_runner/odom_noise_node.py"

if grep -q "noise_std" "$NODE_FILE"; then
    check_pass "Node has noise_std parameter"
else
    check_fail "Node missing noise_std parameter"
fi

if grep -q "drift_rate" "$NODE_FILE"; then
    check_pass "Node has drift_rate parameter"
else
    check_fail "Node missing drift_rate parameter"
fi

if grep -q "odom_in" "$NODE_FILE"; then
    check_pass "Node subscribes to odom_in"
else
    check_fail "Node missing odom_in subscription"
fi

if grep -q "odom_out" "$NODE_FILE"; then
    check_pass "Node publishes to odom_out"
else
    check_fail "Node missing odom_out publisher"
fi

echo ""

# 9. Check record_dataset.launch.py has required elements
echo "Checking record_dataset.launch.py implementation..."

RECORD_LAUNCH="$WS_DIR/src/slam_thesis/experiment_runner/launch/record_dataset.launch.py"

if grep -q "trajectory" "$RECORD_LAUNCH"; then
    check_pass "Launch has trajectory parameter"
else
    check_fail "Launch missing trajectory parameter"
fi

if grep -q "condition" "$RECORD_LAUNCH"; then
    check_pass "Launch has condition parameter"
else
    check_fail "Launch missing condition parameter"
fi

if grep -q "baseline" "$RECORD_LAUNCH"; then
    check_pass "Launch supports baseline condition"
else
    check_fail "Launch missing baseline condition"
fi

if grep -q "odom_degraded" "$RECORD_LAUNCH"; then
    check_pass "Launch supports odom_degraded condition"
else
    check_fail "Launch missing odom_degraded condition"
fi

if grep -q "high_speed" "$RECORD_LAUNCH"; then
    check_pass "Launch supports high_speed condition"
else
    check_fail "Launch missing high_speed condition"
fi

if grep -q "ros2.*bag.*record" "$RECORD_LAUNCH"; then
    check_pass "Launch includes rosbag recording"
else
    check_fail "Launch missing rosbag recording"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M4 VALIDATION PASSED${NC}"
    echo ""
    echo "Next steps for manual verification:"
    echo "  1. Start recording (in terminal 1):"
    echo "     source ~/thesis/ros2_ws/install/setup.bash"
    echo "     ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv condition:=baseline"
    echo ""
    echo "  2. Wait for trajectory to complete (robot will move and stop)"
    echo ""
    echo "  3. Ctrl+C to stop recording"
    echo ""
    echo "  4. Verify bag was created:"
    echo "     ls -la ~/thesis/ros2_ws/bags/traj_01_easy__baseline/"
    echo "     ros2 bag info ~/thesis/ros2_ws/bags/traj_01_easy__baseline"
    echo ""
    echo "  5. Test replay:"
    echo "     ros2 launch experiment_runner replay_bag.launch.py bag:=traj_01_easy__baseline"
    echo ""
    echo "  6. Verify topics are replaying:"
    echo "     ros2 topic list"
    echo "     ros2 topic echo /scan --once"
    echo ""
    exit 0
else
    echo -e "${RED}M4 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
