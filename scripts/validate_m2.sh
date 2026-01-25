#!/bin/bash
#
# M2 Validation Script - Control (Trajectory Following)
#
# Validates:
# 1. Trajectory CSV files exist and have correct format
# 2. rover_control package builds
# 3. trajectory_follower.py is valid Python
# 4. follow_trajectory.launch.py is valid
# 5. Required topics can be checked manually
#
# Note: Full trajectory testing requires Gazebo running

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THESIS_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$THESIS_DIR/ros2_ws"
TRAJ_DIR="$THESIS_DIR/trajectories"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "M2 Validation - Control (Trajectory)"
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

# 1. Check trajectory files exist
echo "Checking trajectory files..."

TRAJ_FILES=(
    "$TRAJ_DIR/traj_01_easy.csv"
    "$TRAJ_DIR/traj_02_loop.csv"
    "$TRAJ_DIR/traj_03_complex.csv"
)

for f in "${TRAJ_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        check_pass "$(basename "$f") exists"
    else
        check_fail "$(basename "$f") missing: $f"
    fi
done

echo ""

# 2. Validate trajectory CSV format
echo "Validating trajectory CSV format..."

validate_csv() {
    local file="$1"
    local name="$(basename "$file")"

    # Check header
    header=$(head -1 "$file")
    if [[ "$header" == "x,y,yaw,speed" ]]; then
        check_pass "$name has correct header"
    else
        check_fail "$name has wrong header: $header (expected: x,y,yaw,speed)"
        return
    fi

    # Check at least one data row
    line_count=$(wc -l < "$file")
    if [[ $line_count -ge 2 ]]; then
        check_pass "$name has $(($line_count - 1)) waypoints"
    else
        check_fail "$name has no waypoints"
        return
    fi

    # Validate data format (check second line)
    second_line=$(sed -n '2p' "$file")
    if echo "$second_line" | grep -qE '^-?[0-9]+\.?[0-9]*,-?[0-9]+\.?[0-9]*,-?[0-9]+\.?[0-9]*,-?[0-9]+\.?[0-9]*$'; then
        check_pass "$name has valid data format"
    else
        check_fail "$name has invalid data format: $second_line"
    fi
}

for f in "${TRAJ_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        validate_csv "$f"
    fi
done

echo ""

# 3. Check rover_control package files
echo "Checking rover_control package files..."

CONTROL_FILES=(
    "$WS_DIR/src/slam_thesis/rover_control/rover_control/trajectory_follower.py"
    "$WS_DIR/src/slam_thesis/rover_control/launch/follow_trajectory.launch.py"
    "$WS_DIR/src/slam_thesis/rover_control/package.xml"
    "$WS_DIR/src/slam_thesis/rover_control/setup.py"
)

for f in "${CONTROL_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        check_pass "$(basename "$f") exists"
    else
        check_fail "$(basename "$f") missing: $f"
    fi
done

echo ""

# 4. Source ROS and build
echo "Building rover_control package..."

# Detect ROS distro
if [[ -f "$THESIS_DIR/.ros_distro" ]]; then
    ROS_DISTRO=$(cat "$THESIS_DIR/.ros_distro")
else
    ROS_DISTRO="jazzy"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_DIR"

if colcon build --packages-select rover_control 2>&1 | tee /tmp/m2_build.log | tail -10; then
    if grep -q "Finished <<< rover_control" /tmp/m2_build.log; then
        check_pass "rover_control built successfully"
    elif grep -q "packages finished" /tmp/m2_build.log; then
        check_pass "rover_control built successfully"
    else
        check_fail "rover_control build may have issues"
    fi
else
    check_fail "rover_control build failed"
fi

echo ""

# 5. Validate Python syntax
echo "Validating Python syntax..."

source "$WS_DIR/install/setup.bash"

if python3 -m py_compile "$WS_DIR/src/slam_thesis/rover_control/rover_control/trajectory_follower.py" 2>/tmp/py_check.log; then
    check_pass "trajectory_follower.py syntax OK"
else
    check_fail "trajectory_follower.py has syntax errors"
    cat /tmp/py_check.log
fi

echo ""

# 6. Validate launch file
echo "Validating launch files..."

if python3 -c "
import sys
import os
sys.path.insert(0, '$WS_DIR/install/rover_control/share/rover_control/launch')
os.environ['HOME'] = os.path.expanduser('~')
from launch import LaunchDescription
exec(open('$WS_DIR/src/slam_thesis/rover_control/launch/follow_trajectory.launch.py').read())
ld = generate_launch_description()
assert isinstance(ld, LaunchDescription), 'Not a LaunchDescription'
" 2>/tmp/launch_check.log; then
    check_pass "follow_trajectory.launch.py is valid"
else
    check_fail "follow_trajectory.launch.py has errors"
    cat /tmp/launch_check.log
fi

echo ""

# 7. Check imports work
echo "Checking Python imports..."

if python3 -c "
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros
" 2>/tmp/import_check.log; then
    check_pass "All ROS2 imports work"
else
    check_fail "ROS2 import errors"
    cat /tmp/import_check.log
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M2 VALIDATION PASSED${NC}"
    echo ""
    echo "Next steps for manual verification:"
    echo "  1. Start simulation:"
    echo "     ros2 launch rover_sim sim.launch.py"
    echo ""
    echo "  2. In another terminal, run trajectory:"
    echo "     source ~/thesis/ros2_ws/install/setup.bash"
    echo "     ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv"
    echo ""
    echo "  3. Verify:"
    echo "     - Robot moves following the trajectory"
    echo "     - /cmd_vel is being published"
    echo "     - /trajectory_done publishes True when complete"
    echo ""
    echo "  4. Monitor topics:"
    echo "     ros2 topic echo /cmd_vel"
    echo "     ros2 topic echo /trajectory_done"
    exit 0
else
    echo -e "${RED}M2 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
