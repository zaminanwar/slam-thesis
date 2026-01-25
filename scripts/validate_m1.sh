#!/bin/bash
#
# M1 Validation Script - Simulation
#
# Validates:
# 1. Packages build successfully
# 2. URDF is valid (xacro processing)
# 3. Launch files parse correctly
# 4. Required files exist
#
# Note: Full Gazebo testing requires manual verification
# (WSL2 display, GPU support, etc.)

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
echo "M1 Validation - Simulation"
echo "=========================================="
echo ""

# Track failures
FAILURES=0

# Helper function
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

# 1. Check required files exist
echo "Checking required files..."

FILES=(
    "$WS_DIR/src/slam_thesis/rover_description/urdf/rover.urdf.xacro"
    "$WS_DIR/src/slam_thesis/rover_description/rviz/rover.rviz"
    "$WS_DIR/src/slam_thesis/rover_description/launch/display.launch.py"
    "$WS_DIR/src/slam_thesis/rover_sim/worlds/simple.sdf"
    "$WS_DIR/src/slam_thesis/rover_sim/launch/sim.launch.py"
)

for f in "${FILES[@]}"; do
    if [[ -f "$f" ]]; then
        check_pass "$(basename "$f") exists"
    else
        check_fail "$(basename "$f") missing: $f"
    fi
done

echo ""

# 2. Source ROS and build
echo "Building packages..."

# Detect ROS distro
if [[ -f "$THESIS_DIR/.ros_distro" ]]; then
    ROS_DISTRO=$(cat "$THESIS_DIR/.ros_distro")
else
    ROS_DISTRO="jazzy"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_DIR"

# Build rover_description and rover_sim
if colcon build --packages-select rover_description rover_sim 2>&1 | tee /tmp/m1_build.log | tail -10; then
    if grep -q "Summary: 2 packages finished" /tmp/m1_build.log || \
       grep -q "packages finished" /tmp/m1_build.log; then
        check_pass "Packages built successfully"
    else
        check_fail "Package build may have issues"
    fi
else
    check_fail "Package build failed"
fi

echo ""

# 3. Source workspace and validate URDF
echo "Validating URDF..."
source "$WS_DIR/install/setup.bash"

URDF_FILE="$WS_DIR/src/slam_thesis/rover_description/urdf/rover.urdf.xacro"

# Process xacro
if xacro "$URDF_FILE" > /tmp/rover.urdf 2>/tmp/xacro_errors.log; then
    check_pass "Xacro processing succeeded"
else
    check_fail "Xacro processing failed"
    cat /tmp/xacro_errors.log
fi

# Check URDF has required links
if grep -q 'link name="base_link"' /tmp/rover.urdf; then
    check_pass "base_link found in URDF"
else
    check_fail "base_link NOT found in URDF"
fi

if grep -q 'link name="laser_frame"' /tmp/rover.urdf; then
    check_pass "laser_frame found in URDF"
else
    check_fail "laser_frame NOT found in URDF"
fi

# Check for wheel joints (diff drive)
if grep -q 'joint name="left_wheel_joint"' /tmp/rover.urdf && \
   grep -q 'joint name="right_wheel_joint"' /tmp/rover.urdf; then
    check_pass "Wheel joints found in URDF"
else
    check_fail "Wheel joints NOT found in URDF"
fi

# Check for Gazebo plugins
if grep -q 'gz-sim-diff-drive-system' /tmp/rover.urdf; then
    check_pass "Diff drive plugin found"
else
    check_fail "Diff drive plugin NOT found"
fi

if grep -q 'type="gpu_lidar"' /tmp/rover.urdf; then
    check_pass "LiDAR sensor found"
else
    check_fail "LiDAR sensor NOT found"
fi

echo ""

# 4. Validate launch files (syntax check)
echo "Validating launch files..."

# Check display.launch.py
if python3 -c "
import sys
sys.path.insert(0, '$WS_DIR/install/rover_description/share/rover_description/launch')
from launch import LaunchDescription
exec(open('$WS_DIR/src/slam_thesis/rover_description/launch/display.launch.py').read())
ld = generate_launch_description()
assert isinstance(ld, LaunchDescription), 'Not a LaunchDescription'
" 2>/tmp/launch_check.log; then
    check_pass "display.launch.py is valid"
else
    check_fail "display.launch.py has errors"
    cat /tmp/launch_check.log
fi

# Check sim.launch.py
if python3 -c "
import sys
sys.path.insert(0, '$WS_DIR/install/rover_sim/share/rover_sim/launch')
from launch import LaunchDescription
exec(open('$WS_DIR/src/slam_thesis/rover_sim/launch/sim.launch.py').read())
ld = generate_launch_description()
assert isinstance(ld, LaunchDescription), 'Not a LaunchDescription'
" 2>/tmp/launch_check.log; then
    check_pass "sim.launch.py is valid"
else
    check_fail "sim.launch.py has errors"
    cat /tmp/launch_check.log
fi

echo ""

# 5. Check SDF world file
echo "Validating SDF world..."

SDF_FILE="$WS_DIR/src/slam_thesis/rover_sim/worlds/simple.sdf"

if grep -q '<world name=' "$SDF_FILE"; then
    check_pass "SDF has world element"
else
    check_fail "SDF missing world element"
fi

if grep -q 'gz-sim-physics-system' "$SDF_FILE"; then
    check_pass "Physics plugin found"
else
    check_fail "Physics plugin NOT found"
fi

if grep -q 'gz-sim-sensors-system' "$SDF_FILE"; then
    check_pass "Sensors plugin found"
else
    check_fail "Sensors plugin NOT found"
fi

echo ""

# 6. Check ros_gz dependencies are available
echo "Checking ros_gz availability..."

if ros2 pkg list 2>/dev/null | grep -q "ros_gz_sim"; then
    check_pass "ros_gz_sim package available"
else
    check_warn "ros_gz_sim not found (install with: sudo apt install ros-$ROS_DISTRO-ros-gz)"
fi

if ros2 pkg list 2>/dev/null | grep -q "ros_gz_bridge"; then
    check_pass "ros_gz_bridge package available"
else
    check_warn "ros_gz_bridge not found (install with: sudo apt install ros-$ROS_DISTRO-ros-gz)"
fi

echo ""

# Summary
echo "=========================================="
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}M1 VALIDATION PASSED${NC}"
    echo ""
    echo "Next steps for manual verification:"
    echo "  1. ros2 launch rover_description display.launch.py jsp_gui:=true"
    echo "     - Verify robot model appears in RViz"
    echo "  2. ros2 launch rover_sim sim.launch.py"
    echo "     - Verify Gazebo opens with world"
    echo "     - Verify robot spawns"
    echo "  3. In another terminal:"
    echo "     ros2 topic list"
    echo "     - Should see /scan, /odom, /cmd_vel, /clock"
    echo "  4. ros2 topic echo /scan --once"
    echo "     - Should see LaserScan data"
    exit 0
else
    echo -e "${RED}M1 VALIDATION FAILED${NC} ($FAILURES failures)"
    exit 1
fi
