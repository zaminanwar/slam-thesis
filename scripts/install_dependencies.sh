#!/bin/bash
#
# SLAM Thesis - Dependency Installation Script
# Supports Ubuntu 22.04 (ROS2 Humble) and Ubuntu 24.04 (ROS2 Jazzy)
#
# Usage: bash ~/thesis/scripts/install_dependencies.sh
#

set -e  # Exit on error

echo "=============================================="
echo "SLAM Thesis - Dependency Installation"
echo "=============================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print status
print_status() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Detect Ubuntu version and set ROS2 distro
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    UBUNTU_VERSION="$VERSION_ID"
    UBUNTU_CODENAME="$UBUNTU_CODENAME"
else
    print_error "Cannot detect Ubuntu version"
    exit 1
fi

# Set ROS2 distro based on Ubuntu version
if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
    print_status "Detected Ubuntu 22.04 - Using ROS2 Humble"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jazzy"
    print_status "Detected Ubuntu 24.04 - Using ROS2 Jazzy"
else
    print_error "Unsupported Ubuntu version: $UBUNTU_VERSION"
    print_error "This script supports Ubuntu 22.04 (Humble) or 24.04 (Jazzy)"
    exit 1
fi

echo ""
echo "Step 1: Updating package lists..."
sudo apt update
print_status "Package lists updated"

echo ""
echo "Step 2: Setting up ROS2 repository..."
# Install prerequisites for adding repos
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-pip

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    print_status "ROS2 GPG key added"
fi

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
print_status "ROS2 ${ROS_DISTRO} repository configured"

echo ""
echo "Step 3: Installing ROS2 ${ROS_DISTRO}..."
sudo apt install -y ros-${ROS_DISTRO}-desktop
print_status "ROS2 ${ROS_DISTRO} Desktop installed"

echo ""
echo "Step 4: Installing colcon and rosdep..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
print_status "Build tools installed"

echo ""
echo "Step 5: Installing Gazebo and ROS2 integration..."
if [[ "$ROS_DISTRO" == "humble" ]]; then
    # Humble uses Gazebo Classic
    sudo apt install -y \
        gazebo \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-gazebo-ros \
        ros-${ROS_DISTRO}-gazebo-plugins \
        ros-${ROS_DISTRO}-gazebo-msgs
    print_status "Gazebo Classic installed"
else
    # Jazzy uses Gazebo Harmonic (new Gazebo) with ros_gz bridge
    sudo apt install -y \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-image \
        ros-${ROS_DISTRO}-ros-gz-interfaces
    print_status "Gazebo Harmonic (ros_gz) installed"
    print_warning "Note: Jazzy uses new Gazebo (Harmonic) instead of Gazebo Classic"
    print_warning "Some launch files may need adjustment for gz-sim syntax"
fi

echo ""
echo "Step 6: Installing SLAM packages..."
sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros
print_status "SLAM packages installed"

echo ""
echo "Step 7: Installing ROS2 common packages..."
sudo apt install -y \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins
print_status "ROS2 common packages installed"

echo ""
echo "Step 8: Installing Python packages..."
pip3 install --user --break-system-packages \
    evo \
    numpy \
    matplotlib \
    pandas \
    pyyaml || pip3 install --user \
    evo \
    numpy \
    matplotlib \
    pandas \
    pyyaml
print_status "Python packages installed"

echo ""
echo "Step 9: Initializing rosdep..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || print_warning "rosdep init failed (may already be initialized)"
fi
rosdep update
print_status "rosdep initialized and updated"

echo ""
echo "Step 10: Setting up environment..."
# Remove any old ROS sourcing first
sed -i '/source \/opt\/ros\/humble\/setup.bash/d' ~/.bashrc 2>/dev/null || true
sed -i '/source \/opt\/ros\/jazzy\/setup.bash/d' ~/.bashrc 2>/dev/null || true
sed -i '/# ROS2 Humble/d' ~/.bashrc 2>/dev/null || true
sed -i '/# ROS2 Jazzy/d' ~/.bashrc 2>/dev/null || true

# Add correct ROS2 sourcing
echo "" >> ~/.bashrc
echo "# ROS2 ${ROS_DISTRO^}" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
print_status "Added ROS2 ${ROS_DISTRO} to ~/.bashrc"

# Add workspace sourcing to bashrc if not already present
if ! grep -q "source ~/thesis/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# SLAM Thesis Workspace" >> ~/.bashrc
    echo "if [ -f ~/thesis/ros2_ws/install/setup.bash ]; then" >> ~/.bashrc
    echo "    source ~/thesis/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
    print_status "Added thesis workspace to ~/.bashrc"
else
    print_status "Thesis workspace already in ~/.bashrc"
fi

# Save ROS_DISTRO to a file for other scripts to use
echo "${ROS_DISTRO}" > ~/thesis/.ros_distro
print_status "Saved ROS distro info to ~/thesis/.ros_distro"

echo ""
echo "=============================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=============================================="
echo ""
echo "ROS2 Distro: ${ROS_DISTRO}"
if [[ "$ROS_DISTRO" == "jazzy" ]]; then
    echo ""
    echo -e "${YELLOW}IMPORTANT for Jazzy/Ubuntu 24.04:${NC}"
    echo "  - Uses Gazebo Harmonic (gz-sim) instead of Gazebo Classic"
    echo "  - Launch files will use 'ros_gz' packages"
    echo "  - World files use SDF format"
fi
echo ""
echo "Next steps:"
echo "  1. Close and reopen your terminal, OR run:"
echo "     source ~/.bashrc"
echo ""
echo "  2. Build the workspace:"
echo "     cd ~/thesis/ros2_ws"
echo "     colcon build"
echo ""
echo "  3. Verify installation:"
echo "     ros2 pkg list | grep rover"
echo ""
if [[ "$ROS_DISTRO" == "humble" ]]; then
    echo "  4. Test Gazebo (optional):"
    echo "     gazebo --verbose"
else
    echo "  4. Test Gazebo Harmonic (optional):"
    echo "     gz sim --version"
fi
echo ""
