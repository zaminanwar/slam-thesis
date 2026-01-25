# SLAM Thesis Experiment Platform

A ROS2 Humble simulation platform for comparing 2D LiDAR SLAM algorithms (slam_toolbox vs Cartographer) on a simulated rover.

## Overview

This project provides a reproducible experimental framework for SLAM algorithm comparison:

- **Simulated rover** with 2D LiDAR in Gazebo Classic
- **Deterministic trajectories** for reproducible experiments
- **Ground truth** from simulation for accurate evaluation
- **Rosbag recording** for fair A/B comparison
- **ATE/RPE metrics** via evo toolbox

## Quick Start

### 1. Install Dependencies

```bash
bash ~/thesis/scripts/install_dependencies.sh
source ~/.bashrc
```

### 2. Build Workspace

```bash
cd ~/thesis/ros2_ws
colcon build
source install/setup.bash
```

### 3. Verify Installation

```bash
bash ~/thesis/scripts/validate_m0.sh
```

## Project Structure

```
~/thesis/
├── .claude/                    # Claude AI continuity system
│   ├── ORIENTATION.md          # Project overview (read first)
│   ├── STATE.md                # Current progress
│   ├── DECISIONS.md            # Architectural decisions
│   ├── INTERFACES.md           # Locked contracts (topics, frames)
│   └── RECOVERY.md             # Troubleshooting guide
├── ros2_ws/
│   ├── src/slam_thesis/        # ROS2 packages
│   │   ├── rover_description/  # URDF robot model
│   │   ├── rover_sim/          # Gazebo world + launch
│   │   ├── rover_control/      # Trajectory follower
│   │   ├── gt_publisher/       # Ground truth publisher
│   │   ├── slam_launch/        # SLAM algorithm configs
│   │   ├── traj_exporter/      # TUM format exporter
│   │   └── experiment_runner/  # Orchestration scripts
│   ├── bags/                   # Recorded datasets
│   └── results/                # Evaluation outputs
├── trajectories/               # CSV waypoint files
├── scripts/                    # Utility scripts
└── docs/                       # Documentation
```

## Usage

### Launch Simulation
```bash
ros2 launch rover_sim sim.launch.py
```

### Follow Trajectory
```bash
ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv
```

### Record Dataset
```bash
ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy condition:=baseline
```

### Run SLAM Evaluation
```bash
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_one.py \
    --algo slam_toolbox \
    --bag ~/thesis/ros2_ws/bags/traj_01_easy__baseline
```

### Run All Experiments
```bash
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_all.py
```

## Milestones

| Milestone | Description | Status |
|-----------|-------------|--------|
| M0 | Foundation - packages build | In Progress |
| M1 | Simulation - Gazebo + LiDAR | Pending |
| M2 | Control - trajectory following | Pending |
| M3 | Ground truth publishing | Pending |
| M4 | Dataset recording + replay | Pending |
| M5 | slam_toolbox evaluation | Pending |
| M6 | Cartographer evaluation | Pending |
| M7 | Trajectory export (TUM) | Pending |
| M8 | Evaluation (evo ATE/RPE) | Pending |
| M9 | Batch runner + aggregation | Pending |

## Environment

- **OS**: Ubuntu 22.04 (WSL2 on Windows 11)
- **ROS**: ROS 2 Humble
- **Simulator**: Gazebo Classic (gazebo11)
- **SLAM**: slam_toolbox, Cartographer 2D
- **Evaluation**: evo toolbox

## For Claude AI Instances

If you are a Claude AI instance working on this project:

1. Read `~/.claude/ORIENTATION.md` first
2. Check `~/.claude/STATE.md` for current task
3. Follow contracts in `~/.claude/INTERFACES.md`
4. Update `~/.claude/STATE.md` after completing tasks

## License

MIT
