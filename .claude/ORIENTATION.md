# SLAM Thesis Project - Claude Orientation

> **For New Claude Instances**: Read this file completely first, then STATE.md, then INTERFACES.md.

## What This Project Is

A ROS2 Jazzy simulation platform comparing two 2D LiDAR SLAM algorithms:
- **slam_toolbox** vs **Cartographer 2D**

The platform uses a simulated rover in Gazebo Harmonic with predefined trajectories.
Evaluation uses rosbag replay for fair A/B comparison, producing ATE/RPE metrics via evo.

**Extended Goal**: Nav2 autonomous navigation integration to evaluate how SLAM quality
affects autonomous navigation (success rate, navigation time, path efficiency).

## Environment

| Component | Value |
|-----------|-------|
| OS | Ubuntu 24.04 on WSL2 (Windows 11) |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim) via ros_gz |
| Nav2 | Navigation2 (for autonomous navigation experiments) |
| Workspace | ~/thesis/ros2_ws |
| Python | 3.12+ |

## Project Philosophy

**"The Repo Is The Brain"** - All decisions, state, and interfaces are documented in files,
not in chat history. Any Claude instance can achieve full context by reading files in .claude/.

## For New Claude Instances

1. Read this file completely
2. Read `.claude/STATE.md` for current progress and next task
3. Read `.claude/INTERFACES.md` before writing any code that involves topics/frames/formats
4. Read `.claude/DECISIONS.md` to understand why things are done a certain way
5. If something is broken, read `.claude/RECOVERY.md`

## Package Overview

| Package | Purpose | Key Outputs |
|---------|---------|-------------|
| rover_description | URDF/Xacro robot model | TF: base_link, laser_frame |
| rover_sim | Gazebo world + spawn | /clock, simulation running |
| rover_control | Trajectory follower | /cmd_vel, /trajectory_done |
| gt_publisher | Ground truth pose | /gt_pose, TF: map_gt→base_link |
| slam_launch | SLAM algorithm configs | TF: map→odom |
| traj_exporter | Export to TUM format | gt.tum, est.tum |
| experiment_runner | Orchestration + eval | metrics.json, summary.csv |
| nav_launch | Nav2 + SLAM integration | /navigate_to_pose, nav_results.json |

## TF Frame Tree (Authoritative)

```
map_gt                    # Ground truth origin (NEVER conflicts with SLAM)
  └── base_link           # True pose from Gazebo

map                       # SLAM estimate origin
  └── odom                # Corrected by SLAM
        └── base_link     # Odometry estimate
              └── laser_frame  # LiDAR sensor (static)
```

## Critical Rules

1. **ALL nodes MUST use `use_sim_time:=true`** when running with simulation or bag replay
2. **Ground truth frame is `map_gt`**, NEVER `map` (SLAM owns `map`)
3. **Odometry publishes `odom→base_link`**, never anything with `map`
4. **SLAM publishes `map→odom`**, completing the tree
5. **LiDAR frame is `laser_frame`**, rigidly attached to `base_link`
6. **Never change interface contracts** without updating INTERFACES.md and all dependent code

## Do NOT

- Change frame names without updating INTERFACES.md
- Add dependencies not declared in package.xml
- Skip validation scripts after completing a milestone
- Assume prior work is correct without running validation
- Make architectural decisions without documenting in DECISIONS.md
- Use `map` frame for ground truth (use `map_gt`)

## Directory Structure

```
~/thesis/
├── .claude/                    # Claude continuity system
│   ├── ORIENTATION.md          # This file
│   ├── STATE.md                # Current progress
│   ├── DECISIONS.md            # Architectural decisions
│   ├── INTERFACES.md           # Locked contracts
│   └── RECOVERY.md             # Troubleshooting
├── ros2_ws/
│   ├── src/slam_thesis/        # All ROS2 packages
│   ├── bags/                   # Recorded datasets
│   └── results/                # Evaluation outputs
├── trajectories/               # CSV waypoint files
├── scripts/                    # Validation + utility scripts
└── docs/                       # User documentation
```

## Milestone Overview

| Milestone | Goal | Validation |
|-----------|------|------------|
| M0 | Foundation - packages build | `colcon build` succeeds |
| M1 | Simulation - Gazebo + LiDAR | validate_m1.sh |
| M2 | Control - trajectory following | validate_m2.sh |
| M3 | Ground truth publishing | validate_m3.sh |
| M4 | Dataset recording + replay | validate_m4.sh |
| M5 | slam_toolbox on bag replay | validate_m5.sh |
| M6 | Cartographer on bag replay | validate_m6.sh |
| M7 | Trajectory export (TUM) | validate_m7.sh |
| M8 | Evaluation (evo ATE/RPE) | validate_m8.sh |
| M9 | Batch runner + aggregation | validate_m9.sh |
| M10 | Nav2 launch package | validate_m10.sh |
| M11 | Navigation experiments | validate_m11.sh |

## Quick Commands Reference

```bash
# Source workspace
source ~/thesis/ros2_ws/install/setup.bash

# Build all packages
cd ~/thesis/ros2_ws && colcon build

# Build single package
colcon build --packages-select rover_description

# Launch simulation
ros2 launch rover_sim sim.launch.py

# Run trajectory
ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv

# Record dataset
ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy condition:=baseline

# Run SLAM evaluation (bag replay)
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_one.py --algo slam_toolbox --bag ~/thesis/ros2_ws/bags/traj_01_easy__baseline
```

### Nav2 Autonomous Navigation Commands

```bash
# Launch Nav2 with SLAM-in-the-loop (requires nav_launch package)
ros2 launch nav_launch nav2_slam.launch.py algorithm:=slam_toolbox

# Launch with Cartographer instead
ros2 launch nav_launch nav2_slam.launch.py algorithm:=cartographer

# Send a single navigation goal (in another terminal)
ros2 launch nav_launch send_goal.launch.py x:=2.0 y:=1.0 yaw:=0.0

# Run automated navigation experiment
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_nav_experiment.py \
  --algorithm slam_toolbox \
  --goals nav_goals_01.yaml \
  --verbose

# Compare navigation with different SLAM algorithms
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_nav_experiment.py \
  --algorithm cartographer \
  --goals nav_goals_01.yaml \
  --verbose

# Check navigation results
cat ~/thesis/ros2_ws/results/nav_slam_toolbox_*/nav_results.json
```
