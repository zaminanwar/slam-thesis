# SLAM Thesis Experiment Platform

A ROS2 Jazzy simulation platform for comparing 2D LiDAR SLAM algorithms (slam_toolbox vs Cartographer) on a simulated rover in Gazebo Harmonic.

## Overview

This project provides a **reproducible experimental framework** for SLAM algorithm comparison:

- **Simulated rover** with 2D LiDAR in Gazebo Harmonic
- **Deterministic trajectories** for reproducible experiments
- **Ground truth** from simulation for accurate evaluation
- **Rosbag recording** for fair A/B comparison (record once, replay many)
- **ATE/RPE metrics** via evo toolbox
- **Batch experiment runner** with results aggregation

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
│   │   ├── rover_control/      # Trajectory follower (pure pursuit)
│   │   ├── gt_publisher/       # Ground truth publisher
│   │   ├── slam_launch/        # SLAM algorithm configs
│   │   ├── traj_exporter/      # TUM format exporter
│   │   └── experiment_runner/  # Orchestration + evaluation scripts
│   ├── bags/                   # Recorded datasets
│   └── results/                # Evaluation outputs
├── trajectories/               # CSV waypoint files
│   ├── traj_01_easy.csv        # Simple forward trajectory
│   ├── traj_02_loop.csv        # Loop closure trajectory
│   └── traj_03_complex.csv     # Complex maneuvers
├── scripts/                    # Validation & utility scripts
└── docs/                       # Documentation
```

## ROS2 Packages

| Package | Purpose | Key Outputs |
|---------|---------|-------------|
| `rover_description` | URDF/Xacro robot model | TF: base_link, laser_frame |
| `rover_sim` | Gazebo world + spawn | /clock, simulation running |
| `rover_control` | Pure pursuit trajectory follower | /cmd_vel, /trajectory_done |
| `gt_publisher` | Ground truth pose from Gazebo | /gt_pose, TF: map_gt→base_link |
| `slam_launch` | SLAM algorithm configs | TF: map→odom |
| `traj_exporter` | Export to TUM format | gt.tum, est.tum |
| `experiment_runner` | Orchestration + evaluation | metrics.json, summary.csv |

## Usage

### Launch Simulation

```bash
# Start Gazebo with rover
ros2 launch rover_sim sim.launch.py
```

### Follow Trajectory

```bash
# Basic trajectory following
ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv

# With custom speed scaling
ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_02_loop.csv speed_scale:=0.8
```

### Record Dataset

```bash
# Record baseline dataset
ros2 launch experiment_runner record_dataset.launch.py \
    trajectory:=traj_01_easy.csv \
    condition:=baseline

# Record with degraded odometry (adds noise)
ros2 launch experiment_runner record_dataset.launch.py \
    trajectory:=traj_01_easy.csv \
    condition:=degraded
```

### Replay Bag for SLAM Evaluation

```bash
# Run slam_toolbox on recorded bag
ros2 launch slam_launch slam_toolbox.launch.py \
    bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline

# Run Cartographer on recorded bag
ros2 launch slam_launch cartographer.launch.py \
    bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline

# Slower playback for debugging
ros2 launch slam_launch slam_toolbox.launch.py \
    bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline \
    rate:=0.5
```

### Export Trajectories

```bash
# Export trajectory during SLAM replay
ros2 launch traj_exporter export_trajectory.launch.py \
    output_file:=~/thesis/ros2_ws/results/traj.tum

# With custom frames
ros2 launch traj_exporter export_trajectory.launch.py \
    output_file:=~/thesis/ros2_ws/results/traj.tum \
    parent_frame:=map \
    child_frame:=base_link
```

### Run Single Experiment

```bash
# Orchestrated experiment (SLAM + export + evaluation)
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_one.py \
    --bag_path ~/thesis/ros2_ws/bags/traj_01_easy__baseline \
    --algorithm slam_toolbox \
    --verbose
```

### Run Batch Experiments

```bash
# Run all experiments
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_all.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --algorithms slam_toolbox cartographer \
    --parallel 2

# Filter by trajectory
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_all.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --filter "traj_01*" \
    --algorithms slam_toolbox
```

### Aggregate Results

```bash
# Generate summary files
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/aggregate_results.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --output results_summary

# Output both CSV and JSON
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/aggregate_results.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --output results_summary \
    --format both
```

## Output Files

### Trajectory Files (TUM Format)

```
# timestamp tx ty tz qx qy qz qw
1234567890.123456 1.234 2.345 0.000 0.000 0.000 0.707 0.707
```

### Metrics (metrics.json)

```json
{
  "success": true,
  "dataset": "traj_01_easy__baseline",
  "algorithm": "slam_toolbox",
  "ate": {"rmse": 0.0523, "mean": 0.0412, "median": 0.0389, ...},
  "rpe": {"rmse": 0.0234, "mean": 0.0198, "median": 0.0187, ...},
  "trajectory_length_m": 45.67,
  "num_poses": 1827
}
```

### Results Directory Structure

```
~/thesis/ros2_ws/bags/traj_01_easy__baseline/
├── metadata.yaml
├── traj_01_easy__baseline_0.db3
├── gt_trajectory.tum
├── slam_toolbox_trajectory.tum
└── results/
    └── slam_toolbox/
        ├── metrics.json
        ├── ate_plot.png
        └── rpe_plot.png
```

## Milestones

| Milestone | Description | Validation | Status |
|-----------|-------------|------------|--------|
| M0 | Foundation - packages build | `validate_m0.sh` | **PASSED** |
| M1 | Simulation - Gazebo + LiDAR | `validate_m1.sh` | **PASSED** |
| M2 | Control - trajectory following | `validate_m2.sh` | **PASSED** |
| M3 | Ground truth publishing | `validate_m3.sh` | **PASSED** |
| M4 | Dataset recording + replay | (programmatic) | **PASSED** |
| M5 | slam_toolbox on bag replay | (programmatic) | **PASSED** |
| M6 | Cartographer on bag replay | (programmatic) | **PASSED** |
| M7 | Trajectory export (TUM) | `validate_m7.sh` | **PASSED** |
| M8 | Evaluation (evo ATE/RPE) | `validate_m8.sh` | **PASSED** |
| M9 | Documentation | `validate_m9.sh` | **PASSED** |

## Validation Scripts

Run validation scripts to verify each milestone:

```bash
# Validate all milestones
for i in 0 1 2 3 7 8 9; do
    bash ~/thesis/scripts/validate_m${i}.sh
done

# Or run individual milestone
bash ~/thesis/scripts/validate_m0.sh
```

See [VALIDATION.md](docs/VALIDATION.md) for detailed validation checklist.

## Environment

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 24.04 (WSL2 on Windows 11) |
| **ROS** | ROS 2 Jazzy |
| **Simulator** | Gazebo Harmonic (gz-sim) via ros_gz |
| **SLAM** | slam_toolbox, Cartographer 2D |
| **Evaluation** | evo toolbox |
| **Python** | 3.12+ |

## TF Frame Tree

```
During Simulation:
    odom                    map_gt
      │                       │
      └── base_link ──────────┘
            │
            └── laser_frame

During SLAM (adds map→odom):
    map                     map_gt
      │                       │
      └── odom                │
            │                 │
            └── base_link ────┘
                  │
                  └── laser_frame
```

## Key Topics

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR at 10 Hz |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry at 50 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Motion commands at 20 Hz |
| `/gt_pose` | `nav_msgs/Odometry` | Ground truth at 50 Hz |
| `/tf` | `tf2_msgs/TFMessage` | Transform frames |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |

## For Claude AI Instances

If you are a Claude AI instance working on this project:

1. Read `.claude/ORIENTATION.md` first
2. Check `.claude/STATE.md` for current task
3. Follow contracts in `.claude/INTERFACES.md`
4. Update `.claude/STATE.md` after completing tasks
5. Reference `.claude/DECISIONS.md` for architectural context
6. Use `.claude/RECOVERY.md` for troubleshooting

## Troubleshooting

### Gazebo Not Launching

```bash
# Check display settings
echo $DISPLAY
# Should be :0 or similar

# For WSL2, ensure X server is running
```

### SLAM Not Working

```bash
# Ensure use_sim_time is set
ros2 param get /slam_toolbox use_sim_time

# Check /clock is being published
ros2 topic hz /clock
```

### TF Lookup Failures

```bash
# Verify TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

## License

MIT

## Contributing

This is a thesis project. For questions or issues, see the `.claude/` directory for project context.
