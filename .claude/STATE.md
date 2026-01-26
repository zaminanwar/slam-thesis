# Project State

**Last Updated**: 2026-01-26T14:55:00
**Last Claude Instance**: M7 Implementation (Opus 4.5)
**Current Milestone**: M8 (Experiment Runner)
**Current Task**: T8.1 - run_one.py

## Status Summary

| Status | Count |
|--------|-------|
| Completed | 20 |
| In Progress | 0 |
| Pending | 9 |
| Blocked | 0 |

## Progress

### EPIC 0 - Foundation ✓
- [x] T0.1 - Create workspace + repo skeleton
- [x] T0.2 - Create ROS2 packages (empty but correct)

### EPIC 1 - Rover Model + Simulation ✓
- [x] T1.1 - Implement minimal rover URDF/Xacro
- [x] T1.2 - Add RViz config
- [x] T1.3 - Implement Gazebo simulation launch
- [x] T1.4 - Publish 2D LiDAR /scan

### EPIC 2 - Odometry + Motion Control ✓
- [x] T2.1 - Wheel odometry (/odom + TF)
- [x] T2.2 - Trajectory CSV files
- [x] T2.3 - Pure pursuit trajectory follower
- [x] T2.4 - Trajectory launch file

### EPIC 3 - Ground Truth ✓
- [x] T3.1 - Ground truth publisher node

### EPIC 4 - Dataset Recording ✓
- [x] T4.1 - Dataset recorder launch
- [x] T4.2 - Condition system (baseline/degraded with odom noise)
- [x] T4.3 - Bag replay wrapper

### EPIC 5 - SLAM Pipelines ✓
- [x] T5.1 - slam_toolbox bag replay
- [x] T5.2 - Cartographer 2D bag replay

### EPIC 6 - Trajectory Export ✓
- [x] T6.1 - TF-based trajectory exporter
- [x] T6.2 - Export integration

### EPIC 7 - Evaluation ✓
- [x] T7.1 - Per-run evaluation script
- [x] T7.2 - Failure handling

### EPIC 8 - Experiment Runner ← CURRENT
- [ ] T8.1 - run_one.py
- [ ] T8.2 - run_all.py
- [ ] T8.3 - aggregate_results.py

### EPIC 9 - Documentation
- [ ] T9.1 - Top-level README
- [ ] T9.2 - Validation checklist

## Milestone Validation Status

| Milestone | Script | Last Run | Status |
|-----------|--------|----------|--------|
| M0 | validate_m0.sh | 2026-01-24 | **PASSED** |
| M1 | validate_m1.sh | 2026-01-24 | **PASSED** (script + manual) |
| M2 | validate_m2.sh | 2026-01-24 | **PASSED** |
| M3 | validate_m3.sh | 2026-01-24 | **PASSED** (script + manual) |
| M4 | - | 2026-01-26 | **PASSED** (build + launch args verified) |
| M5 | - | 2026-01-26 | **PASSED** (build + launch args verified) |
| M6 | - | 2026-01-26 | **PASSED** (build + launch args verified) |
| M7 | validate_m7.sh | 2026-01-26 | **PASSED** |
| M8 | validate_m8.sh | - | Not created yet |
| M9 | validate_m9.sh | - | Not created yet |

## Next Action

**For new Claude instance:**
Implement M8 (Experiment Runner):
- T8.1: run_one.py - Single experiment orchestrator
- T8.2: run_all.py - Batch experiment runner
- T8.3: aggregate_results.py - Results aggregation

run_one.py should:
- Take bag_path and algorithm as input
- Launch SLAM (slam_toolbox or cartographer) with bag replay
- Launch traj_exporter to export gt.tum and est.tum
- Call evaluate_run.py to compute metrics
- Handle timeouts and failures gracefully

## Environment

- **Ubuntu**: 24.04
- **ROS2**: Jazzy (saved in ~/thesis/.ros_distro)
- **Gazebo**: Harmonic (gz-sim) via ros_gz
- **WSL2 display**: DISPLAY=:0, WAYLAND_DISPLAY=wayland-0

## Key Files Created in M4

```
experiment_runner/
├── config/topics_to_record.yaml       # Topics: /scan, /odom, /tf, /gt_pose, /clock
├── launch/record_dataset.launch.py    # Record bags during trajectory run
├── launch/replay_bag.launch.py        # Replay bags for offline SLAM
└── experiment_runner/odom_noise_node.py  # Adds Gaussian noise for degraded condition
```

**M4 Usage:**
```bash
# Record dataset
ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv

# Record with degraded odometry
ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy.csv condition:=degraded

# Replay bag
ros2 launch experiment_runner replay_bag.launch.py bag_path:=/path/to/bag
```

## Key Files Created in M5

```
slam_launch/
├── config/slam_toolbox.yaml           # slam_toolbox parameters (5cm resolution, loop closure enabled)
├── config/cartographer_2d.lua         # Cartographer parameters (2D SLAM with scan matching)
├── launch/slam_toolbox.launch.py      # slam_toolbox with bag replay
└── launch/cartographer.launch.py      # Cartographer with bag replay
```

**M5 Usage:**
```bash
# Run slam_toolbox on a recorded bag
ros2 launch slam_launch slam_toolbox.launch.py bag_path:=/path/to/bag

# Run Cartographer on a recorded bag
ros2 launch slam_launch cartographer.launch.py bag_path:=/path/to/bag

# With slower playback for debugging
ros2 launch slam_launch slam_toolbox.launch.py bag_path:=/path/to/bag rate:=0.5
```

## Key Files Created in M6

```
traj_exporter/
├── traj_exporter/traj_exporter_node.py  # TF trajectory exporter node (TUM format)
└── launch/export_trajectory.launch.py   # Export launch file
```

**M6 Usage:**
```bash
# Export trajectory standalone
ros2 launch traj_exporter export_trajectory.launch.py output_file:=/path/to/traj.txt

# With custom frame names
ros2 launch traj_exporter export_trajectory.launch.py \
  output_file:=/path/to/traj.txt \
  parent_frame:=map \
  child_frame:=base_link

# Run node directly
ros2 run traj_exporter traj_exporter --ros-args \
  -p output_file:=/path/to/traj.txt \
  -p parent_frame:=map \
  -p child_frame:=base_footprint
```

**TUM Format Output:**
```
# timestamp tx ty tz qx qy qz qw
1234567890.123456789 1.234 2.345 0.000 0.000 0.000 0.707 0.707
```

## Key Files Created in M7

```
experiment_runner/
└── scripts/
    └── evaluate_run.py  # Per-run evaluation using evo library
```

**M7 Usage:**
```bash
# Evaluate trajectory comparison
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/evaluate_run.py \
  --gt_file /path/to/gt.tum \
  --est_file /path/to/est.tum \
  --output_dir /path/to/output \
  --dataset traj_01_easy__baseline \
  --algorithm slam_toolbox \
  --save_plots --verbose
```

**Output Files:**
- `metrics.json`: ATE/RPE statistics (rmse, mean, median, std, min, max)
- `ate_plot.png`: Trajectory comparison plot (optional)
- `rpe_plot.png`: RPE distribution histogram (optional)

**metrics.json Format:**
```json
{
  "success": true,
  "dataset": "traj_01_easy__baseline",
  "algorithm": "slam_toolbox",
  "timestamp": "2026-01-26T15:00:00",
  "ate": {"rmse": 0.0523, "mean": 0.0412, ...},
  "rpe": {"rmse": 0.0234, "mean": 0.0198, ...},
  "trajectory_length_m": 45.67,
  "duration_s": 91.34,
  "num_poses": 1827,
  "error": null
}
```

## Git Branch Info

- **Current branch**: fresh-start (at M0-M3, working on M4+)
- **master**: synced with origin/master (has M4-M9 from previous work)
- **backup-m8**: previous M4-M8 implementation (reference)
