# Project State

**Last Updated**: 2026-01-26T14:35:00
**Last Claude Instance**: M4 Implementation (Opus 4.5)
**Current Milestone**: M5 (SLAM Pipelines)
**Current Task**: T5.1 - slam_toolbox bag replay

## Status Summary

| Status | Count |
|--------|-------|
| Completed | 14 |
| In Progress | 0 |
| Pending | 15 |
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

### EPIC 5 - SLAM Pipelines ← CURRENT
- [ ] T5.1 - slam_toolbox bag replay
- [ ] T5.2 - Cartographer 2D bag replay

### EPIC 6 - Trajectory Export
- [ ] T6.1 - TF-based trajectory exporter
- [ ] T6.2 - Export integration

### EPIC 7 - Evaluation
- [ ] T7.1 - Per-run evaluation script
- [ ] T7.2 - Failure handling

### EPIC 8 - Experiment Runner
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
| M5 | validate_m5.sh | - | Not created yet |
| M6 | validate_m6.sh | - | Not created yet |
| M7 | validate_m7.sh | - | Not created yet |
| M8 | validate_m8.sh | - | Not created yet |
| M9 | validate_m9.sh | - | Not created yet |

## Next Action

**For new Claude instance:**
Implement M5 (SLAM Pipelines):
- T5.1: slam_toolbox launch file for bag replay evaluation
- T5.2: Cartographer 2D launch file for bag replay evaluation

Both should:
- Accept bag_path parameter
- Run SLAM algorithm on replayed sensor data
- Publish map and SLAM-corrected pose for evaluation

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

## Key Files for M5

```
slam_launch/
├── config/slam_toolbox.yaml           # slam_toolbox parameters
├── config/cartographer_2d.lua         # Cartographer parameters
├── launch/slam_toolbox.launch.py      # slam_toolbox with bag replay
└── launch/cartographer.launch.py      # Cartographer with bag replay
```

## Git Branch Info

- **Current branch**: fresh-start (at M0-M3, working on M4+)
- **master**: synced with origin/master (has M4-M9 from previous work)
- **backup-m8**: previous M4-M8 implementation (reference)
