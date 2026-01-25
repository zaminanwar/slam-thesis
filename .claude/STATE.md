# Project State

**Last Updated**: 2026-01-24T18:00:00
**Last Claude Instance**: M3 Implementation (Opus 4.5)
**Current Milestone**: M4 (Dataset Recording)
**Current Task**: T4.1 - Dataset recorder launch

## Status Summary

| Status | Count |
|--------|-------|
| Completed | 11 |
| In Progress | 0 |
| Pending | 18 |
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

### EPIC 4 - Dataset Recording ← CURRENT
- [ ] T4.1 - Dataset recorder launch
- [ ] T4.2 - Condition system
- [ ] T4.3 - Bag replay wrapper

### EPIC 5 - SLAM Pipelines
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
| M4 | validate_m4.sh | - | Not created yet |
| M5 | validate_m5.sh | - | Not created yet |
| M6 | validate_m6.sh | - | Not created yet |
| M7 | validate_m7.sh | - | Not created yet |
| M8 | validate_m8.sh | - | Not created yet |
| M9 | validate_m9.sh | - | Not created yet |

## Next Action

**For new Claude instance:**
Implement M4 (Dataset Recording):
- T4.1: Dataset recorder launch - record bags with topics for replay
- T4.2: Condition system - support baseline and degraded conditions
- T4.3: Bag replay wrapper - launch file for replaying bags

## Environment

- **Ubuntu**: 24.04
- **ROS2**: Jazzy (saved in ~/thesis/.ros_distro)
- **Gazebo**: Harmonic (gz-sim) via ros_gz
- **WSL2 display**: DISPLAY=:0, WAYLAND_DISPLAY=wayland-0

## Recent Changes

| Date | Task | Change | Notes |
|------|------|--------|-------|
| 2026-01-24 | T0.1 | Created workspace structure | ~/thesis/ with all directories |
| 2026-01-24 | T0.2 | Created all 7 package skeletons | All package.xml, setup.py, CMakeLists.txt |
| 2026-01-24 | - | Created .claude/ continuity system | 5 files |
| 2026-01-24 | - | Updated for Ubuntu 24.04/Jazzy | Script detects distro |
| 2026-01-24 | M0 | **VALIDATED** | colcon build passes |
| 2026-01-24 | T1.1 | Created rover URDF/Xacro | base_link, laser_frame, diff_drive, LiDAR |
| 2026-01-24 | T1.2 | Created RViz config | rover.rviz |
| 2026-01-24 | T1.3 | Created sim.launch.py + simple.sdf | Gazebo Harmonic world with obstacles |
| 2026-01-24 | T1.4 | LiDAR publishes /scan | Via gz-sim gpu_lidar sensor |
| 2026-01-24 | M1 | **VALIDATED** | validate_m1.sh passes |
| 2026-01-24 | - | Fixed Jazzy launch bug | ParameterValue wrapper (see AD-010) |
| 2026-01-24 | M1 | **MANUAL TEST PASSED** | Robot moves, /scan works, /cmd_vel works |
| 2026-01-24 | T2.1 | Verified odometry | diff_drive plugin publishes /odom + TF |
| 2026-01-24 | T2.2 | Created trajectory CSVs | traj_01_easy, traj_02_loop, traj_03_complex |
| 2026-01-24 | T2.3 | Implemented trajectory follower | Pure pursuit algorithm |
| 2026-01-24 | T2.4 | Created launch file | follow_trajectory.launch.py |
| 2026-01-24 | M2 | **VALIDATED** | validate_m2.sh passes |
| 2026-01-24 | T3.1 | Created gt_publisher_node.py | Subscribes to Gazebo pose, publishes /gt_pose + TF |
| 2026-01-24 | T3.1 | Added pose bridge to sim.launch.py | /model/rover/pose bridged from Gazebo |
| 2026-01-24 | T3.1 | Created gt_publisher.launch.py | Launch file for GT publisher |
| 2026-01-24 | M3 | **VALIDATED** | validate_m3.sh passes |
| 2026-01-24 | T3.1 | Added PosePublisher plugin to rover URDF | Required for Gazebo to publish /model/rover/pose |
| 2026-01-24 | M3 | **MANUAL TEST PASSED** | /gt_pose updates, TF map_gt->base_link works |

## Key Files Created in M1

```
rover_description/
├── urdf/rover.urdf.xacro      # Differential drive robot with LiDAR
├── rviz/rover.rviz            # Visualization config
└── launch/display.launch.py   # RViz launch

rover_sim/
├── worlds/simple.sdf          # 10x10m room with obstacles
└── launch/sim.launch.py       # Gazebo Harmonic + ros_gz_bridge
```

## Key Files Created in M2

```
trajectories/
├── traj_01_easy.csv           # Simple square trajectory (6 waypoints)
├── traj_02_loop.csv           # Loop with turns (11 waypoints)
└── traj_03_complex.csv        # Complex path (17 waypoints)

rover_control/
├── rover_control/trajectory_follower.py  # Pure pursuit node
└── launch/follow_trajectory.launch.py    # Trajectory launch
```

## Key Files Created in M3

```
gt_publisher/
├── gt_publisher/gt_publisher_node.py  # Ground truth from Gazebo pose
└── launch/gt_publisher.launch.py      # GT publisher launch

rover_description/
└── urdf/rover.urdf.xacro              # Added PosePublisher plugin for GT

rover_sim/
└── launch/sim.launch.py               # Updated with /model/rover/pose bridge
```

## Key Files for M4

```
experiment_runner/
├── launch/record_dataset.launch.py    # Record bags with GT + sensors
├── launch/replay_bag.launch.py        # Replay bags for SLAM evaluation
└── config/topics_to_record.yaml       # Topics list for rosbag record
```
