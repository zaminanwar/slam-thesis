# Validation Checklist

This document provides a comprehensive checklist for validating each milestone of the SLAM Thesis Experiment Platform.

## Quick Validation

Run all available validation scripts:

```bash
cd ~/thesis
for script in scripts/validate_m*.sh; do
    echo "Running $script..."
    bash "$script"
    echo ""
done
```

---

## M0: Foundation

**Goal**: Workspace structure and packages build successfully.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m0.sh
```

### Manual Checklist

- [ ] Workspace structure exists:
  - [ ] `~/thesis/ros2_ws/src/slam_thesis/` directory exists
  - [ ] All 7 packages present (rover_description, rover_sim, rover_control, gt_publisher, slam_launch, traj_exporter, experiment_runner)
- [ ] ROS2 sourced correctly:
  ```bash
  source /opt/ros/jazzy/setup.bash
  echo $ROS_DISTRO  # Should output: jazzy
  ```
- [ ] Workspace builds without errors:
  ```bash
  cd ~/thesis/ros2_ws
  colcon build
  ```
- [ ] Install directory created with all packages

### Expected Output

```
Starting >>> rover_description
...
Finished <<< experiment_runner
Summary: 7 packages finished
```

---

## M1: Simulation

**Goal**: Gazebo launches with rover and publishes LiDAR scans.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m1.sh
```

### Manual Checklist

- [ ] URDF/Xacro valid:
  ```bash
  cd ~/thesis/ros2_ws
  source install/setup.bash
  xacro src/slam_thesis/rover_description/urdf/rover.urdf.xacro
  ```
- [ ] Simulation launches:
  ```bash
  ros2 launch rover_sim sim.launch.py
  ```
- [ ] Gazebo window opens with rover visible
- [ ] Topics published:
  ```bash
  ros2 topic list | grep -E "^/(scan|odom|clock|tf)$"
  ```
- [ ] LiDAR data streaming:
  ```bash
  ros2 topic hz /scan  # Should show ~10 Hz
  ```
- [ ] TF tree correct:
  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```

### Expected Topics

| Topic | Message Type | Rate |
|-------|--------------|------|
| `/scan` | sensor_msgs/LaserScan | 10 Hz |
| `/odom` | nav_msgs/Odometry | 50 Hz |
| `/clock` | rosgraph_msgs/Clock | ~100 Hz |
| `/tf` | tf2_msgs/TFMessage | 50 Hz |

---

## M2: Control

**Goal**: Rover follows predefined trajectories.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m2.sh
```

### Manual Checklist

- [ ] Trajectory CSV files exist:
  ```bash
  ls ~/thesis/trajectories/*.csv
  ```
- [ ] CSV format valid (x,y,yaw,speed columns):
  ```bash
  head -2 ~/thesis/trajectories/traj_01_easy.csv
  ```
- [ ] Trajectory follower launches:
  ```bash
  # Terminal 1: Start simulation
  ros2 launch rover_sim sim.launch.py

  # Terminal 2: Start trajectory follower
  ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv
  ```
- [ ] Rover moves through waypoints
- [ ] `/cmd_vel` being published:
  ```bash
  ros2 topic echo /cmd_vel --once
  ```
- [ ] `/trajectory_done` published when complete:
  ```bash
  ros2 topic echo /trajectory_done
  ```

### Expected Behavior

1. Rover starts at origin
2. Follows waypoints from CSV
3. Publishes `/trajectory_done: true` when finished

---

## M3: Ground Truth

**Goal**: Ground truth pose published from Gazebo.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m3.sh
```

### Manual Checklist

- [ ] gt_publisher node exists:
  ```bash
  ros2 pkg executables gt_publisher
  ```
- [ ] Ground truth published during simulation:
  ```bash
  # Terminal 1: Start simulation
  ros2 launch rover_sim sim.launch.py

  # Terminal 2: Check ground truth
  ros2 topic echo /gt_pose --once
  ```
- [ ] `/gt_pose` has correct frame_id (`map_gt`):
  ```bash
  ros2 topic echo /gt_pose --once | grep frame_id
  ```
- [ ] TF: map_gt → base_link available:
  ```bash
  ros2 run tf2_ros tf2_echo map_gt base_link
  ```

### Expected Output

```yaml
header:
  frame_id: "map_gt"
child_frame_id: "base_link"
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

---

## M4: Dataset Recording

**Goal**: Record rosbags with all required topics.

### Manual Checklist

- [ ] Recording launch file exists:
  ```bash
  ros2 launch experiment_runner record_dataset.launch.py --show-args
  ```
- [ ] Launch arguments available:
  - [ ] `trajectory` - CSV file name
  - [ ] `condition` - baseline or degraded
  - [ ] `output_dir` - bag output directory
- [ ] Recording works:
  ```bash
  ros2 launch experiment_runner record_dataset.launch.py \
      trajectory:=traj_01_easy.csv \
      condition:=baseline
  ```
- [ ] Bag created with correct topics:
  ```bash
  ros2 bag info ~/thesis/ros2_ws/bags/traj_01_easy__baseline
  ```
- [ ] Required topics present: /scan, /odom, /tf, /gt_pose, /clock

### Required Topics in Bag

| Topic | Required |
|-------|----------|
| `/scan` | Yes |
| `/odom` | Yes |
| `/tf` | Yes |
| `/tf_static` | Yes |
| `/gt_pose` | Yes |
| `/clock` | Yes |

---

## M5: SLAM Pipelines

**Goal**: slam_toolbox and Cartographer work with bag replay.

### Manual Checklist

- [ ] slam_toolbox launch exists:
  ```bash
  ros2 launch slam_launch slam_toolbox.launch.py --show-args
  ```
- [ ] Cartographer launch exists:
  ```bash
  ros2 launch slam_launch cartographer.launch.py --show-args
  ```
- [ ] slam_toolbox runs on bag:
  ```bash
  ros2 launch slam_launch slam_toolbox.launch.py \
      bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline
  ```
- [ ] Cartographer runs on bag:
  ```bash
  ros2 launch slam_launch cartographer.launch.py \
      bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline
  ```
- [ ] TF: map → odom published during SLAM:
  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ```
- [ ] /map topic published:
  ```bash
  ros2 topic echo /map --once
  ```

### Configuration Files

| Algorithm | Config File |
|-----------|-------------|
| slam_toolbox | `slam_launch/config/slam_toolbox.yaml` |
| Cartographer | `slam_launch/config/cartographer_2d.lua` |

---

## M6: Trajectory Export

**Goal**: Export trajectories to TUM format.

### Manual Checklist

- [ ] traj_exporter node exists:
  ```bash
  ros2 pkg executables traj_exporter
  ```
- [ ] Export launch exists:
  ```bash
  ros2 launch traj_exporter export_trajectory.launch.py --show-args
  ```
- [ ] Export works during SLAM replay:
  ```bash
  # Run SLAM with export
  ros2 launch slam_launch slam_toolbox.launch.py \
      bag_path:=~/thesis/ros2_ws/bags/traj_01_easy__baseline &

  ros2 launch traj_exporter export_trajectory.launch.py \
      output_file:=/tmp/test_traj.tum
  ```
- [ ] TUM file created with correct format:
  ```bash
  head -3 /tmp/test_traj.tum
  # Expected: timestamp tx ty tz qx qy qz qw
  ```

### TUM Format Specification

```
# timestamp tx ty tz qx qy qz qw
1234567890.123456789 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
```

---

## M7: Evaluation

**Goal**: Compute ATE/RPE metrics using evo.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m7.sh
```

### Manual Checklist

- [ ] evaluate_run.py exists:
  ```bash
  ls ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/evaluate_run.py
  ```
- [ ] Script has required arguments:
  ```bash
  python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/evaluate_run.py --help
  ```
- [ ] evo library installed:
  ```bash
  python3 -c "import evo; print(evo.__version__)"
  ```
- [ ] Evaluation produces metrics.json:
  ```bash
  python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/evaluate_run.py \
      --gt_file /path/to/gt.tum \
      --est_file /path/to/est.tum \
      --output_dir /tmp/eval_test

  cat /tmp/eval_test/metrics.json
  ```

### Expected metrics.json Structure

```json
{
  "success": true,
  "dataset": "...",
  "algorithm": "...",
  "ate": {"rmse": 0.05, "mean": 0.04, "median": 0.03, "std": 0.02, "min": 0.01, "max": 0.10},
  "rpe": {"rmse": 0.02, "mean": 0.015, ...},
  "trajectory_length_m": 45.0,
  "num_poses": 900
}
```

---

## M8: Experiment Runner

**Goal**: Automated batch experiment execution.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m8.sh
```

### Manual Checklist

- [ ] run_one.py exists and works:
  ```bash
  python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_one.py --help
  ```
- [ ] run_all.py exists and works:
  ```bash
  python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_all.py --help
  ```
- [ ] aggregate_results.py exists and works:
  ```bash
  python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/aggregate_results.py --help
  ```
- [ ] Scripts support required arguments:
  - [ ] `--bag_path` / `--bags_dir`
  - [ ] `--algorithm` / `--algorithms`
  - [ ] `--parallel` (run_all.py)
  - [ ] `--output` (aggregate_results.py)

### Script Purposes

| Script | Purpose |
|--------|---------|
| `run_one.py` | Run single experiment (SLAM + export + evaluate) |
| `run_all.py` | Run batch experiments with parallelization |
| `aggregate_results.py` | Collect all metrics into summary files |

---

## M9: Documentation

**Goal**: Complete project documentation.

### Automated Validation

```bash
bash ~/thesis/scripts/validate_m9.sh
```

### Manual Checklist

- [ ] Top-level README.md exists and is comprehensive:
  ```bash
  ls ~/thesis/README.md
  ```
- [ ] README contains:
  - [ ] Project overview
  - [ ] Quick start guide
  - [ ] Project structure
  - [ ] Usage examples for all workflows
  - [ ] Milestone table with status
  - [ ] Environment requirements
  - [ ] Troubleshooting section
- [ ] VALIDATION.md exists:
  ```bash
  ls ~/thesis/docs/VALIDATION.md
  ```
- [ ] All milestone validation scripts present:
  ```bash
  ls ~/thesis/scripts/validate_m*.sh
  ```

### Documentation Files

| File | Purpose |
|------|---------|
| `README.md` | Main project documentation |
| `docs/VALIDATION.md` | This validation checklist |
| `.claude/ORIENTATION.md` | Claude AI onboarding |
| `.claude/STATE.md` | Current progress |
| `.claude/INTERFACES.md` | Interface contracts |
| `.claude/DECISIONS.md` | Architectural decisions |
| `.claude/RECOVERY.md` | Troubleshooting guide |

---

## Complete System Validation

Run the full experiment pipeline to validate everything works together:

```bash
# 1. Build workspace
cd ~/thesis/ros2_ws
colcon build
source install/setup.bash

# 2. Record a dataset
ros2 launch experiment_runner record_dataset.launch.py \
    trajectory:=traj_01_easy.csv \
    condition:=baseline

# 3. Run experiments
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_all.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --algorithms slam_toolbox cartographer \
    --parallel 1

# 4. Aggregate results
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/aggregate_results.py \
    --bags_dir ~/thesis/ros2_ws/bags \
    --output ~/thesis/ros2_ws/results/summary

# 5. Check results
cat ~/thesis/ros2_ws/results/summary.csv
```

### Success Criteria

- [ ] Dataset recorded successfully
- [ ] Both SLAM algorithms complete
- [ ] Trajectory files exported
- [ ] Metrics computed for all runs
- [ ] Summary CSV/JSON generated
