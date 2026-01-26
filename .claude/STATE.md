# Project State - SLAM Thesis Implementation

**Last Updated**: 2026-01-26T02:30:00
**Last Claude Instance**: Loop Closure Investigation (Opus 4.5)
**Current Phase**: Phase 6 - LOOP CLOSURE INTERFERENCE IDENTIFIED
**Next Action**: Run with `--pose_mode odometry` to bypass loop closure issue

---

## QUICK START FOR NEW CONTEXT

**Read this section first. It tells you exactly what to do next.**

### Current Status: LOOP CLOSURE INTERFERENCE WITH CONTROL

**PROBLEM IDENTIFIED**: Robot completes ~89% of traj_02_loop but gets stuck at the final corner (0, 4) when returning to origin.

**ROOT CAUSE**: Loop closure interference with SLAM-based control
- When robot revisits (0, 4) area (previously seen at trajectory start)
- Cartographer's loop closure detects the revisited location
- High loop closure weights (28,176 translation, 454,522 rotation) cause sudden pose corrections
- SLAM pose "jumps" confuse the controller, causing it to spin in place

**WHAT WE TRIED**:
| Change | ATE RMSE | Completion | Result |
|--------|----------|------------|--------|
| Dense world (20 obstacles) | 1.4 cm | 88.9% | Much better SLAM, still stuck |
| + Smooth arc turns | 1.1 cm | 89.5% | Slightly better, still stuck |
| + Lookahead 0.1m | (interrupted) | - | Loop closure identified as cause |

**KEY INSIGHT**: SLAM accuracy is excellent (1.1-1.4 cm ATE). The problem is NOT SLAM drift - it's loop closure corrections causing pose jumps that the controller can't handle.

### What To Do Next

**RUN WITH ODOMETRY MODE** (recommended):
```bash
cd ~/thesis/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/slam_thesis/experiment_runner/scripts/run_one.py \
  --algo cartographer --trajectory traj_02_loop \
  --world simple_dense.sdf --pose_mode odometry \
  --timeout 400 --verbose
```

This provides:
- Consistent control baseline (odometry doesn't have loop closure jumps)
- Fair SLAM evaluation (loop closure still happens, recorded in est.tum)
- Comparable results across algorithms

---

## CHANGES MADE THIS SESSION (M9 Commit)

### 1. Optuna Tuning System Added
Location: `ros2_ws/src/slam_thesis/experiment_runner/scripts/tuning/`
- `config_generators.py` - Research-based baselines with ±10% tuning ranges
- `optuna_tuner.py` - Multi-phase tuning (coarse → refine → validate)
- Cartographer tuning achieved 0.73 cm ATE on short trajectories

### 2. Dense Obstacle World Created
File: `ros2_ws/src/slam_thesis/rover_sim/worlds/simple_dense.sdf`
- 20 obstacles distributed throughout room
- Outer edges (between trajectory and walls): 9 obstacles at x=±4.5, y=±4.5
- Interior quadrants: 8 obstacles
- Center area: 2 obstacles
- Significantly reduces SLAM drift in corridor environments

### 3. Smooth Arc Trajectories
File: `trajectories/traj_02_loop.csv`
- Sharp 90° corners replaced with gradual arc waypoints
- Each turn has 4-5 intermediate points along ~0.3m radius arc
- Speed reduced to 0.15 m/s during turns

### 4. Controller Updates
File: `rover_control/rover_control/trajectory_follower.py`
- lookahead_distance: 1.5m → 0.5m → 0.1m (reduced for tighter turns)

File: `experiment_runner/scripts/run_one.py`
- Added `--world` parameter to specify world file

---

## KEY FILES

### Trajectory Files
```
~/thesis/trajectories/
├── traj_micro_tune.csv       # ~5m - for tuning
├── traj_01_easy.csv          # 16m - left side only, works
├── traj_01_mirror.csv        # 16m - right side, works
├── traj_02_loop.csv          # 40m - full perimeter, NOW HAS SMOOTH TURNS
├── traj_02_loop_original.csv # 40m - backup with sharp corners
├── traj_03_complex.csv       # 56m - figure-8
└── traj_tune_long_straight.csv # 12m straight for tuning
```

### World Files
```
~/thesis/ros2_ws/src/slam_thesis/rover_sim/worlds/
├── simple.sdf          # Original - 5 obstacles
└── simple_dense.sdf    # NEW - 20 obstacles for better SLAM features
```

### Experiment Scripts
```
~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/
├── run_one.py              # Single experiment (has --world param now)
├── run_all.py              # Batch runner
├── evaluate_run.py         # ATE/RPE calculation
├── aggregate_results.py    # Summary generation
└── tuning/                 # NEW - Optuna tuning system
    ├── config_generators.py
    ├── optuna_tuner.py
    └── visualize_tuning.py
```

---

## CURRENT PARAMETERS

### Trajectory Follower
```python
lookahead_distance = 0.1    # meters (reduced from 1.5)
goal_tolerance = 0.4        # meters
max_angular_velocity = 0.8  # rad/s
```

### Cartographer (Tuned - from optuna)
```lua
motion_filter_max_distance = 0.055
ceres_translation_weight = 107.9
loop_closure_translation_weight = 28175.9  -- HIGH - causes pose jumps
loop_closure_rotation_weight = 454522.7    -- VERY HIGH
optimize_every_n_nodes = 23
```

---

## TUNING RESULTS

| Algorithm | Validated ATE | Std Dev | Config Location |
|-----------|---------------|---------|-----------------|
| **Cartographer** | **0.73 cm** | ± 0.02 cm | `results/tuning/cartographer_tuning_*/` |
| SLAM Toolbox | 1.77 cm | ± 0.24 cm | `results/tuning/slam_toolbox_tuning_*/` |

---

## TEST RESULTS WITH DENSE WORLD

| Configuration | ATE RMSE | Completion | Notes |
|--------------|----------|------------|-------|
| SLAM pose + 1.5m lookahead | 1.40 cm | 88.9% | Stuck at (0,4) turn |
| SLAM pose + smooth turns | 1.13 cm | 89.5% | Still stuck at (0,4) |
| **Odometry pose** | TBD | TBD | **Try this next** |

---

## ENVIRONMENT

- **Ubuntu**: 24.04
- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-sim)
- **Python**: 3.12
- **Simulation speed**: 1.0x real-time

---

## COMMANDS

### Run experiment with odometry control (RECOMMENDED NEXT)
```bash
cd ~/thesis/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/slam_thesis/experiment_runner/scripts/run_one.py \
  --algo cartographer --trajectory traj_02_loop \
  --world simple_dense.sdf --pose_mode odometry \
  --timeout 400 --verbose
```

### Run Optuna tuning
```bash
python3 src/slam_thesis/experiment_runner/scripts/tuning/optuna_tuner.py \
  --algorithm cartographer --mode quick
```

### Rebuild packages
```bash
colcon build --packages-select rover_control rover_sim experiment_runner --symlink-install
```

---

## WHAT NEXT SESSION SHOULD DO

1. **Run with odometry mode** to verify trajectory completes without loop closure interference

2. **If successful**, run comparison experiments:
   - Both algorithms (cartographer, slam_toolbox)
   - Dense world (simple_dense.sdf)
   - Odometry control mode
   - Multiple trajectories (traj_01_easy, traj_02_loop, traj_03_complex)

3. **Alternative**: If odometry mode works, could also try:
   - Reducing loop closure weights in Cartographer config
   - Adding pose smoothing to filter out sudden jumps
