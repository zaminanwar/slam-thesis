# Project State - SLAM Thesis Implementation

**Last Updated**: 2026-01-26T02:00:00
**Last Claude Instance**: Trajectory Corner Turn Debugging (Opus 4.5)
**Current Phase**: Phase 6 - DEBUGGING CORNER TURN ISSUE
**Next Action**: Fix robot not turning at (4, -4) corner

---

## QUICK START FOR NEW CONTEXT

**Read this section first. It tells you exactly what to do next.**

### Current Status: DEBUGGING CORNER TURN ISSUE

**PROBLEM**: Robot fails to turn at waypoint (4, -4) in traj_02_loop
- Robot travels down right side (y: 4 → -4) at x=4
- At corner (4, -4), robot should turn LEFT toward (2, -4)
- Instead, robot keeps going STRAIGHT and hits SOUTH WALL (y = -5)

**WHAT WORKS vs FAILS**:
| Trajectory | Path | Result |
|------------|------|--------|
| traj_01_easy | Left side only (x ≤ 0), 16m | ✅ Works |
| traj_01_mirror | Right side, short (4m at x=4), 16m | ✅ Works |
| traj_02_loop | Full perimeter, long (8m at x=4), 40m | ❌ Fails at (4,-4) |

**KEY INSIGHT**: traj_01_mirror goes to x=4 and turns at (4,0) successfully.
traj_02_loop goes to x=4 but fails to turn at (4,-4) after 8m of travel.

**ATTEMPTED FIXES (didn't work)**:
1. Scaled trajectory to ±3.7m - still fails
2. Reduced speed from 0.3 to 0.2 m/s - still fails
3. Increased goal_tolerance from 0.15m to 0.4m - still fails

**SUSPECTED ROOT CAUSE**:
The Pure Pursuit controller or waypoint advancement logic has an issue with:
- Long straight segments before corners
- OR the specific geometry of the (4,-4) corner
- OR SLAM pose lag after extended travel

### What Needs To Be Done

**DEBUG THE TURN FAILURE**:
1. Add logging to see what the controller thinks is happening at (4,-4)
2. Check if waypoint index advances (does it think it reached (4,-4)?)
3. Check the angle calculation to next waypoint (2,-4)
4. Consider if lookahead_distance (0.5m) is appropriate

**FILES TO INVESTIGATE**:
- `rover_control/rover_control/trajectory_follower.py` - Pure Pursuit logic
- Lines 453-470: waypoint advancement
- Lines 354-436: _pure_pursuit() turn calculation
- Line 56: goal_tolerance (changed to 0.4m)

**CURRENT TRAJECTORY FILE** (traj_02_loop.csv is at ±4m scale):
```
...
4.0,-2.0,-1.5708,0.2   # approaching corner
4.0,-4.0,3.1416,0.2    # THE PROBLEM CORNER - should turn left here
2.0,-4.0,3.1416,0.2    # should go here but robot goes straight instead
...
```

---

## CHANGES MADE THIS SESSION

### Goal Tolerance Increased
File: `rover_control/rover_control/trajectory_follower.py`
- Changed `goal_tolerance` from 0.15m to 0.4m (line 56)
- Rebuilt with `colcon build --packages-select rover_control`
- **Did not fix the issue**

### Trajectory Testing
- traj_02_loop.csv currently at ±4m scale (reverted from ±3.7m for testing)
- Created traj_01_mirror.csv (mirrors traj_01 to +X side) - this WORKS
- Created traj_02_loop_original.csv for testing

### Previous: Trajectories Scaled to 0.925x (not currently active)
The 0.925x scaling was validated as obstacle-safe but didn't fix the turn issue.

### Previous Session: RTF Changed to 1.0
Files modified (already rebuilt):
- `rover_sim/worlds/simple.sdf`: `real_time_factor` 10.0 → 1.0
- `rover_sim/worlds/empty_room.sdf`: `real_time_factor` 10.0 → 1.0

This means experiments now run at real-time (not 10x). A 37m trajectory at 0.3 m/s takes ~123s real time.

---

## TUNING RESULTS (STILL VALID)

Tuning was done on traj_01_easy (16m) which works fine:

| Algorithm | Validated ATE | Std Dev |
|-----------|---------------|---------|
| **Cartographer** | **0.54 cm** | ± 0.17 cm |
| SLAM Toolbox | 1.77 cm | ± 0.24 cm |

Best configs are saved and symlinked:
```
~/thesis/ros2_ws/results/tuning/slam_toolbox_tuning_20260125_210455/best_slam_toolbox_config.yaml
~/thesis/ros2_ws/results/tuning/cartographer_tuning_20260125_213727/best_cartographer_config.lua
```

---

## KEY FILES

### Trajectory Files (debugging state)
```
~/thesis/trajectories/
├── traj_micro_tune.csv       # ~5m - OK (small, for tuning)
├── traj_01_easy.csv          # 16m - OK (left side only)
├── traj_01_mirror.csv        # 16m - OK (right side, short segment) ← NEW, WORKS
├── traj_02_loop.csv          # 40m - FAILING at (4,-4) corner (currently ±4m scale)
├── traj_02_loop_original.csv # 40m - backup of original
└── traj_03_complex.csv       # 56m - not tested yet (still at ±3.7m scale)
```

### World Files (walls at ±5m)
```
~/thesis/ros2_ws/src/slam_thesis/rover_sim/worlds/simple.sdf
~/thesis/ros2_ws/src/slam_thesis/rover_sim/worlds/empty_room.sdf
```

### Experiment Scripts
```
~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/
├── run_one.py              # Single experiment
├── run_all.py              # Batch runner
├── evaluate_run.py         # ATE/RPE calculation
└── aggregate_results.py    # Generates summary.csv
```

---

## CRITICAL KNOWLEDGE

### Trajectory Format
```csv
x,y,yaw,speed
0.0,0.0,1.5708,0.3
0.0,2.0,1.5708,0.3
...
```
- Coordinates in meters
- Yaw in radians
- Speed in m/s

### Room Dimensions
- Walls at x = ±5m, y = ±5m
- Safe trajectory zone: ±3m to ±3.5m (leave 1.5-2m margin)

### pose_mode parameter
- `--pose_modes slam` → Uses SLAM-corrected poses (has some drift)
- `--pose_modes ground_truth` → Uses perfect poses (no drift, for debugging)
- `--pose_modes odometry` → Uses raw wheel odometry (lots of drift)

---

## ENVIRONMENT

- **Ubuntu**: 24.04
- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-sim)
- **Python**: 3.12
- **Simulation speed**: 1.0x real-time (changed from 10x)

---

## WHAT NEXT SESSION SHOULD DO

1. **Debug why robot doesn't turn at (4,-4)**:
   - Add debug logging to trajectory_follower.py to see:
     - Current waypoint index when approaching corner
     - Distance to next waypoint
     - Calculated angle to target
     - Whether it enters rotate-in-place mode (Zone 1)

2. **Potential fixes to try**:
   - Increase lookahead_distance from 0.5m to 1.0m
   - Add explicit corner handling in the controller
   - Use a different waypoint advancement strategy (look-ahead based vs distance based)

3. **Once turn issue is fixed**: Run final comparison experiments
