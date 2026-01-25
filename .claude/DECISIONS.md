# Architectural Decisions

This document records all significant architectural decisions with rationale.
New Claude instances should read this to understand WHY things are done a certain way.

---

## AD-001: Gazebo Version Based on ROS Distro

**Date**: 2026-01-24
**Status**: SUPERSEDED (updated for multi-distro support)
**Decider**: User confirmed, updated due to Ubuntu 24.04 detection

### Context
Need to choose simulation backend. User has Ubuntu 24.04, which requires ROS2 Jazzy.
- Ubuntu 22.04 + ROS2 Humble → Gazebo Classic (gazebo11)
- Ubuntu 24.04 + ROS2 Jazzy → Gazebo Harmonic (gz-sim)

### Decision
Use **distro-appropriate Gazebo**:
- **Humble**: Gazebo Classic via `gazebo_ros_pkgs`
- **Jazzy**: Gazebo Harmonic via `ros_gz`

### Rationale
1. Gazebo Classic is deprecated and not available for Ubuntu 24.04
2. ROS2 Jazzy is the LTS for Ubuntu 24.04
3. Gazebo Harmonic is the modern replacement with full ROS2 support
4. Launch files will detect distro and use appropriate APIs

### Consequences for Jazzy (Ubuntu 24.04)
- Uses `ros_gz` packages instead of `gazebo_ros_pkgs`
- Ground truth via `ros_gz_bridge` topic bridging
- World files use SDF format (same as before)
- Launch commands use `gz sim` instead of `gazebo`
- Spawn uses `ros_gz_sim` create service

### Consequences for Humble (Ubuntu 22.04)
- Uses `gazebo_ros_pkgs` (original plan)
- Ground truth via `/gazebo/model_states`
- Launch uses `gazebo` command

### Detection
ROS distro is saved to `~/thesis/.ros_distro` during installation.
Launch files read this to determine which simulator APIs to use.

---

## AD-002: Pure Pursuit for Trajectory Following

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need deterministic trajectory controller for predefined waypoints.

### Decision
Use **Pure Pursuit algorithm**

### Rationale
1. Simple, well-understood, deterministic given same inputs
2. No tuning complexity of MPC or other advanced controllers
3. Sufficient for predefined collision-free paths
4. Easy to implement in Python
5. Predictable behavior aids reproducibility

### Consequences
- Lookahead distance is key tuning parameter (default: 0.5m)
- May cut corners on sharp turns (acceptable for our trajectories)
- Speed is controlled separately from steering

### Parameters
| Parameter | Default | Range |
|-----------|---------|-------|
| lookahead_distance | 0.5m | 0.3-1.0m |
| goal_tolerance | 0.2m | 0.1-0.5m |

---

## AD-003: TUM Format for Trajectory Export

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need trajectory format for evo evaluation tool.

### Decision
Use **TUM trajectory format**

### Rationale
1. Native support in evo toolbox
2. Human-readable (space-separated text)
3. Industry standard for trajectory evaluation
4. Simple: `timestamp tx ty tz qx qy qz qw`

### Consequences
- Must convert ROS quaternions correctly (ROS uses xyzw order internally, TUM uses xyzw in file)
- Timestamps must be monotonically increasing
- Z coordinate will be ~0 for 2D SLAM (that's fine)

### Format Specification
```
# timestamp tx ty tz qx qy qz qw
1706123456.789000 1.234 5.678 0.0 0.0 0.0 0.123 0.992
```

---

## AD-004: Separate map_gt Frame for Ground Truth

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need ground truth pose that doesn't interfere with SLAM's TF tree.

### Decision
Use **`map_gt`** as ground truth frame, completely separate from SLAM's `map`

### Rationale
1. SLAM algorithms own the `map` frame - we must not interfere
2. Ground truth must be available even when SLAM hasn't initialized
3. Allows simultaneous visualization of GT and estimated paths
4. Clear semantic separation: `map` = estimate, `map_gt` = truth

### Consequences
- `gt_publisher` broadcasts `map_gt → base_link`
- `traj_exporter` looks up both `map_gt→base_link` (GT) and `map→base_link` (estimate)
- RViz can display both in same view using different fixed frames
- Two separate TF trees that share `base_link` as common child

### TF Tree Visualization
```
Tree 1 (Ground Truth):     Tree 2 (SLAM):
    map_gt                     map
      │                          │
      └── base_link              └── odom
                                       │
                                       └── base_link
```

---

## AD-005: Rosbag Replay for Fair SLAM Comparison

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need to ensure both SLAM algorithms receive identical sensor data.

### Decision
Use **"Record Once, Replay Many"** pattern with rosbag2

### Rationale
1. Guarantees identical sensor streams for both algorithms
2. Decouples dataset generation from evaluation
3. Allows re-running evaluations without re-running simulation
4. Industry-standard approach for SLAM benchmarking
5. Bags are portable and shareable

### Consequences
- Must record all required topics: /scan, /odom, /tf, /tf_static, /clock, /gt_pose
- Replay must use `--clock` flag for proper time synchronization
- All evaluation nodes must use `use_sim_time:=true`
- Slight overhead from bag I/O (negligible for our data rates)

---

## AD-006: evo Toolbox for Evaluation Metrics

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need to compute ATE and RPE metrics from trajectory files.

### Decision
Use **evo** (https://github.com/MichaelGrupp/evo)

### Rationale
1. De facto standard for trajectory evaluation in robotics
2. Supports TUM format natively
3. Provides both metrics and visualization
4. Well-documented, actively maintained
5. Command-line interface suitable for scripting

### Consequences
- Install via pip: `pip3 install evo`
- Use `evo_ape` for ATE (Absolute Pose Error)
- Use `evo_rpe` for RPE (Relative Pose Error)
- Must document alignment policy for reproducibility

### Evaluation Commands
```bash
# ATE with SE3 alignment
evo_ape tum gt.tum est.tum --align --correct_scale

# RPE per meter
evo_rpe tum gt.tum est.tum --delta 1 --delta_unit m --align
```

---

## AD-007: Python for ROS2 Nodes (Default)

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Choose implementation language for ROS2 nodes.

### Decision
Use **Python** for all nodes unless performance demands C++

### Rationale
1. Faster development iteration
2. Easier debugging
3. No compilation step for quick changes
4. Performance sufficient for our sensor rates (~10-50 Hz)
5. Better for thesis timeline

### Consequences
- All packages use `ament_python` build type
- Nodes are Python scripts with entry points in setup.py
- May need C++ only if profiling shows bottleneck (unlikely)

---

## AD-008: Condition System via Parameters

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need to implement experiment conditions (baseline, odom_degraded, high_speed).

### Decision
Implement conditions via **ROS2 parameters**, not separate node implementations

### Rationale
1. Single codebase, multiple behaviors
2. Easy to add new conditions
3. Parameters can be set at launch time
4. Clear traceability in bag metadata

### Implementation
| Condition | Parameters Changed |
|-----------|-------------------|
| baseline | All defaults |
| odom_degraded | odom_noise_std=0.05, odom_drift_rate=0.01 |
| high_speed | speed_scale=1.5 |

---

## AD-009: Milestone-Based Validation

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
Need to verify work is correct before proceeding.

### Decision
Create **validation scripts per milestone** that programmatically verify success criteria

### Rationale
1. Enables new Claude instances to verify prior work
2. Catches regressions early
3. Provides clear definition of "done"
4. Reduces debugging time

### Implementation
- Scripts live in `~/thesis/scripts/validate_m*.sh`
- Each script checks topics, TF, file existence, etc.
- Exit code 0 = pass, non-zero = fail
- Human-readable output for debugging

---

## AD-010: ParameterValue Wrapper for robot_description in Jazzy

**Date**: 2026-01-24
**Status**: ACCEPTED

### Context
ROS 2 Jazzy changed how launch parameters are parsed. Using `Command(['xacro ', path])`
directly for `robot_description` parameter causes launch to fail with:
```
Unable to parse the value of parameter robot_description as yaml
```

### Decision
Wrap all `robot_description` parameters with `ParameterValue(..., value_type=str)`

### Rationale
1. Jazzy's launch system tries to parse parameter values as YAML by default
2. URDF/Xacro output is XML, not YAML, causing parse failure
3. `ParameterValue` with `value_type=str` forces string interpretation
4. This is the officially recommended fix for Jazzy

### Implementation
```python
from launch_ros.parameter_descriptions import ParameterValue

robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
```

### Files Affected
- `rover_description/launch/display.launch.py`
- `rover_sim/launch/sim.launch.py`

---

## Template for New Decisions

```markdown
## AD-XXX: [Title]

**Date**: YYYY-MM-DD
**Status**: PROPOSED | ACCEPTED | DEPRECATED | SUPERSEDED
**Superseded by**: AD-YYY (if applicable)

### Context
[What is the issue we're addressing?]

### Decision
[What did we decide?]

### Rationale
[Why did we make this decision?]

### Consequences
[What are the implications?]
```
