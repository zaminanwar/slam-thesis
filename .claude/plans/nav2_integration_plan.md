# Nav2 Autonomous Navigation Integration Plan

## Overview

Add Nav2 autonomous navigation as a **new capability** to the existing SLAM comparison thesis. This extends the research question from "which SLAM is more accurate?" to "how does SLAM quality affect autonomous navigation?"

**Key Principle**: All existing functionality (M0-M9) remains unchanged. Nav2 is purely additive.

---

## Architecture Summary

```
┌─────────────────────────────────────────────────────────────┐
│                    EXISTING (UNCHANGED)                      │
│  rover_sim → SLAM (slam_toolbox/cartographer) → metrics     │
│              ↓                                               │
│         map→odom TF                                          │
└─────────────┬───────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────┐
│                    NEW (nav_launch)                          │
│  Nav2 Stack ← SLAM's map→odom ← /scan for costmaps          │
│       ↓                                                      │
│  Autonomous goal-seeking navigation                          │
│       ↓                                                      │
│  Navigation metrics (success rate, time, path efficiency)    │
└─────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Package Creation (M10.1-M10.2)

### Files to Create

```
ros2_ws/src/slam_thesis/nav_launch/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── nav2_params.yaml          # Full Nav2 config (with AMCL)
│   └── nav2_slam_params.yaml     # SLAM-in-the-loop config (no AMCL)
├── launch/
│   ├── nav2_slam.launch.py       # Main: Sim + SLAM + Nav2
│   └── send_goal.launch.py       # Utility: Send single goal
├── behavior_trees/
│   └── .gitkeep
└── resource/
    └── nav_launch
```

### Key Configuration Decisions

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Local planner | DWB | Best for differential drive |
| Global planner | NavFn | Simple, reliable for 2D |
| Robot radius | 0.25m | Based on 0.4x0.3m chassis + inflation |
| Costmap resolution | 0.05m | Matches SLAM grid |
| Controller frequency | 20 Hz | Matches existing cmd_vel rate |
| No AMCL | - | Use SLAM's map→odom directly |

---

## Phase 2: Launch Files (M10.3-M10.5)

### nav2_slam.launch.py Structure

```
t=0s:  Gazebo simulation + GT publisher
t=0s:  SLAM algorithm (conditional: slam_toolbox OR cartographer)
t=5s:  Nav2 stack (delayed for SLAM initialization)
       - lifecycle_manager
       - controller_server
       - planner_server
       - behavior_server
       - bt_navigator
       - waypoint_follower
       - velocity_smoother
```

### Launch Arguments

| Argument | Default | Options |
|----------|---------|---------|
| `algorithm` | `slam_toolbox` | `slam_toolbox`, `cartographer` |
| `world` | `simple.sdf` | Any Gazebo world |
| `use_sim_time` | `true` | Required for simulation |

---

## Phase 3: Experiment Scripts (M11.1-M11.2)

### New Files in experiment_runner/

```
scripts/
└── run_nav_experiment.py    # Navigation experiment orchestrator

config/
├── nav_goals_01.yaml        # Simple square path (4 goals)
└── nav_goals_02.yaml        # Complex exploration (7 goals)
```

### Navigation Goals Format (YAML)

```yaml
goals:
  - x: 3.0
    y: 0.0
    yaw: 0.0
    name: "point_1"
```

### Output: nav_results.json

```json
{
  "algorithm": "slam_toolbox",
  "goals_succeeded": 4,
  "goals_attempted": 4,
  "success_rate": 1.0,
  "total_time_s": 120.5,
  "per_goal_results": [...]
}
```

---

## Phase 4: Documentation Updates

### Files to Update

| File | Changes |
|------|---------|
| `.claude/STATE.md` | Add EPIC 10-11, M10-M11 tasks |
| `.claude/INTERFACES.md` | Add Nav2 topics, action servers, output format |
| `.claude/DECISIONS.md` | Add AD-011 (no AMCL), AD-012 (DWB planner) |
| `.claude/ORIENTATION.md` | Add nav_launch to package table, new commands |

---

## Phase 5: Validation

### validate_m10.sh
- Package builds successfully
- Config files installed
- Launch file parseable
- Nav2 dependencies present

### validate_m11.sh
- Experiment script exists
- Goal files exist
- Documentation updated

---

## Implementation Order

1. **Install Nav2** (prerequisite)
   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
   ```

2. **Create nav_launch package** (Phase 1)
   - package.xml, CMakeLists.txt
   - nav2_slam_params.yaml
   - Build and verify

3. **Implement launch files** (Phase 2)
   - nav2_slam.launch.py
   - send_goal.launch.py
   - Manual testing with both SLAM algorithms

4. **Create experiment infrastructure** (Phase 3)
   - run_nav_experiment.py
   - Goal YAML files

5. **Update documentation** (Phase 4)
   - All .claude/ files

6. **Validation scripts** (Phase 5)
   - validate_m10.sh, validate_m11.sh

---

## Verification Steps

### After Phase 1-2 (M10):
```bash
# Build
cd ~/thesis/ros2_ws && colcon build --packages-select nav_launch

# Verify package
ros2 pkg list | grep nav_launch

# Test launch (should start without errors)
ros2 launch nav_launch nav2_slam.launch.py algorithm:=slam_toolbox

# Send a test goal (in another terminal)
ros2 launch nav_launch send_goal.launch.py x:=2.0 y:=1.0 yaw:=0.0
```

### After Phase 3-5 (M11):
```bash
# Run navigation experiment
python3 ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_nav_experiment.py \
  --algorithm slam_toolbox \
  --goals ~/thesis/ros2_ws/src/slam_thesis/experiment_runner/config/nav_goals_01.yaml

# Check results
cat ~/thesis/ros2_ws/results/nav_slam_toolbox_*/nav_results.json

# Run validation
~/thesis/scripts/validate_m10.sh
~/thesis/scripts/validate_m11.sh
```

---

## Context Continuity

The `.claude/` documentation system ensures any future Claude instance can:

1. **Read ORIENTATION.md** → Understand nav_launch exists
2. **Read STATE.md** → See M10/M11 progress
3. **Read INTERFACES.md** → Know Nav2 topics and output formats
4. **Read DECISIONS.md** → Understand why no AMCL, why DWB
5. **Run validation scripts** → Verify everything works

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| Nav2 TF timing | 5s delay before Nav2 starts; transform_tolerance=1.0 |
| SLAM not ready | Nav2 waits for map frame via lifecycle |
| WSL2 performance | Headless Gazebo, 0.05m costmap resolution |
| Goal in obstacle | NavFn allows_unknown=true, inflation_radius=0.55 |

---

## Files Summary (16 total)

### New Files (10):
1. `nav_launch/package.xml`
2. `nav_launch/CMakeLists.txt`
3. `nav_launch/config/nav2_params.yaml`
4. `nav_launch/config/nav2_slam_params.yaml`
5. `nav_launch/launch/nav2_slam.launch.py`
6. `nav_launch/launch/send_goal.launch.py`
7. `experiment_runner/scripts/run_nav_experiment.py`
8. `experiment_runner/config/nav_goals_01.yaml`
9. `experiment_runner/config/nav_goals_02.yaml`
10. `scripts/validate_m10.sh`
11. `scripts/validate_m11.sh`

### Updated Files (4):
1. `.claude/STATE.md`
2. `.claude/INTERFACES.md`
3. `.claude/DECISIONS.md`
4. `.claude/ORIENTATION.md`
