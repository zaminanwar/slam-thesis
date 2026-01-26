# Quick Reference: Next Steps

**For new Claude context: Read STATE.md for full details. This is the TL;DR.**

## Current Task
**Phase 3**: Add Missing Metrics (CPU, odom delta, CTE, loop closures)

## What's Needed
Additional metrics for thesis analysis:
- Odometry vs SLAM delta (show SLAM correction value)
- Cross-track error (path following accuracy)
- Loop closure count (SLAM quality indicator)
- CPU/memory usage (computational cost)
- Trajectory completion % (task success)

## Phase 3 Tasks

| Task | File | What to Do | Effort |
|------|------|------------|--------|
| 3.1 | traj_exporter_node.py | Add /odom subscription, compute SLAM vs odom delta | 2h |
| 3.2 | NEW: compute_cte.py | Post-process actual vs intended path deviation | 3h |
| 3.3 | NEW: loop_closure_monitor.py | Subscribe to SLAM constraint topics | 4h |
| 3.4 | run_one.py | Add psutil CPU/memory monitoring | 2h |
| 3.5 | evaluate_run.py | Add trajectory completion rate metric | 1h |
| 3.6 | aggregate_results.py | Add new metrics to CSV output | 2h |

## Key Files
```
~/thesis/ros2_ws/src/slam_thesis/traj_exporter/traj_exporter/traj_exporter_node.py
~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/run_one.py
~/thesis/ros2_ws/src/slam_thesis/experiment_runner/scripts/evaluate_run.py
```

## Phase Summary
| Phase | What | Status |
|-------|------|--------|
| 1 | Fix blockers (pose_mode in run_one/run_all) | ✅ DONE |
| 2 | Simplify trajectory_follower.py (720→527 lines) | ✅ DONE |
| 3 | Add metrics (CPU, odom delta, CTE, loops) | ⬜ DO NOW |
| 4 | Run Parts A & B experiments | ⬜ |
| 5 | Nav2 integration | ⬜ |
| 6 | Run Part C experiments | ⬜ |

## Full Details
See [STATE.md](STATE.md) for complete implementation plan.
