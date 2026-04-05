# Thesis Outline

**Working Title**: Navigation-Driven Evaluation of 2D LiDAR SLAM: A Comparative Study of slam_toolbox and Cartographer in Autonomous Rover Missions

---

## Chapter 1: Introduction (~8-10 pages)

### 1.1 Motivation
- Mobile robot autonomy depends on accurate self-localization
- SLAM is the de-facto solution; many algorithms exist with different trade-offs
- Practitioners face choice paralysis: which SLAM to deploy?
- Academic benchmarks emphasize trajectory accuracy (ATE/RPE) but rarely test under closed-loop autonomous navigation

### 1.2 Problem Statement
- How do 2D LiDAR SLAM algorithms compare when evaluated through **task performance** (autonomous navigation) rather than **raw trajectory accuracy**?
- Does better SLAM accuracy translate to better navigation success?
- Under what conditions do SLAM accuracy differences become navigation-relevant?

### 1.3 Research Questions
- **RQ1**: How do slam_toolbox and Cartographer compare in SLAM accuracy (ATE/RPE) across trajectories of varying complexity?
- **RQ2**: Does SLAM accuracy predict navigation success rate and efficiency in closed-loop Nav2 missions?
- **RQ3**: How do the algorithms scale as trajectory length and complexity increase?
- **RQ4**: Which algorithm is more suitable for deployment, and under what conditions?

### 1.4 Contributions
- A reproducible experimental framework for SLAM-in-the-loop Nav2 evaluation
- Quantitative comparison across 3 trajectory complexity levels
- Analysis of the gap between trajectory accuracy and navigation performance
- Open-source code and datasets

### 1.5 Thesis Structure

---

## Chapter 2: Background & Related Work (~15-20 pages)

### 2.1 Simultaneous Localization and Mapping (SLAM)
- Problem definition
- Pose graph SLAM vs filter-based SLAM
- 2D LiDAR SLAM landscape

### 2.2 The Algorithms Under Study
- **2.2.1 slam_toolbox** (Macenski & Jambrecic, 2021) — architecture, Ceres solver, loop closure
- **2.2.2 Cartographer** (Hess et al., 2016) — real-time loop closure, submap-based approach
- Side-by-side algorithmic comparison table

### 2.3 Autonomous Navigation & Nav2
- Nav2 architecture (planner, controller, behavior tree)
- SLAM-in-the-loop vs static map localization
- DWB local planner

### 2.4 SLAM Evaluation Methodology
- **2.4.1 ATE/RPE metrics** (Sturm et al., 2012)
- **2.4.2 Trajectory alignment** (Umeyama 1991, Zhang & Scaramuzza tutorial 2018)
- **2.4.3 TUM trajectory format & evo library**

### 2.5 Related Benchmarking Work
- TUM RGB-D benchmark
- KITTI odometry benchmark
- SLAM Hive (Wisth et al., 2024)
- Prior slam_toolbox vs Cartographer comparisons
- Gap: most benchmarks evaluate trajectories open-loop; few test navigation impact

---

## Chapter 3: Methodology (~10-15 pages)

### 3.1 Simulation Platform
- **3.1.1 Gazebo Harmonic + ROS 2 Jazzy** (rationale for choice)
- **3.1.2 Simulated rover** (differential drive, URDF, 2D LiDAR sensor)
- **3.1.3 Simulation world** (simple.sdf, 10x10m bounded environment)

### 3.2 Ground Truth System
- Gazebo model pose -> /model/rover/pose
- gt_publisher node architecture
- TF frame design: separate `map_gt -> base_footprint_gt` tree avoids conflicts with SLAM's `map -> odom -> base_footprint`
- Synchronization via `use_sim_time`

### 3.3 SLAM Algorithm Configuration
- **3.3.1 slam_toolbox config** (async mode, Ceres solver, 5cm resolution, published defaults + robot-specific frames)
- **3.3.2 Cartographer config** (trajectory builder, loop closure weights, published defaults)
- **3.3.3 Fairness**: Both use published default parameters adjusted only for rover-specific geometry

### 3.4 Navigation Stack
- Nav2 SLAM-in-the-loop architecture (no AMCL)
- Component selection: NavfnPlanner, DWB local planner, default behavior trees
- Lifecycle management with SLAM as first-activated node

### 3.5 Experimental Design
- **3.5.1 Trajectory/goal set design** (3 difficulty levels)
  - nav_goals_01: 4-waypoint square (~12m)
  - nav_goals_02: 7-waypoint exploration (~15m)
  - nav_goals_03: 11-waypoint perimeter with revisits (~35m)
- **3.5.2 Rationale**: tests scaling with length, turning frequency, loop closure opportunity

### 3.6 Metrics
- **3.6.1 Navigation metrics**: success rate, time-per-goal, total mission time
- **3.6.2 SLAM metrics**: ATE (RMSE, mean, median, std), RPE (RMSE, mean)
- **3.6.3 Collection pipeline**: traj_exporter nodes -> TUM files -> evo library

### 3.7 Experimental Protocol
- [N] repetitions per (algorithm x goal_set) combination
- Startup delay (25s) for system initialization
- Automated orchestration via run_nav_experiment.py
- Clean environment between runs

---

## Chapter 4: Implementation (~8-10 pages)

### 4.1 System Architecture Overview (diagram)

### 4.2 Custom ROS 2 Packages
- rover_description (URDF)
- rover_sim (Gazebo launch)
- gt_publisher (ground truth)
- slam_launch (SLAM configs)
- traj_exporter (TUM trajectory export)
- experiment_runner (orchestration)
- nav_launch (Nav2 integration)

### 4.3 Key Technical Challenges
- **4.3.1 TF tree conflicts** (two-parent problem, resolution via base_footprint_gt)
- **4.3.2 Lifecycle node ordering** (SLAM must activate before Nav2 costmap)
- **4.3.3 Behavior tree configuration** (default BT XML paths)
- **4.3.4 Trajectory exporter resilience** (ConnectivityException handling)

### 4.4 Reproducibility
- Open-source GitHub repository
- Automated build and launch
- Docker/WSL setup documentation

---

## Chapter 5: Results (~15-20 pages)

### 5.1 Experimental Overview
- N runs per config, total experiments run
- Hardware/compute specs

### 5.2 Navigation Success
- **Table**: Success rate per (algo x goal_set), mean +/- std
- Both achieve 100% success -> first key finding

### 5.3 Mission Time Analysis
- **Table**: Mean +/- std completion time per config
- **Figure**: Bar chart comparing times across algorithms and goal sets
- Observation: slam_toolbox ~20-30% faster

### 5.4 SLAM Accuracy: Absolute Trajectory Error
- **Table**: ATE RMSE/mean/std per config
- **Figure**: Bar chart (slam_toolbox vs cartographer x 3 goal sets)
- **Figure**: Trajectory overlay plots (4-6 selected runs, side-by-side)
- slam_toolbox consistently 5-12x better

### 5.5 SLAM Accuracy: Relative Pose Error
- **Table**: RPE RMSE per config
- Discussion: RPE gap is smaller than ATE gap -> local accuracy is comparable; difference is global drift

### 5.6 Scaling Behavior
- **Figure**: ATE vs. trajectory length (scatter/line)
- Linear drift growth for both algorithms
- slam_toolbox starts lower AND has shallower slope

### 5.7 Per-Goal Analysis
- Which waypoints were hardest?
- Navigation time outliers
- Correlation with goal distance/turning angle

---

## Chapter 6: Discussion (~10-15 pages)

### 6.1 Answering the Research Questions
- **RQ1**: slam_toolbox is more accurate (5-12x lower ATE) across all tested conditions
- **RQ2**: Navigation success does NOT require high SLAM accuracy; 10+ cm ATE still yields 100% success in these environments
- **RQ3**: Both algorithms degrade roughly linearly with distance, but from different starting points
- **RQ4**: slam_toolbox preferable for accuracy-critical tasks; Cartographer may suffice for coarse navigation

### 6.2 Why does slam_toolbox outperform Cartographer?
- Hypothesis 1: default parameter suitability for smaller ground robots
- Hypothesis 2: global pose graph optimization vs. submap-based approach
- Hypothesis 3: loop closure weights

### 6.3 The Accuracy-Utility Gap
- 12x better ATE yields 0% improvement in success rate
- What ATE threshold becomes task-relevant?
- Implications for SLAM benchmark design

### 6.4 Limitations
- **6.4.1 Simulation-only** (no sensor/odom noise, no wheel slip)
- **6.4.2 Single environment** (simple world, no corridors/clutter)
- **6.4.3 Default parameters** (no algorithm-specific tuning)
- **6.4.4 Simulation fidelity** (Gazebo physics != real world)
- **6.4.5 Single Nav2 config** (DWB planner only)
- **6.4.6 Limited repetitions** (statistical power)

### 6.5 Threats to Validity
- Internal, external, construct validity

---

## Chapter 7: Conclusions & Future Work (~5-8 pages)

### 7.1 Summary of Contributions
- Reproducible SLAM-in-the-loop Nav2 benchmark framework
- Evidence that navigation success decouples from fine-grained SLAM accuracy in bounded environments
- Public codebase for future researchers

### 7.2 Key Findings
- Both algorithms are navigation-capable in simple environments
- slam_toolbox offers superior accuracy at comparable compute cost
- Academic ATE benchmarks may over-emphasize precision beyond task-relevant thresholds

### 7.3 Future Work
- **7.3.1 Real hardware validation**
- **7.3.2 Degraded conditions** (noisy odometry, sensor dropout)
- **7.3.3 Challenging environments** (corridors, dynamic obstacles, symmetric spaces)
- **7.3.4 Parameter tuning study** (Optuna-based)
- **7.3.5 Alternative Nav2 controllers** (MPPI, Regulated Pure Pursuit)
- **7.3.6 3D LiDAR / visual SLAM extension**

---

## Appendices
- **A**: URDF robot model
- **B**: Complete parameter configurations
- **C**: Per-run raw results tables
- **D**: Setup instructions for reproduction
- **E**: Code repository structure

## References (~50-80 citations)

### Key Citations
1. Macenski & Jambrecic (2021) - slam_toolbox (JOSS)
2. Hess et al. (2016) - Google Cartographer real-time loop closure
3. Sturm et al. (2012) - TUM RGB-D SLAM benchmark (ATE/RPE methodology)
4. Zhang & Scaramuzza (IROS 2018) - Quantitative trajectory evaluation tutorial
5. Wisth et al. (2024) - SLAM Hive benchmarking suite
6. Umeyama (1991) - Least-squares trajectory alignment
7. Macenski et al. (2020) - Marathon 2: Nav2 paper

---

## Page Budget (~80-113 pages)

| Chapter | Pages |
|---|---|
| 1. Introduction | 8-10 |
| 2. Background & Related Work | 15-20 |
| 3. Methodology | 10-15 |
| 4. Implementation | 8-10 |
| 5. Results | 15-20 |
| 6. Discussion | 10-15 |
| 7. Conclusion | 5-8 |
| Appendices | 10-15 |
| **Total** | **80-113** |

---

## Current Experimental Status (as of 2026-04-05)

**Completed experiments** (6 runs, 1 per config):

| Trajectory | Algorithm | Success | ATE RMSE | RPE RMSE | Time |
|---|---|---|---|---|---|
| goals_01 (square, 4pts, ~12m) | slam_toolbox | 4/4 | 0.86 cm | 0.61 cm | 53s |
| goals_01 (square, 4pts, ~12m) | cartographer | 4/4 | 10.12 cm | 1.17 cm | 71s |
| goals_02 (explore, 7pts, ~15m) | slam_toolbox | 7/7 | 1.82 cm | 0.78 cm | 120s |
| goals_02 (explore, 7pts, ~15m) | cartographer | 7/7 | 12.31 cm | 1.49 cm | 132s |
| goals_03 (perimeter, 11pts, ~35m) | slam_toolbox | 11/11 | 3.57 cm | 0.89 cm | 285s |
| goals_03 (perimeter, 11pts, ~35m) | cartographer | 11/11 | 16.72 cm | 2.02 cm | 290s |

**Outstanding work before writing**:
- Run multiple repetitions (3-5 per config) for statistical validity
- Generate comparison plots from aggregated data
- Optional: noise condition, challenging environment, resource profiling
