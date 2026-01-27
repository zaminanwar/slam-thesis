# Interface Contracts

> **WARNING**: These interfaces are LOCKED. Do not modify without explicit user approval
> and updating ALL dependent code. These are the authoritative definitions.

---

## TF Frames

### Frame Definitions

| Frame | Description |
|-------|-------------|
| `base_link` | Center of rover body, on ground plane, X forward, Y left, Z up |
| `laser_frame` | LiDAR sensor frame, rigidly attached to base_link |
| `odom` | Odometry origin, initialized at robot start position |
| `map` | SLAM global frame (owned by SLAM algorithm) |
| `map_gt` | Ground truth global frame (owned by gt_publisher) |

### TF Publishers

| Transform | Publisher | Type | Rate |
|-----------|-----------|------|------|
| `odom → base_link` | diff_drive plugin (Gazebo) | Dynamic | 50 Hz |
| `base_link → laser_frame` | robot_state_publisher | Static | Once |
| `map → odom` | SLAM algorithm | Dynamic | ~10 Hz |
| `map_gt → base_link` | gt_publisher | Dynamic | 50 Hz |

### TF Tree Structure

```
During Simulation:

    odom                    map_gt
      │                       │
      └── base_link ──────────┘  (same base_link, two parents)
            │
            └── laser_frame


During SLAM (adds map→odom):

    map                     map_gt
      │                       │
      └── odom                │
            │                 │
            └── base_link ────┘
                  │
                  └── laser_frame
```

---

## Topics

### Core Topics (Required)

| Topic | Message Type | Publisher | Rate | QoS |
|-------|--------------|-----------|------|-----|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo LiDAR plugin | 10 Hz | Sensor |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo diff_drive | 50 Hz | Default |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | trajectory_follower | 20 Hz | Default |
| `/gt_pose` | `nav_msgs/msg/Odometry` | gt_publisher | 50 Hz | Default |
| `/trajectory_done` | `std_msgs/msg/Bool` | trajectory_follower | Latched | Transient Local |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo | ~100 Hz | Best Effort |
| `/tf` | `tf2_msgs/msg/TFMessage` | Multiple | ~50 Hz | Default |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Latched | Transient Local |

### SLAM Topics (Algorithm-Specific)

| Topic | Message Type | Publisher | Notes |
|-------|--------------|-----------|-------|
| `/map` | `nav_msgs/msg/OccupancyGrid` | slam_toolbox / cartographer | Optional to record |
| `/slam_toolbox/graph_visualization` | `visualization_msgs/msg/MarkerArray` | slam_toolbox | Debug only |

---

## Message Field Conventions

### LaserScan (/scan)

```yaml
header:
  frame_id: "laser_frame"
angle_min: -3.14159      # -180 degrees
angle_max: 3.14159       # +180 degrees
angle_increment: 0.0175  # ~1 degree (360 samples)
range_min: 0.1           # 10cm minimum
range_max: 10.0          # 10m maximum
ranges: [...]            # 360 float values
```

### Odometry (/odom, /gt_pose)

```yaml
header:
  frame_id: "odom"       # or "map_gt" for /gt_pose
child_frame_id: "base_link"
pose:
  pose:
    position: {x, y, z}  # meters
    orientation: {x, y, z, w}  # quaternion
  covariance: [...]      # 36 floats (6x6)
twist:
  twist:
    linear: {x, y, z}    # m/s
    angular: {x, y, z}   # rad/s
  covariance: [...]
```

### Twist (/cmd_vel)

```yaml
linear:
  x: 0.5    # Forward velocity (m/s), positive = forward
  y: 0.0    # Lateral (always 0 for differential drive)
  z: 0.0    # Vertical (always 0)
angular:
  x: 0.0    # Roll rate (always 0)
  y: 0.0    # Pitch rate (always 0)
  z: 0.3    # Yaw rate (rad/s), positive = counter-clockwise
```

---

## File Formats

### Trajectory CSV (Input)

Location: `~/thesis/trajectories/*.csv`

```csv
x,y,yaw,speed
0.0,0.0,0.0,0.5
5.0,0.0,0.0,0.5
5.0,5.0,1.5708,0.5
0.0,5.0,3.1416,0.5
0.0,0.0,-1.5708,0.5
```

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| x | float | meters | World X coordinate |
| y | float | meters | World Y coordinate |
| yaw | float | radians | Heading (-π to π) |
| speed | float | m/s | Desired forward speed |

### TUM Trajectory (Output)

Location: `~/thesis/ros2_ws/results/<dataset>/<algorithm>/*.tum`

```
# timestamp tx ty tz qx qy qz qw
1706123456.789000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
1706123456.839000 0.025000 0.001000 0.000000 0.000000 0.000000 0.002000 0.999998
```

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| timestamp | float | seconds | Unix timestamp (from ROS time) |
| tx | float | meters | Position X |
| ty | float | meters | Position Y |
| tz | float | meters | Position Z (≈0 for 2D) |
| qx | float | - | Quaternion X |
| qy | float | - | Quaternion Y |
| qz | float | - | Quaternion Z |
| qw | float | - | Quaternion W |

### metrics.json (Output)

Location: `~/thesis/ros2_ws/results/<dataset>/<algorithm>/metrics.json`

```json
{
  "success": true,
  "dataset": "traj_01_easy__baseline",
  "algorithm": "slam_toolbox",
  "timestamp": "2026-01-24T15:30:00",
  "ate": {
    "rmse": 0.0523,
    "mean": 0.0412,
    "median": 0.0389,
    "std": 0.0321,
    "min": 0.0021,
    "max": 0.1234
  },
  "rpe": {
    "rmse": 0.0234,
    "mean": 0.0198,
    "median": 0.0187,
    "std": 0.0123,
    "min": 0.0005,
    "max": 0.0567
  },
  "trajectory_length_m": 45.67,
  "duration_s": 91.34,
  "num_poses": 1827,
  "error": null
}
```

On failure:
```json
{
  "success": false,
  "dataset": "traj_01_easy__baseline",
  "algorithm": "slam_toolbox",
  "timestamp": "2026-01-24T15:30:00",
  "ate": null,
  "rpe": null,
  "error": "TF lookup failed: map->base_link not available"
}
```

### summary.csv (Aggregated Output)

Location: `~/thesis/ros2_ws/results/summary.csv`

```csv
dataset,trajectory,condition,algorithm,ate_rmse,ate_mean,rpe_rmse,rpe_mean,success,error
traj_01_easy__baseline,traj_01_easy,baseline,slam_toolbox,0.0523,0.0412,0.0234,0.0198,true,
traj_01_easy__baseline,traj_01_easy,baseline,cartographer,0.0487,0.0398,0.0201,0.0178,true,
traj_02_loop__odom_degraded,traj_02_loop,odom_degraded,slam_toolbox,0.1234,0.0987,0.0456,0.0389,true,
traj_02_loop__odom_degraded,traj_02_loop,odom_degraded,cartographer,,,,,false,SLAM diverged
```

---

## Directory Structure

### Bags

```
~/thesis/ros2_ws/bags/
└── <trajectory>__<condition>/
    ├── metadata.yaml
    └── <bagname>_0.db3
```

Example: `~/thesis/ros2_ws/bags/traj_01_easy__baseline/`

### Results

```
~/thesis/ros2_ws/results/
├── <trajectory>__<condition>/
│   ├── slam_toolbox/
│   │   ├── gt.tum
│   │   ├── est.tum
│   │   ├── ate_plot.png
│   │   ├── rpe_plot.png
│   │   └── metrics.json
│   └── cartographer/
│       ├── gt.tum
│       ├── est.tum
│       ├── ate_plot.png
│       ├── rpe_plot.png
│       └── metrics.json
└── summary.csv
```

---

## Parameters (Defaults)

### rover_control

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `trajectory_file` | (required) | string | Path to CSV file |
| `lookahead_distance` | 0.5 | float | Pure pursuit lookahead (m) |
| `goal_tolerance` | 0.2 | float | Waypoint reached threshold (m) |
| `speed_scale` | 1.0 | float | Multiplier on CSV speeds |
| `use_sim_time` | true | bool | Use /clock for time |

### gt_publisher

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `model_name` | "rover" | string | Gazebo model name |
| `publish_rate` | 50.0 | float | Hz |
| `use_sim_time` | true | bool | Use /clock for time |

### traj_exporter

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `gt_parent_frame` | "map_gt" | string | GT TF parent |
| `gt_child_frame` | "base_link" | string | GT TF child |
| `est_parent_frame` | "map" | string | Estimate TF parent |
| `est_child_frame` | "base_link" | string | Estimate TF child |
| `output_dir` | (required) | string | Where to write .tum files |
| `sample_rate` | 20.0 | float | Hz |
| `use_sim_time` | true | bool | Use /clock for time |

### Experiment Conditions

| Condition | Parameter Overrides |
|-----------|---------------------|
| `baseline` | (all defaults) |
| `odom_degraded` | odom_noise_std=0.05, odom_drift_rate=0.01 |
| `high_speed` | speed_scale=1.5 |

---

## SLAM Configuration Frames

### slam_toolbox

```yaml
# slam_toolbox.yaml
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
use_sim_time: true
```

### Cartographer

```lua
-- cartographer_2d.lua
MAP_FRAME = "map"
TRACKING_FRAME = "base_link"
PUBLISHED_FRAME = "base_link"
ODOM_FRAME = "odom"
PROVIDE_ODOM_FRAME = false  -- Use existing odom
USE_ODOMETRY = true
```

---

## Nav2 Autonomous Navigation

### Nav2 Action Servers

| Action | Type | Purpose |
|--------|------|---------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Send robot to a goal pose |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Navigate through waypoint sequence |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Follow predefined waypoints |

### Nav2 Topics (Input)

| Topic | Message Type | Publisher | Purpose |
|-------|--------------|-----------|---------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo | LiDAR data for costmaps |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo | Robot odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Multiple | Transform tree |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM | Map for global planning |

### Nav2 Topics (Output)

| Topic | Message Type | Publisher | Purpose |
|-------|--------------|-----------|---------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 Controller | Velocity commands |
| `/local_plan` | `nav_msgs/msg/Path` | Local Planner | Local trajectory |
| `/global_plan` | `nav_msgs/msg/Path` | Global Planner | Global path |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Costmap2D | Local obstacle map |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Costmap2D | Global planning map |

### Nav2 TF Requirements

Nav2 requires these TF transforms to function:

```
map → odom        # Published by SLAM (not AMCL - see AD-011)
odom → base_link  # Published by Gazebo diff_drive
base_link → laser_frame  # Published by robot_state_publisher (static)
```

**Important**: No AMCL is used. SLAM provides the `map → odom` transform directly.

### Navigation Goals Format (YAML)

Location: `~/thesis/ros2_ws/src/slam_thesis/experiment_runner/config/nav_goals_*.yaml`

```yaml
# Navigation goal waypoints
goals:
  - x: 3.0
    y: 0.0
    yaw: 0.0
    name: "point_1"
  - x: 3.0
    y: 3.0
    yaw: 1.5708
    name: "point_2"
  - x: 0.0
    y: 3.0
    yaw: 3.1416
    name: "point_3"
  - x: 0.0
    y: 0.0
    yaw: -1.5708
    name: "point_4"
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| x | float | meters | Goal X coordinate in map frame |
| y | float | meters | Goal Y coordinate in map frame |
| yaw | float | radians | Goal heading (-π to π) |
| name | string | - | Human-readable goal identifier |

### nav_results.json (Navigation Output)

Location: `~/thesis/ros2_ws/results/nav_<algorithm>_<goals>_<timestamp>/nav_results.json`

```json
{
  "algorithm": "slam_toolbox",
  "goals_file": "nav_goals_01.yaml",
  "world": "simple.sdf",
  "timestamp": "2026-01-26T20:30:00",
  "goals_succeeded": 4,
  "goals_attempted": 4,
  "success_rate": 1.0,
  "total_time_s": 120.5,
  "timeout_per_goal_s": 120.0,
  "per_goal_results": [
    {
      "name": "point_1",
      "x": 3.0,
      "y": 0.0,
      "yaw": 0.0,
      "success": true,
      "time_s": 25.3,
      "error": ""
    },
    {
      "name": "point_2",
      "x": 3.0,
      "y": 3.0,
      "yaw": 1.5708,
      "success": true,
      "time_s": 32.1,
      "error": ""
    }
  ]
}
```

On failure:
```json
{
  "algorithm": "cartographer",
  "goals_succeeded": 2,
  "goals_attempted": 4,
  "success_rate": 0.5,
  "per_goal_results": [
    {
      "name": "point_3",
      "success": false,
      "time_s": 120.0,
      "error": "Timeout"
    }
  ]
}
```

### Nav2 Parameter Configuration

Key parameters in `nav2_slam_params.yaml`:

| Component | Parameter | Value | Rationale |
|-----------|-----------|-------|-----------|
| Local Planner | controller_plugin | dwb_core::DWBLocalPlanner | Best for differential drive |
| Global Planner | planner_plugin | nav2_navfn_planner/NavfnPlanner | Simple, reliable for 2D |
| Robot Footprint | robot_radius | 0.25m | Based on 0.4x0.3m chassis + safety |
| Costmap | resolution | 0.05m | Matches SLAM grid resolution |
| Controller | controller_frequency | 20.0 Hz | Matches /cmd_vel rate |
| Costmap | inflation_radius | 0.55m | Robot radius + safety margin |
| Transform | transform_tolerance | 1.0s | WSL2 timing tolerance |
