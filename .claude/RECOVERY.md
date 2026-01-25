# Recovery Guide

This document helps diagnose and fix common issues.
Read this when something isn't working.

---

## Quick Diagnostics

### Check Everything Script

```bash
#!/bin/bash
# Run this to check system health
echo "=== ROS2 ===" && source /opt/ros/humble/setup.bash && echo "OK: ROS2 sourced"
echo "=== Workspace ===" && source ~/thesis/ros2_ws/install/setup.bash 2>/dev/null && echo "OK: Workspace sourced" || echo "FAIL: Workspace not built"
echo "=== Gazebo ===" && which gzserver && echo "OK: Gazebo found" || echo "FAIL: Gazebo not installed"
echo "=== Topics ===" && ros2 topic list 2>/dev/null | head -5 || echo "No ROS2 daemon running"
```

---

## Common Issues

### 1. "colcon build fails"

**Symptoms**: Build errors, package not found, CMake errors

**Diagnosis**:
```bash
# Check ROS2 is sourced
echo $ROS_DISTRO  # Should print "humble"

# Check package.xml validity
cd ~/thesis/ros2_ws/src/slam_thesis/<package>
cat package.xml | head -20
```

**Fixes**:
1. Source ROS2 first: `source /opt/ros/humble/setup.bash`
2. Check package.xml has valid XML (no syntax errors)
3. For Python packages, ensure setup.py AND setup.cfg exist
4. Build single package to isolate: `colcon build --packages-select <package>`
5. Clean and rebuild: `rm -rf build install log && colcon build`

---

### 2. "TF lookup fails" / "Could not transform"

**Symptoms**: Errors like `Could not transform base_link to map`, transform timeout

**Diagnosis**:
```bash
# View current TF tree
ros2 run tf2_tools view_frames
evince frames.pdf  # or open in file browser

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link

# List all frames
ros2 run tf2_ros tf2_monitor
```

**Common Causes**:
| Cause | Fix |
|-------|-----|
| Node not using sim_time | Add `use_sim_time:=true` to launch |
| /clock not publishing | Ensure Gazebo is running |
| Frame name mismatch | Check INTERFACES.md for correct names |
| Publisher not running | Check node is alive with `ros2 node list` |

**Fixes**:
1. Ensure ALL nodes have `use_sim_time:=true`
2. Check /clock is publishing: `ros2 topic hz /clock`
3. Verify frame names match INTERFACES.md exactly
4. Wait for TF buffer to fill (add 1-2 sec sleep after launch)

---

### 3. "Gazebo crashes on startup"

**Symptoms**: Segfault, black window, immediate exit

**Diagnosis**:
```bash
# Check for zombie processes
ps aux | grep gz
killall -9 gzserver gzclient

# Check display
echo $DISPLAY  # Should be :0 or similar

# Run with verbose output
gzserver --verbose
```

**Fixes**:
1. Kill zombie processes: `killall -9 gzserver gzclient`
2. Check WSL2 display: `export DISPLAY=:0`
3. Check GPU: Try `export LIBGL_ALWAYS_SOFTWARE=1` if GPU issues
4. Verify world file exists and is valid XML
5. Check GAZEBO_MODEL_PATH includes model directories

---

### 4. "/scan not publishing"

**Symptoms**: `ros2 topic echo /scan` shows nothing

**Diagnosis**:
```bash
# Check topic exists
ros2 topic list | grep scan

# Check rate
ros2 topic hz /scan

# Check Gazebo terminal for plugin errors
```

**Fixes**:
1. Verify LiDAR plugin in URDF has `<ros>` block with correct topic
2. Check Gazebo loaded plugin (look for "Loading plugin" in terminal)
3. Verify `frame_id` in plugin matches `laser_frame`
4. Check robot actually spawned (visible in Gazebo)

---

### 5. "Bag replay doesn't work"

**Symptoms**: Topics don't appear, time doesn't advance, nodes don't receive data

**Diagnosis**:
```bash
# Check bag contents
ros2 bag info ~/thesis/ros2_ws/bags/<bagname>

# Check topics in bag
ros2 bag info ~/thesis/ros2_ws/bags/<bagname> | grep -A 100 "Topic information"
```

**Fixes**:
1. Use `--clock` flag: `ros2 bag play <bag> --clock`
2. Ensure nodes use `use_sim_time:=true`
3. Check bag has /clock topic recorded
4. Check bag path is correct (it's a directory, not a file)
5. Wait for bag to start before checking topics

---

### 6. "SLAM doesn't produce map->odom transform"

**Symptoms**: TF tree missing map frame, SLAM appears stuck

**Diagnosis**:
```bash
# Check SLAM node is running
ros2 node list | grep -E "slam|cartographer"

# Check SLAM is receiving scans
ros2 topic echo /scan --once

# Check TF tree
ros2 run tf2_tools view_frames
```

**Fixes**:
1. Verify /scan is publishing during bag replay
2. Check SLAM config frame names match INTERFACES.md
3. For Cartographer: Check .lua config syntax
4. Ensure use_sim_time:=true in SLAM launch
5. Some SLAM algorithms need motion before publishing (move the robot)

---

### 7. "evo fails"

**Symptoms**: Alignment error, file format error, empty plots

**Diagnosis**:
```bash
# Check file format
head -5 gt.tum
head -5 est.tum

# Check timestamps overlap
evo_traj tum gt.tum est.tum -p
```

**Fixes**:
1. Verify 8 columns per line (timestamp tx ty tz qx qy qz qw)
2. Check files aren't empty
3. Ensure timestamps overlap between gt.tum and est.tum
4. Check for NaN or inf values in files
5. Try without alignment first: remove `--align` flag

---

### 8. "WSL2 GUI doesn't work"

**Symptoms**: Cannot open Gazebo/RViz, display errors

**Diagnosis**:
```bash
echo $DISPLAY
echo $WAYLAND_DISPLAY

# Test basic GUI
sudo apt install x11-apps
xeyes
```

**Fixes**:
1. Windows 11 with WSLg should work automatically
2. Set `export DISPLAY=:0` in ~/.bashrc
3. For older Windows: Install VcXsrv or similar X server
4. Try: `export LIBGL_ALWAYS_SOFTWARE=1` for software rendering

---

## Verification Commands

### Verify M1 (Simulation)
```bash
source ~/thesis/ros2_ws/install/setup.bash
ros2 launch rover_sim sim.launch.py &
sleep 10
ros2 topic list | grep -E "/scan|/clock|/odom"
ros2 topic hz /scan  # Should show ~10 Hz
ros2 run tf2_ros tf2_echo odom base_link  # Should show transform
```

### Verify M2 (Trajectory)
```bash
ros2 launch rover_sim sim.launch.py &
sleep 5
ros2 launch rover_control follow_trajectory.launch.py trajectory:=traj_01_easy.csv &
# Watch Gazebo - rover should move
ros2 topic echo /trajectory_done  # Should become True when done
```

### Verify M3 (Ground Truth)
```bash
ros2 launch rover_sim sim.launch.py &
ros2 launch gt_publisher gt_publisher.launch.py &
sleep 5
ros2 topic echo /gt_pose --once  # Should show pose
ros2 run tf2_ros tf2_echo map_gt base_link  # Should show transform
```

### Verify M4 (Bag Recording)
```bash
# Record
ros2 launch experiment_runner record_dataset.launch.py trajectory:=traj_01_easy condition:=baseline
# Wait for completion...

# Verify bag
ros2 bag info ~/thesis/ros2_ws/bags/traj_01_easy__baseline
# Should list all required topics

# Test replay
ros2 bag play ~/thesis/ros2_ws/bags/traj_01_easy__baseline --clock &
ros2 topic echo /scan --once  # Should show data
```

---

## Starting Fresh

If the project is in an unrecoverable state:

```bash
# 1. Kill all ROS/Gazebo processes
killall -9 gzserver gzclient
pkill -f ros2

# 2. Clean build artifacts
cd ~/thesis/ros2_ws
rm -rf build install log

# 3. Rebuild
source /opt/ros/humble/setup.bash
colcon build

# 4. Source and test
source install/setup.bash
ros2 pkg list | grep rover  # Should show our packages

# 5. Check STATE.md for where to resume
cat ~/thesis/.claude/STATE.md | grep -A 5 "Current Task"
```

---

## Getting Help

1. Check this file first
2. Read relevant section in INTERFACES.md
3. Check ROS2 Humble documentation: https://docs.ros.org/en/humble/
4. Check Gazebo Classic wiki: http://gazebosim.org/tutorials
5. For evo issues: https://github.com/MichaelGrupp/evo

## Reporting Issues to Claude

When asking Claude for help, provide:
1. The exact error message
2. Output of `ros2 topic list`
3. Output of `ros2 node list`
4. Contents of the launch file if relevant
5. Which milestone/task you're on (from STATE.md)
