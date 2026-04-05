#!/bin/bash
# Quick batch status checker
echo '=== Batch Progress ==='
python3 -c "
import json, os
path = os.path.expanduser('~/thesis/ros2_ws/results/batch_progress.json')
if os.path.exists(path):
    with open(path) as f: d = json.load(f)
    print(f'Completed: {len(d)}/18')
    for r in d:
        ate = r.get('metrics', {}).get('ate_rmse_m')
        ate_str = f'{ate*100:.2f}cm' if ate else 'N/A'
        succ = 'OK  ' if r['success'] else 'FAIL'
        print(f\"  {succ} {r['algorithm']:<14} {r['goals_file']:<22} run{r['run_num']}  ATE={ate_str}\")
else:
    print('No progress file yet')
"
echo ''
echo '=== Current Running Processes ==='
pgrep -af python3 | grep -E 'run_batch|run_nav' | awk '{print , 1, 2, 3}' 2>/dev/null
echo ''
echo '=== Gazebo State ==='
pgrep -af 'gz sim|async_slam|cartographer_node' | awk '{print , }' 2>/dev/null | head -3
