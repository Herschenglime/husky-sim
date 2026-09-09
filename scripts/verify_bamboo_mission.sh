#!/usr/bin/env bash
# Verify bamboo field mission (single sim entry point).
set -eo pipefail

WS=/home/robopi/ros2_ws
LOG=/tmp/launch_diag.txt

set +u
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install --packages-select husky_bamboo_sim husky_nav_bringup husky_exploration
source install/setup.bash

pkill -f bamboo_field_mission.launch.py 2>/dev/null || true
pkill -f simulation.launch.py 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 2

ros2 launch husky_nav_bringup bamboo_field_mission.launch.py 2>&1 | tee "$LOG" &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"
sleep 140

echo "=== SIM / SPAWN (expect simulation.launch, NOT husky_gz_spawn) ==="
grep -E "process started|simulation|generate_description|robot_state_publisher|create|lidar3d_0|odom_base_tf" "$LOG" | head -40 || true

echo "=== DUAL-NAMESPACE CHECK (should only see /husky/, not a300) ==="
ros2 topic list 2>&1 | grep -E "husky|a300" | head -30 || true

echo "=== TF ==="
timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -12 || true

kill "$LAUNCH_PID" 2>/dev/null || true
pkill -f bamboo_field_mission.launch.py 2>/dev/null || true
