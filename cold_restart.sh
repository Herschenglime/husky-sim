#!/usr/bin/env bash
# ==============================================================================
# cold_restart.sh - ROS 2 & Gazebo Sim Teardown to Ground Zero
# ==============================================================================
# Deterministically shuts down all ROS 2 nodes, Gazebo instances, bridges,
# static transform publishers, and middleware discovery caches.
#
# Standalone and project-independent: works in any workspace or ROS 2 environment.
#
# Usage:
#   ./cold_restart.sh
# ==============================================================================

set -eo pipefail

echo "============================================================"
echo "[GROUND ZERO] Terminating ROS 2 & Gazebo environment..."
echo "============================================================"

# Phase 1: Graceful termination attempt (SIGTERM)
echo "-> Phase 1: Sending SIGTERM to active launch and simulation processes..."
pkill -15 -f "ros2 launch" 2>/dev/null || true
pkill -15 -f "gz sim" 2>/dev/null || true
pkill -15 -f "ign gazebo" 2>/dev/null || true
sleep 1

# Phase 2: Forceful kill (SIGKILL) on simulation, bridge, and navigation processes
echo "-> Phase 2: Forcing SIGKILL on simulation, bridges, and infrastructure..."
TARGET_PATTERNS=(
  "gz sim"
  "gz-sim"
  "ign gazebo"
  "parameter_bridge"
  "image_bridge"
  "ros_gz_bridge"
  "static_transform_publisher"
  "robot_state_publisher"
  "controller_manager"
  "ros2_control_node"
  "spawner"
  "ros2 launch"
  "rviz2"
  "nav2_"
  "amcl"
  "slam_toolbox"
)

for pattern in "${TARGET_PATTERNS[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done
sleep 1

# Phase 3: Middleware cleanup (ROS 2 Daemon & Shared Memory)
echo "-> Phase 3: Flushing ROS 2 discovery daemon and DDS shared memory..."
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
  ros2 daemon stop >/dev/null 2>&1 || true
fi

# Clean stale POSIX shared memory segments from FastDDS & CycloneDDS
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cyclonedds_* 2>/dev/null || true

# Phase 4: Audit and verify process table
echo "-> Phase 4: Auditing process table..."
REMAINING=$(ps -u "$USER" -o pid,comm,args 2>/dev/null | grep -E "gz sim|parameter_bridge|image_bridge|static_transform_publisher|robot_state_publisher|nav2_|amcl|rviz2" | grep -v grep || true)

if [[ -n "$REMAINING" ]]; then
  echo "[WARNING] Found surviving processes after teardown:"
  echo "$REMAINING"
  echo "Terminating lingering PIDs..."
  echo "$REMAINING" | awk '{print $1}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
else
  echo "[OK] All target processes terminated."
fi

# Reinitialize ROS 2 daemon cleanly
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon start >/dev/null 2>&1 || true
fi

echo "============================================================"
echo "[GROUND ZERO] Reset complete. System is ready for clean runs."
echo "============================================================"
