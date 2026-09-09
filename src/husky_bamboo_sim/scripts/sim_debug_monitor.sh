#!/usr/bin/env bash
# Periodic health monitor for the A300 Clearpath Gazebo simulation.
set -uo pipefail

NS="${1:-a300_00000}"
WORLD="${2:-warehouse}"
INTERVAL="${3:-5}"

log() { echo "[sim_debug_monitor] $*"; }

while true; do
  results=()
  ok=0
  total=0

  check() {
    local name="$1"
    local pass="$2"
    total=$((total + 1))
    if [ "$pass" = "1" ]; then
      ok=$((ok + 1))
      results+=("  OK   ${name}")
    else
      results+=("  FAIL ${name}")
    fi
  }

  pgrep -f "gz sim" >/dev/null 2>&1 && c=1 || c=0; check gz_sim "$c"
  ros2 topic list 2>/dev/null | grep -qx /clock && c=1 || c=0; check clock "$c"
  # >1 clock publisher means stale bridges from an earlier run: ROS time jitters
  # backwards and RViz keeps resetting. Fix: scripts/a300_sim.sh (runs a300_sim_stop).
  clock_pubs=$(ros2 topic info /clock 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "${clock_pubs:-0}" = "1" ] && c=1 || c=0; check "single_clock_publisher (${clock_pubs:-0})" "$c"
  ros2 topic list 2>/dev/null | grep -qx "/${NS}/robot_description" && c=1 || c=0; check robot_description "$c"
  ros2 topic list 2>/dev/null | grep -qx "/${NS}/cmd_vel" && c=1 || c=0; check cmd_vel "$c"
  ros2 topic list 2>/dev/null | grep -qx "/${NS}/platform/odom" && c=1 || c=0; check platform_odom "$c"
  ros2 topic list 2>/dev/null | grep -qx "/${NS}/sensors/lidar2d_0/scan" && c=1 || c=0; check lidar2d "$c"
  ros2 topic list 2>/dev/null | grep -qx "/${NS}/sensors/imu_0/data" && c=1 || c=0; check imu "$c"

  if [ "$ok" -eq "$total" ]; then
    status="HEALTHY"
  elif [ "$ok" -ge $((total - 2)) ]; then
    status="DEGRADED"
  else
    status="UNHEALTHY"
  fi

  log "[${status}] A300 sim health (${ok}/${total}) world=${WORLD}"
  printf '%s\n' "${results[@]}"
  sleep "$INTERVAL"
done
