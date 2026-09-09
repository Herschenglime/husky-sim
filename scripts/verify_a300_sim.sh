#!/usr/bin/env bash
# Full verification of A300 observer Clearpath simulation.
# Tests spawn, topics, TF, teleop cmd_vel, and all six stock worlds.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_a300_sim.bash"

WS=/home/robopi/ros2_ws
LOG=/tmp/a300_sim_verify.log
NS=a300_00000
PASS=0
FAIL=0
WORLDS=(construction office orchard pipeline solar_farm warehouse)

log() { echo "[verify] $*"; }
pass() { PASS=$((PASS + 1)); log "PASS: $*"; }
fail() { FAIL=$((FAIL + 1)); log "FAIL: $*"; }

stop_sim() {
  a300_sim_stop
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-60}"
  local elapsed=0
  while [ "$elapsed" -lt "$timeout" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  return 1
}

wait_for_spawn() {
  local timeout="${1:-120}"
  local elapsed=0
  while [ "$elapsed" -lt "$timeout" ]; do
    if ros2 topic list 2>/dev/null | grep -q "/${NS}/robot_description"; then
      return 0
    fi
    sleep 3
    elapsed=$((elapsed + 3))
  done
  return 1
}

test_teleop_cmd_vel() {
  log "Publishing test TwistStamped to /${NS}/cmd_vel"
  timeout 8 ros2 topic pub --once "/${NS}/cmd_vel" geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    >/dev/null 2>&1 && pass "cmd_vel publish accepted" || fail "cmd_vel publish failed"
}

test_tf() {
  if timeout 10 ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/${NS}/tf 2>&1 | grep -q "Translation"; then
    pass "TF odom -> base_link (via /${NS}/tf)"
  else
    fail "TF odom -> base_link (via /${NS}/tf)"
  fi
}

test_world() {
  local world="$1"
  log "=== Testing world: ${world} ==="
  stop_sim
  : >"$LOG"

  ros2 launch husky_bamboo_sim a300_observer_sim.launch.py \
    world:="${world}" gui:=false keyboard_teleop:=false debug_monitor:=true \
    2>&1 | tee -a "$LOG" &
  LAUNCH_PID=$!

  if wait_for_spawn 150; then
    pass "${world}: robot spawned"
  else
    fail "${world}: robot spawn timeout"
    kill "$LAUNCH_PID" 2>/dev/null || true
    stop_sim
    return
  fi

  for topic in "/clock" "/${NS}/robot_description" "/${NS}/cmd_vel" "/${NS}/platform/odom"; do
    if wait_for_topic "$topic" 30; then
      pass "${world}: topic ${topic}"
    else
      fail "${world}: missing topic ${topic}"
    fi
  done

  if grep -q "sim_debug_monitor" "$LOG" 2>/dev/null; then
    pass "${world}: debug monitor started"
  else
    fail "${world}: debug monitor not seen in log"
  fi

  if grep -qiE "error|exception|failed" "$LOG" | head -5 | grep -qv "0 error"; then
  :
  fi

  kill "$LAUNCH_PID" 2>/dev/null || true
  stop_sim
  sleep 2
}

log "Building husky_bamboo_sim..."
cd "$WS"
colcon build --symlink-install --packages-select husky_bamboo_sim
source install/setup.bash

log "=== Single-world deep test (warehouse, headless) ==="
stop_sim
: >"$LOG"
ros2 launch husky_bamboo_sim a300_observer_sim.launch.py \
  world:=warehouse gui:=false keyboard_teleop:=false debug_monitor:=true \
  2>&1 | tee "$LOG" &
LAUNCH_PID=$!

if wait_for_spawn 150; then pass "warehouse spawn"; else fail "warehouse spawn"; fi
wait_for_topic "/${NS}/cmd_vel" 30 && pass "cmd_vel topic" || fail "cmd_vel topic"
wait_for_topic "/${NS}/platform/odom" 30 && pass "odom topic" || fail "odom topic"
test_teleop_cmd_vel
test_tf

kill "$LAUNCH_PID" 2>/dev/null || true
stop_sim

log "=== World-switch smoke test (all 6 worlds, headless) ==="
for world in "${WORLDS[@]}"; do
  test_world "$world"
done

log "=== Summary: ${PASS} passed, ${FAIL} failed ==="
if [ "$FAIL" -gt 0 ]; then
  exit 1
fi
