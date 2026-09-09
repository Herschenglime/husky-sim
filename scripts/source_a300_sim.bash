#!/usr/bin/env bash
# Source ROS + both workspaces for A300 Clearpath simulation.
set -eo pipefail

# ROS/Clearpath generators require system Python (Conda's python3 lacks python3-apt).
export PATH="/usr/bin:${PATH}"

source /opt/ros/jazzy/setup.bash
source /home/robopi/clearpath_ws/install/setup.bash
source /home/robopi/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Avoid stale Fast-DDS shared-memory port locks after crashed sim runs.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

# NVIDIA GPU rendering for Gazebo Harmonic (sensors + GUI).
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export LIBGL_ALWAYS_SOFTWARE=0

export A300_NAMESPACE="${A300_NAMESPACE:-a300_00000}"

# Stop every process left over from earlier A300 sim runs.
#
# Killing `ros2 launch` mid-shutdown orphans its children (clock bridge, EKF,
# static TF publishers, RViz, gz bridges). Stale clock bridges then republish
# /clock with jitter, ROS time appears to jump backwards, and RViz resets
# (map vanishes, robot model drops out) every few seconds.
_a300_kill_matching() {
  # SIGKILL every process whose command line matches, sparing this shell and its parent.
  local pid
  for pid in $(pgrep -f "$1" || true); do
    [[ "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
    kill -9 "${pid}" 2>/dev/null || true
  done
}

a300_sim_stop() {
  local pids
  pids="$(pgrep -f 'a300_observer_sim.launch.py' || true)"
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill -INT ${pids} 2>/dev/null || true
    for _ in $(seq 1 10); do
      pgrep -f 'a300_observer_sim.launch.py' >/dev/null || break
      sleep 1
    done
  fi
  # Patterns are anchored on install paths / ROS remap args so a shell that merely
  # mentions these names (e.g. a grep) is not caught.
  _a300_kill_matching 'ros2 launch .*a300_observer_sim.launch.py'
  _a300_kill_matching "__ns:=/${A300_NAMESPACE}"
  _a300_kill_matching "/model/${A300_NAMESPACE}/robot"
  _a300_kill_matching "child-frame-id ${A300_NAMESPACE}/robot"
  _a300_kill_matching 'parameter_bridge /clock@'
  _a300_kill_matching 'rviz2 -d .*(clearpath_viz|a300_nav2)'
  _a300_kill_matching 'lib/husky_bamboo_sim/(sim_keyboard_teleop|sim_debug_monitor|ensure_controllers)'
  _a300_kill_matching 'lib/ros_gz_sim/create'
  # Both the `sh -c ruby /usr/bin/gz sim ...` wrapper and the re-exec'd `gz sim <world>.sdf` server.
  _a300_kill_matching 'gz sim [a-z_]+\.sdf'
  sleep 1
}
