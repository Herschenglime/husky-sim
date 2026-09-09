#!/usr/bin/env bash
# Headless Gazebo + RViz only. No map, AMCL, or Nav2 — ready to SLAM/map.
# Usage: a300_sim.sh [world] [extra launch args...]
# Worlds: construction, office, orchard, pipeline, solar_farm, warehouse
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_a300_sim.bash"

_cleanup_stale_a300_sim() {
  if [[ "${A300_SIM_CLEAN:-1}" == "0" ]]; then
    return 0
  fi
  a300_sim_stop
}

WORLD="${1:-warehouse}"
shift || true

VALID_WORLDS="construction office orchard pipeline solar_farm warehouse"
if ! echo "$VALID_WORLDS" | grep -qw "$WORLD"; then
  echo "Invalid world: $WORLD" >&2
  echo "Valid worlds: $VALID_WORLDS" >&2
  exit 1
fi

_cleanup_stale_a300_sim

exec ros2 launch husky_bamboo_sim a300_observer_sim.launch.py \
  world:="${WORLD}" \
  gui:=false \
  rviz:=true \
  rviz_map:=false \
  "$@"
