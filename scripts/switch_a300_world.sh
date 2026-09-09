#!/usr/bin/env bash
# Switch A300 sim to a different stock Clearpath world (restarts simulation).
# Usage: switch_a300_world.sh <world>
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-}"

VALID_WORLDS="construction office orchard pipeline solar_farm warehouse"
if ! echo "$VALID_WORLDS" | grep -qw "$WORLD"; then
  echo "Usage: $0 <world>" >&2
  echo "Valid worlds: $VALID_WORLDS" >&2
  exit 1
fi

echo "Stopping existing A300 simulation..."
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_a300_sim.bash"
a300_sim_stop

echo "Starting world: ${WORLD}"
exec "${SCRIPT_DIR}/a300_sim.sh" "${WORLD}"
