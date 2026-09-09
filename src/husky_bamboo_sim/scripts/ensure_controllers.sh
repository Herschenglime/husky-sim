#!/usr/bin/env bash
# Recovery step: (re)spawn ros2_control controllers only if they are not active.
# Usage: ensure_controllers.sh <namespace> <controller>...
set -uo pipefail

NS="${1:?namespace}"
shift
CONTROLLERS=("$@")

log() { echo "[ensure_controllers] $*"; }

states="$(timeout 20 ros2 service call "/${NS}/controller_manager/list_controllers" \
  controller_manager_msgs/srv/ListControllers 2>/dev/null)"
if [[ -z "${states}" ]]; then
  log "controller_manager not reachable; nothing to do"
  exit 0
fi

missing=()
for ctrl in "${CONTROLLERS[@]}"; do
  if grep -qE "name='${ctrl}', state='active'" <<< "${states}"; then
    continue
  fi
  missing+=("${ctrl}")
  if grep -qE "name='${ctrl}'" <<< "${states}"; then
    # Loaded but stuck unconfigured/inactive (earlier spawner timed out mid-switch):
    # unload so the spawner can run the full load -> configure -> activate again.
    log "unloading stale controller ${ctrl}"
    timeout 20 ros2 service call "/${NS}/controller_manager/unload_controller" \
      controller_manager_msgs/srv/UnloadController "{name: '${ctrl}'}" >/dev/null 2>&1 || true
  fi
done

if [[ ${#missing[@]} -eq 0 ]]; then
  log "all controllers active: ${CONTROLLERS[*]}"
  exit 0
fi

log "controllers not active, spawning: ${missing[*]}"
ROS_SUPER_CLIENT=True exec ros2 run controller_manager spawner \
  --controller-manager-timeout 120 "${missing[@]}" \
  --ros-args -r "__ns:=/${NS}"
