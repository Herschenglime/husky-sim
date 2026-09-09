#!/usr/bin/env bash
# Bring up simulation + SLAM + Nav2 for the a200, waiting for each stage to be
# genuinely ready before starting the next.
#
#   bringup.sh [world] [--localize]
#
# Staged deliberately rather than with fixed delays in a launch file. Starting
# Nav2 while the world is still loading starves the lifecycle handshake -
# "failed to send response to change_state (timeout)" - and the servers strand
# in unconfigured/inactive with no action server ever appearing. How long a
# world takes to load varies by an order of magnitude between warehouse and
# office, so any fixed delay is either too short somewhere or wasteful
# everywhere. These waits are on observable state instead.
set -o pipefail

WORLD=""
MODE=""
HEADLESS="${HEADLESS:-true}"
DETACH=false

for arg in "$@"; do
  case "$arg" in
    --gui) HEADLESS=false ;;
    --headless) HEADLESS=true ;;
    --localize) MODE="--localize" ;;
    --detach|-d) DETACH=true ;;
    --stop|stop|--kill)
      echo "Stopping running simulation processes..."
      pkill -9 -f "gz sim|ruby.*gz|ros_gz_bridge|parameter_bridge|scan_self_filter|slam_toolbox|nav2_|clearpath_nav2_demos|sim.launch.py" 2>/dev/null || true
      echo "Stopped."
      exit 0
      ;;
    -*)
      echo "Unknown flag: $arg"
      exit 1
      ;;
    *)
      [ -z "$WORLD" ] && WORLD="$arg"
      ;;
  esac
done
WORLD="${WORLD:-warehouse}"
: "${ROS_DISTRO:=jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"

PIDS=()
PGIDS=()

launch_bg() {
  local log_file="$1"; shift
  setsid "$@" > "${log_file}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  local pgid
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
  [ -n "${pgid}" ] && PGIDS+=("${pgid}")
  return 0
}

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "Shutting down simulation and cleaning up background processes..."
  for pgid in "${PGIDS[@]}"; do
    [ -n "${pgid}" ] && kill -TERM -"${pgid}" 2>/dev/null || true
  done

  local waited=0
  while [ "$waited" -lt 3 ]; do
    local any_alive=0
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        any_alive=1
        break
      fi
    done
    [ "$any_alive" -eq 0 ] && break
    sleep 1
    waited=$((waited + 1))
  done

  for pgid in "${PGIDS[@]}"; do
    [ -n "${pgid}" ] && kill -9 -"${pgid}" 2>/dev/null || true
  done

  pkill -9 -f "gz sim|ruby.*gz|ros_gz_bridge|parameter_bridge|scan_self_filter\.py|slam_toolbox|nav2_" 2>/dev/null || true
  echo "Cleanup complete."
}
trap cleanup INT TERM EXIT

NS="$(python3 -c "
import yaml;print(yaml.safe_load(open('$HOME/clearpath/robot.yaml'))['system']['ros2']['namespace'])" 2>/dev/null || echo a200_0000)"
LOG="$(mktemp -d -t nav_worlds.XXXXXX)"
echo "world=${WORLD} namespace=${NS} headless=${HEADLESS} logs=${LOG}"

# The name inside the SDF is not always the file name: clearpath's
# construction.sdf declares <world name="office_construction">. Gazebo's topics
# use the internal name, so every /world/<name>/... check has to use it too -
# watching /world/construction/... simply never resolves, and the bringup sits
# at its first gate while the simulation is in fact running perfectly.
WORLD_SDF="${WORLD_FILE:-/opt/ros/jazzy/share/clearpath_gz/worlds/${WORLD}.sdf}"
GZ_WORLD="$(grep -oE "<world name=['\"][^'\"]+" "${WORLD_SDF}" 2>/dev/null | head -1 | sed "s/.*=.//")"
GZ_WORLD="${GZ_WORLD:-${WORLD}}"
[ "${GZ_WORLD}" != "${WORLD}" ] && echo "  (world file ${WORLD} declares itself '${GZ_WORLD}')"

wait_for () {  # wait_for "description" seconds command...
  local what="$1" limit="$2"; shift 2
  local i=0
  until "$@" >/dev/null 2>&1; do
    i=$((i+1))
    if [ "$i" -gt "$limit" ]; then echo "FAIL: ${what} never became ready"; return 1; fi
    sleep 3
  done
  echo "  ${what} ready (${i} x3 s)"
}

# depot is not one of clearpath_gz's worlds, so hand the launch an absolute
# path instead of a bare name. clearpath_gz builds gz_args as "<world>.sdf" and
# overwrites GZ_SIM_RESOURCE_PATH with its own directories, so an exported path
# would be discarded - but an absolute path resolves regardless.
# Worlds carried in this package (depot) are passed by file; clearpath's own
# are passed by name. nav_worlds/sim.launch.py handles both - clearpath_gz's
# simulation.launch.py cannot, because its `world` argument has a choices list.
LOCAL_WORLD="$(ros2 pkg prefix nav_worlds)/share/nav_worlds/worlds/${WORLD}.sdf"
WORLD_FILE=""
if [ -f "${LOCAL_WORLD}" ]; then
  WORLD_FILE="${LOCAL_WORLD}"
  echo "using world file ${WORLD_FILE}"
fi

# Only pass world_file when there is one: ros2 launch rejects an empty value
# outright ("malformed launch argument 'world_file:='"), which took out every
# clearpath world - they resolve by name and have no local file.
# Headless by default: the GUI costs a large slice of the CPU that the
# physics and sensor rendering need, and nothing here looks at it.
# HEADLESS=false gives the GUI back for watching a run.
SIM_ARGS=(world:="${WORLD}" setup_path:="$HOME/clearpath/" headless:="${HEADLESS:-true}")
[ -n "${WORLD_FILE}" ] && SIM_ARGS+=(world_file:="${WORLD_FILE}")

launch_bg "${LOG}/sim.log" ros2 launch nav_worlds sim.launch.py "${SIM_ARGS[@]}"
SIM_PID="${PIDS[-1]}"
echo "launched simulation (PID ${SIM_PID})"
wait_for "world + robot" 60 bash -c \
  "gz topic -e -t /world/${GZ_WORLD}/dynamic_pose/info -n 1 2>/dev/null | grep -q ${NS}/robot" || exit 1
# Wait for the simulation to actually reach real-time speed before starting
# anything that depends on it.
#
# A heavy world sits near RTF 0.0005 while Gazebo loads its meshes, and every
# consumer started during that window fails in its own way: the controller
# spawner's activation switch hits an internal 5 s timeout and dies, and
# slam_toolbox logs "Configuring" and never reaches Activating. Both look like
# unrelated bugs and are just starvation. The world settles to RTF 1.0 shortly
# afterwards, so the fix is simply not to be early.
settled=0
for i in $(seq 1 80); do
  rtf=$(timeout 5 gz topic -e -t "/world/${GZ_WORLD}/stats" -n 1 2>/dev/null \
        | grep -o 'real_time_factor: [0-9.]*' | awk '{print $2}')
  ok=$(awk -v r="${rtf:-0}" 'BEGIN{print (r>0.5)?1:0}')
  if [ "$ok" = "1" ]; then
    settled=$((settled+1))
    [ "$settled" -ge 3 ] && { echo "  simulation at speed (RTF ${rtf})"; break; }
  else
    settled=0
  fi
  sleep 3
done

wait_for "lidar" 40 bash -c \
  "timeout 5 ros2 topic echo /${NS}/sensors/lidar2d_0/scan --once >/dev/null 2>&1"
controllers_active () {
  ros2 service call "/${NS}/controller_manager/list_controllers" \
    controller_manager_msgs/srv/ListControllers 2>/dev/null \
    | grep -q "platform_velocity_controller', state='active'"
}

# Top up the controllers rather than just waiting for them.
#
# Clearpath's own spawner runs as soon as the robot entity exists, which in a
# heavy world is while Gazebo is still loading meshes - the depot world sits at
# an RTF of 0.0005 for a while. The controller_manager update loop is starved,
# the activation switch hits its internal 5 s timeout, and the spawner dies:
# "Switch controller timed out after 5 seconds", leaving joint_state_broadcaster
# inactive and platform_velocity_controller not loaded at all. The sim settles
# to RTF 1.0 a few seconds later, so simply asking again works.
if ! controllers_active; then
  echo "  controllers not up (heavy world startup) - spawning them again"
  for attempt in 1 2 3; do
    ros2 run controller_manager spawner \
      --controller-manager "/${NS}/controller_manager" --controller-manager-timeout 60 \
      joint_state_broadcaster platform_velocity_controller >/dev/null 2>&1 || true
    controllers_active && break
    sleep 5
  done
fi
wait_for "controllers" 40 controllers_active

# Strip the robot's own structure out of the scan before anything plans on it.
#
# With the sensor arch fitted, the arch legs sit ~0.40 m from the lidar, inside
# the 1.1 x 0.9 m footprint. 22 of 720 beams come back off the robot, and Nav2's
# collision_monitor (FootprintApproach, min_points 12) reads them as an imminent
# collision and scales every command to zero - the robot takes the goal, plans a
# path, and never moves, with nothing in any log to say why. SLAM wants the
# filtered scan too, or those returns get drawn into the map as a permanent
# obstacle blob that follows the robot around.
SCAN_FILTERED="/${NS}/sensors/lidar2d_0/scan_filtered"
# tf2_ros subscribes to an *absolute* /tf and /tf_static regardless of the
# node namespace, so the remaps are not optional: without them the filter
# never resolves base_link -> lidar2d_0_laser and silently passes nothing on.
launch_bg "${LOG}/scan_filter.log" ros2 run nav_worlds scan_self_filter.py --ros-args \
  -r __ns:="/${NS}" -p use_sim_time:=true \
  -r /tf:="/${NS}/tf" -r /tf_static:="/${NS}/tf_static"
echo "launched scan self-filter"
wait_for "filtered scan" 20 bash -c \
  "timeout 5 ros2 topic echo ${SCAN_FILTERED} --once >/dev/null 2>&1" || exit 1
grep -o "masked [0-9]* of [0-9]* beams" "${LOG}/scan_filter.log" | tail -1 | sed 's/^/  /'

if [ "${MODE}" = "--localize" ]; then
  MAP="$(ros2 pkg prefix nav_worlds)/share/nav_worlds/maps/${WORLD}.yaml"
  [ -f "${MAP}" ] || { echo "FAIL: no saved map at ${MAP}; run without --localize to SLAM"; exit 1; }
  launch_bg "${LOG}/loc.log" ros2 launch clearpath_nav2_demos localization.launch.py \
    use_sim_time:=true setup_path:="$HOME/clearpath/" map:="${MAP}" \
    scan_topic:="${SCAN_FILTERED}"
  echo "launched localization (${MAP})"
else
  launch_bg "${LOG}/slam.log" ros2 launch clearpath_nav2_demos slam.launch.py \
    use_sim_time:=true setup_path:="$HOME/clearpath/" scan_topic:="${SCAN_FILTERED}"
  echo "launched slam"
fi
map_ready () { timeout 5 ros2 topic echo "/${NS}/map" --once --qos-durability transient_local >/dev/null 2>&1; }

# Drive slam_toolbox's lifecycle by hand if its own transition was lost.
#
# slam_toolbox is a lifecycle node that self-transitions on startup. On a
# CPU-heavy world the change_state response times out - "failed to send response
# to /<ns>/slam_toolbox/change_state" - and it strands in unconfigured or
# inactive, logging "Configuring" and never "Activating", so no /map is ever
# published and Nav2 comes up with nothing to plan on. Waiting longer does not
# help; the transition has to be requested again. Note this outlasts the RTF
# gate: construction stalls here even at RTF 1.0.
if [ "${MODE}" != "--localize" ]; then
  for attempt in 1 2 3; do
    map_ready && break
    state="$(ros2 lifecycle get "/${NS}/slam_toolbox" 2>/dev/null | awk '{print $1}')"
    case "${state}" in
      unconfigured) echo "  slam_toolbox ${state} - configuring and activating"
                    ros2 lifecycle set "/${NS}/slam_toolbox" configure >/dev/null 2>&1
                    ros2 lifecycle set "/${NS}/slam_toolbox" activate  >/dev/null 2>&1 ;;
      inactive)     echo "  slam_toolbox ${state} - activating"
                    ros2 lifecycle set "/${NS}/slam_toolbox" activate  >/dev/null 2>&1 ;;
      *)            ;;
    esac
    sleep 8
  done
fi

wait_for "map" 60 map_ready || exit 1

launch_bg "${LOG}/nav2.log" ros2 launch clearpath_nav2_demos nav2.launch.py \
  use_sim_time:=true setup_path:="$HOME/clearpath/" scan_topic:="${SCAN_FILTERED}"
echo "launched nav2"
wait_for "navigate_to_pose action" 80 bash -c \
  "ros2 action list 2>/dev/null | grep -q /${NS}/navigate_to_pose" || exit 1

if [ "${MODE}" = "--localize" ]; then
  ros2 topic pub -t 3 --qos-reliability reliable "/${NS}/initialpose" \
    geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    >/dev/null 2>&1
  echo "  seeded initial pose at the spawn"
fi

echo
echo "READY. Send goals with:"
echo "  ros2 run nav_worlds send_goal.py X Y [YAW_DEG] --ns ${NS}"

if [ "${DETACH}" = "true" ]; then
  trap - INT TERM EXIT
  echo
  echo "Detached mode: simulation running in background."
  echo "To stop later: ros2 run nav_worlds bringup.sh --stop"
  exit 0
fi

echo
echo "Simulation running. Press Ctrl+C to stop (or close the Gazebo window)."

while true; do
  if [ -n "${SIM_PID:-}" ] && ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "Simulation process ended."
    break
  fi
  sleep 1
done
