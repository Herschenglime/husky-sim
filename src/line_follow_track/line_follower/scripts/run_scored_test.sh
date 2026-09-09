#!/usr/bin/env bash
# Launch the sim + follower, score one run against the painted centreline, tear
# everything down. Exit status is the run's PASS/FAIL.
#
#   ./run_scored_test.sh [duration_seconds] [extra score_run args...]
#
# Teardown kills the launch's process group by PGID rather than by pattern:
# this script's own command line contains the patterns a pkill -f would match,
# so pattern-killing can kill the script mid-teardown and silently skip the rest.
# No `set -u`: ROS's setup.bash reads unset variables and would abort here.
set -o pipefail

DURATION="${1:-120}"
shift || true

: "${ROS_DISTRO:=jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# Walk up from this script until a built workspace turns up, rather than
# assuming a fixed depth: the packages may sit at src/<pkg> or be nested a
# level deeper inside a bundle directory.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [ "${WS}" != "/" ] && [ ! -f "${WS}/install/setup.bash" ]; do
  WS="$(dirname "${WS}")"
done
if [ ! -f "${WS}/install/setup.bash" ]; then
  echo "FAIL: no built workspace above $(dirname "${BASH_SOURCE[0]}")."
  echo "      Build first: colcon build --symlink-install"
  exit 1
fi
source "${WS}/install/setup.bash"

CENTERLINE="$(ros2 pkg prefix line_follow_sim)/share/line_follow_sim/config/track_centerline.csv"
LOG="$(mktemp -t line_follow_run.XXXXXX.log)"
OUT="${OUT:-/tmp/line_follow_samples.csv}"

echo "launching sim + follower (log: ${LOG})"
setsid ros2 launch line_follower follow.launch.py > "${LOG}" 2>&1 &
LAUNCH_PID=$!
PGID="$(ps -o pgid= -p "${LAUNCH_PID}" | tr -d ' ')"

cleanup() {
  echo "tearing down process group ${PGID}"
  [ -n "${PGID:-}" ] && kill -9 -"${PGID}" 2>/dev/null
  sleep 2
}
trap cleanup EXIT

echo "waiting for camera and cmd_vel topics"
ready=""
for _ in $(seq 1 60); do
  topics="$(ros2 topic list 2>/dev/null)"
  if grep -q "camera_0/color/image" <<<"${topics}" \
     && grep -q "cmd_vel" <<<"${topics}"; then
    ready=1
    break
  fi
  sleep 3
done
if [ -z "${ready}" ]; then
  echo "FAIL: camera/cmd_vel topics never appeared - the sim did not come up."
  echo "      see ${LOG}"
  exit 1
fi

# The controllers are topped up on a 10 s timer inside the sim launch; without
# them the robot cannot move and the run would score as a stationary failure.
#
# Asked over the service rather than with `ros2 control list_controllers`: that
# CLI lives in ros2controlcli, which is not pulled in by ros-jazzy-controller-
# manager and so is missing on a plain clearpath-simulator install. The service
# is part of controller_manager itself and is always there.
echo "waiting for platform_velocity_controller"
ready=""
for _ in $(seq 1 30); do
  if ros2 service call /a200_0000/controller_manager/list_controllers \
       controller_manager_msgs/srv/ListControllers 2>/dev/null \
     | grep -q "name='platform_velocity_controller', state='active'"; then
    ready=1
    break
  fi
  sleep 3
done
if [ -z "${ready}" ]; then
  echo "FAIL: platform_velocity_controller never activated - the robot cannot move,"
  echo "      so the run would score as a stationary failure. See ${LOG}"
  exit 1
fi

echo "scoring ${DURATION} s of driving"
ros2 run line_follower score_run --centerline "${CENTERLINE}" \
  --duration "${DURATION}" --out "${OUT}" "$@"
