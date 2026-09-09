#!/usr/bin/env bash
# Backward-compatibility wrapper forwarding to a200_point_nav.launch.py.
#
#   bringup.sh [world] [--localize] [--gui]
#
# Prefer running directly via ROS 2 launch:
#   ros2 launch nav_worlds a200_point_nav.launch.py world:=warehouse
#
set -o pipefail

: "${ROS_DISTRO:=jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"

WORLD=""
SLAM="true"
HEADLESS="${HEADLESS:-true}"

for arg in "$@"; do
  case "$arg" in
    --gui) HEADLESS="false" ;;
    --headless) HEADLESS="true" ;;
    --localize) SLAM="false" ;;
    --stop|stop|--kill)
      echo "Stopping running simulation and navigation processes..."
      pkill -9 -f "gz sim|ruby.*gz|ros_gz_bridge|parameter_bridge|scan_self_filter|slam_toolbox|nav2_|clearpath_nav2_demos|sim.launch.py" 2>/dev/null || true
      echo "Done."
      exit 0
      ;;
    -*)
      echo "Unknown option: $arg"
      exit 1
      ;;
    *)
      [ -z "$WORLD" ] && WORLD="$arg"
      ;;
  esac
done

WORLD="${WORLD:-warehouse}"

exec ros2 launch nav_worlds a200_point_nav.launch.py \
  world:="${WORLD}" \
  slam:="${SLAM}" \
  headless:="${HEADLESS}"

