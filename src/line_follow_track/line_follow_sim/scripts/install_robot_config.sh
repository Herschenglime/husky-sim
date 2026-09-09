#!/usr/bin/env bash
# Install the robot.yaml this sim needs into the Clearpath setup directory.
#
# The follower needs a camera aimed at the floor, and that lives in the
# Clearpath robot config - outside these packages - so a fresh checkout has no
# camera and the follower sits silent. This copies the config in, keeping a
# timestamped backup of whatever was there.
#
#   ./install_robot_config.sh [setup_dir]     (default: ~/clearpath)
set -e

source /opt/ros/jazzy/setup.bash


SETUP_DIR="${1:-$HOME/clearpath}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/../config" && pwd)/robot.yaml"
DST="${SETUP_DIR}/robot.yaml"

if [ ! -d "${SETUP_DIR}" ]; then
  echo "No Clearpath setup directory at ${SETUP_DIR}."
  echo "Install ros-<distro>-clearpath-simulator and create it first."
  exit 1
fi

if [ -f "${DST}" ]; then
  if cmp -s "${SRC}" "${DST}"; then
    echo "${DST} already matches - nothing to do."
    exit 0
  fi
  BACKUP="${DST}.backup.$(date +%Y%m%d-%H%M%S)"
  cp "${DST}" "${BACKUP}"
  echo "backed up existing config to ${BACKUP}"
fi

cp "${SRC}" "${DST}"
echo "installed ${DST}"
echo "The camera is mounted on sensor_arch_mount, 0.30 m forward, pitched 0.45 rad down."
