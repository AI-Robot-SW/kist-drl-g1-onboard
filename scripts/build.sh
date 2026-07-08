#!/usr/bin/env bash
# build.sh — colcon build wrapper for kist-drl-g1-onboard
#
# Usage:
#   ./scripts/build.sh                 # build everything, symlink install
#   ./scripts/build.sh sensors         # build a single package
#   ROS_DISTRO=jazzy ./scripts/build.sh   # override distro (default: humble)

set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# Ensure the shared g1_onboard_msgs submodule is materialised before building.
# A submodule is a pinned pointer; a fresh clone (or a pin bump) leaves its dir
# empty/stale until inited, which would make colcon find 0 packages.
git submodule update --init --recursive

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # ROS setup references unset vars (AMENT_TRACE_SETUP_FILES)
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "[build.sh] WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found." >&2
  echo "[build.sh] Set ROS_DISTRO=<your-distro> or install ROS 2." >&2
fi

ARGS=( --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo )

if [ "$#" -gt 0 ]; then
  ARGS+=( --packages-select "$@" )
fi

colcon build "${ARGS[@]}"

echo
echo "[build.sh] Done. Source the overlay:"
echo "    source ${REPO_ROOT}/install/setup.bash"
