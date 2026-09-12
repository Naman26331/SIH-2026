#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${repo_root}/backend/ros2_ws/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "Missing ROS 2 ${ros_distro}: ${ros_setup}" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "Build first: cd ${repo_root}/backend/ros2_ws && colcon build --symlink-install" >&2
  exit 1
fi

source "${ros_setup}"
source "${workspace_setup}"
export FLEETX_SIMULATION=false
exec python3 "${repo_root}/backend/run.py" "$@"
