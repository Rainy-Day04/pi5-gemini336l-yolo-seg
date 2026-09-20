#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${1:-${HOME}/robot_team_ws}"
distro="${ROS_DISTRO:-jazzy}"
if [[ ! -f "/opt/ros/${distro}/setup.bash" ]]; then
  echo "Install/source the same ROS 2 distribution used by your robot first." >&2
  exit 1
fi
set +u
source "/opt/ros/${distro}/setup.bash"
set -u
if ! command -v rosdep >/dev/null || ! command -v colcon >/dev/null; then
  echo "Install python3-rosdep and python3-colcon-common-extensions first." >&2
  exit 1
fi
rosdep install --from-paths "${repo_dir}/gemini336l_msgs" \
  "${repo_dir}/robot_task_coordinator" "${repo_dir}/times_arm_perception" \
  --ignore-src -r -y --rosdistro "${distro}"
mkdir -p "${workspace_dir}"
cd "${workspace_dir}"
# Explicit roots avoid Orbbec backup packages and do not build the chassis stack.
colcon build --symlink-install --base-paths "${repo_dir}/gemini336l_msgs" \
  "${repo_dir}/robot_task_coordinator" "${repo_dir}/times_arm_perception" \
  --packages-up-to robot_task_coordinator times_arm_perception
echo "Ready. Source ${workspace_dir}/install/setup.bash"
echo "Preview only: ros2 launch robot_task_coordinator task.launch.py"
