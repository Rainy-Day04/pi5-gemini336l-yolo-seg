#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${1:-${HOME}/gemini336l_ws}"
distro="${ROS_DISTRO:-jazzy}"
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Build/install the existing perception workspace first: ${workspace_dir}" >&2
  exit 1
fi
set +u
source "/opt/ros/${distro}/setup.bash"
source "${workspace_dir}/install/setup.bash"
set -u
rosdep install --from-paths "${repo_dir}/times_arm_perception" \
  --ignore-src -r -y --rosdistro "${distro}"
cd "${workspace_dir}"
# Restrict discovery to the new package; Orbbec backups cannot cause duplicates.
colcon build --symlink-install --base-paths "${repo_dir}/times_arm_perception" \
  --packages-select times_arm_perception
echo "Ready: ${repo_dir}/scripts/run.sh arm ${workspace_dir} mode:=preview"
