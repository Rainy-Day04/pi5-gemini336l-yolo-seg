#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(basename "$(dirname "${repo_dir}")")" == "src" ]]; then
  default_workspace="$(cd "${repo_dir}/../.." && pwd)"
else
  default_workspace="${HOME}/gemini336l_ws"
fi

workspace_dir="${1:-${default_workspace}}"
driver_dir="${workspace_dir}/src/OrbbecSDK_ROS2"
patch_file="${repo_dir}/patches/orbbec_ros2_v2.4.3_skip_disabled_noise_filter.patch"
source_file="orbbec_camera/src/ob_camera_node.cpp"

if [[ ! -d "${driver_dir}/.git" || ! -f "${driver_dir}/${source_file}" ]]; then
  echo "OrbbecSDK_ROS2 checkout not found: ${driver_dir}" >&2
  exit 1
fi

driver_version="$(git -C "${driver_dir}" describe --tags --always 2>/dev/null || true)"
if [[ "${driver_version}" != v2.4.3* ]]; then
  echo "This patch is only for OrbbecSDK_ROS2 v2.4.3; found: ${driver_version:-unknown}" >&2
  exit 1
fi

if git -C "${driver_dir}" apply --check "${patch_file}" 2>/dev/null; then
  git -C "${driver_dir}" apply "${patch_file}"
  echo "Applied the Gemini 336L firmware 1.4.60 noise-filter compatibility patch."
elif git -C "${driver_dir}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "Compatibility patch is already applied."
else
  echo "The Orbbec source differs from clean v2.4.3; patch was not applied." >&2
  echo "Run: git -C ${driver_dir} status --short" >&2
  exit 1
fi

cat <<EOF

Rebuild the patched SDK2 driver:
  cd ${workspace_dir}
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  colcon build --symlink-install --packages-up-to orbbec_camera --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF
