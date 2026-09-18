#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-all}"
if [[ "$(basename "$(dirname "${repo_dir}")")" == "src" ]]; then
  default_workspace="$(cd "${repo_dir}/../.." && pwd)"
else
  default_workspace="${HOME}/gemini336l_ws"
fi
workspace_dir="${2:-${default_workspace}}"
shift $(( $# > 0 ? 1 : 0 ))
shift $(( $# > 0 ? 1 : 0 ))

if [[ -z "${ROS_DISTRO:-}" ]]; then
  for distro in jazzy humble; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      export ROS_DISTRO="${distro}"
      break
    fi
  done
fi
if [[ -z "${ROS_DISTRO:-}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 Humble or Jazzy must already be installed under /opt/ros." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "${workspace_dir}/.venv/bin/activate" || ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Workspace is not installed: ${workspace_dir}. Run scripts/install.sh first." >&2
  exit 1
fi
source "${workspace_dir}/.venv/bin/activate"
source "${workspace_dir}/install/setup.bash"

model_path="${GEMINI336L_MODEL:-${repo_dir}/models/yolo26n-seg_ncnn_model}"
export GEMINI336L_MODEL="${model_path}"

case "${mode}" in
  all)
    exec ros2 launch gemini336l_bringup all.launch.py model:="${model_path}" "$@"
    ;;
  camera)
    exec ros2 launch gemini336l_bringup camera.launch.py "$@"
    ;;
  perception)
    exec ros2 launch gemini336l_bringup perception.launch.py model:="${model_path}" "$@"
    ;;
  *)
    echo "Usage: $0 [all|camera|perception] [workspace] [launch arguments...]" >&2
    exit 2
    ;;
esac
