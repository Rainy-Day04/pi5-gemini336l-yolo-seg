#!/usr/bin/env bash
set -euo pipefail

source_environment() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

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
source_environment "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Workspace is not installed: ${workspace_dir}. Run scripts/install.sh first." >&2
  exit 1
fi
source_environment "${workspace_dir}/install/setup.bash"

# Team interfaces and arm API adapter use native ROS packages only. They must
# also work in a separate MiniPC workspace without YOLO, Orbbec or a venv.
case "${mode}" in
  arm) exec ros2 launch times_arm_perception integration.launch.py "$@" ;;
  task) exec ros2 launch robot_task_coordinator task.launch.py "$@" ;;
esac
if [[ ! -f "${workspace_dir}/.venv/bin/activate" ]]; then
  echo "Perception venv missing. Run scripts/install.sh first." >&2
  exit 1
fi
source_environment "${workspace_dir}/.venv/bin/activate"

# ROS 2 console scripts keep the Python interpreter that was used by colcon in
# their shebang.  When that is /usr/bin/python3, merely activating the venv is
# not enough for the node to see ultralytics and the NCNN runtime installed in
# the workspace venv.  Make those packages visible without changing the ROS 2
# interpreter or installing them globally.
venv_python="${workspace_dir}/.venv/bin/python"
venv_site_packages="$("${venv_python}" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${venv_site_packages}${PYTHONPATH:+:${PYTHONPATH}}"

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
    echo "Usage: $0 [all|camera|perception|arm|task] [workspace] [launch arguments...]" >&2
    exit 2
    ;;
esac
