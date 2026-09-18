#!/usr/bin/env bash
set -euo pipefail

source_environment() {
  local setup_file="$1"
  # ROS 2 Jazzy setup files read AMENT_TRACE_SETUP_FILES before defining it.
  # Temporarily relax nounset, then restore the installer's strict mode.
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

update_apt_indexes() {
  if [[ "${SKIP_APT_UPDATE:-0}" == "1" ]]; then
    echo "Skipping apt index update because SKIP_APT_UPDATE=1."
    return 0
  fi

  local attempt
  for attempt in 1 2 3; do
    echo "Updating apt indexes (attempt ${attempt}/3)..."
    if sudo apt-get update; then
      return 0
    fi
    if (( attempt < 3 )); then
      echo "apt update failed; the mirror may be synchronizing. Retrying shortly..." >&2
      sleep $((attempt * 5))
    fi
  done

  echo "apt indexes could not be updated after 3 attempts." >&2
  echo "If the error says 'Mirror sync in progress', wait or change only the ROS 2 mirror, then rerun this installer." >&2
  return 1
}

clone_orbbec_driver() {
  local driver_dir="$1"
  local backup_dir
  local repository_url
  local attempt
  local -a repository_urls

  if [[ -d "${driver_dir}/.git" ]] \
    && git -C "${driver_dir}" rev-parse --verify 'HEAD^{commit}' >/dev/null 2>&1 \
    && [[ -f "${driver_dir}/orbbec_camera/package.xml" ]]; then
    echo "Using existing Orbbec driver: ${driver_dir}"
    return 0
  fi

  if [[ -e "${driver_dir}" ]]; then
    backup_dir="${driver_dir}.incomplete.$(date +%Y%m%d%H%M%S).${RANDOM}"
    echo "Moving incomplete driver checkout to ${backup_dir}" >&2
    mv -- "${driver_dir}" "${backup_dir}"
  fi

  if [[ -n "${ORBBEC_REPOSITORY_URL:-}" ]]; then
    repository_urls=("${ORBBEC_REPOSITORY_URL}")
  else
    repository_urls=(
      "https://github.com/orbbec/OrbbecSDK_ROS2.git"
      "https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2.git"
    )
  fi

  for repository_url in "${repository_urls[@]}"; do
    for attempt in 1 2 3; do
      echo "Cloning Orbbec driver from ${repository_url} (attempt ${attempt}/3)..."
      if git -c http.version=HTTP/1.1 clone \
        --depth 1 \
        --single-branch \
        --branch v2-main \
        --filter=blob:none \
        "${repository_url}" "${driver_dir}"; then
        return 0
      fi
      if [[ -e "${driver_dir}" ]]; then
        backup_dir="${driver_dir}.incomplete.$(date +%Y%m%d%H%M%S).${RANDOM}"
        mv -- "${driver_dir}" "${backup_dir}"
      fi
      sleep $((attempt * 3))
    done
  done

  echo "Unable to clone the Orbbec driver from all configured mirrors." >&2
  return 1
}

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(basename "$(dirname "${repo_dir}")")" == "src" ]]; then
  default_workspace="$(cd "${repo_dir}/../.." && pwd)"
else
  default_workspace="${HOME}/gemini336l_ws"
fi
workspace_dir="${1:-${default_workspace}}"

if [[ "$(uname -m)" != "aarch64" && "$(uname -m)" != "arm64" ]]; then
  echo "Warning: this installer is tuned for Raspberry Pi 5 ARM64." >&2
fi

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

update_apt_indexes
sudo apt-get install -y \
  build-essential cmake git libdw-dev libgflags-dev libgl1 libgoogle-glog-dev \
  libssl-dev libusb-1.0-0-dev mesa-utils nlohmann-json3-dev \
  python3-colcon-common-extensions python3-pip python3-rosdep python3-venv \
  "ros-${ROS_DISTRO}-backward-ros" \
  "ros-${ROS_DISTRO}-camera-info-manager" \
  "ros-${ROS_DISTRO}-compressed-image-transport" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-diagnostic-msgs" \
  "ros-${ROS_DISTRO}-diagnostic-updater" \
  "ros-${ROS_DISTRO}-image-publisher" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-image-transport-plugins" \
  "ros-${ROS_DISTRO}-statistics-msgs" \
  "ros-${ROS_DISTRO}-xacro"

mkdir -p "${workspace_dir}/src"
link_path="${workspace_dir}/src/pi5-gemini336l-yolo-seg"
if [[ "$(cd "${workspace_dir}/src" && pwd)" != "$(dirname "${repo_dir}")" ]]; then
  if [[ ! -e "${link_path}" ]]; then
    ln -s "${repo_dir}" "${link_path}"
  fi
fi

if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
  driver_dir="${workspace_dir}/src/OrbbecSDK_ROS2"
  clone_orbbec_driver "${driver_dir}"
  sudo bash "${driver_dir}/orbbec_camera/scripts/install_udev_rules.sh"
else
  rules="/opt/ros/${ROS_DISTRO}/share/orbbec_camera/udev/99-obsensor-libusb.rules"
  if [[ -f "${rules}" ]]; then
    sudo install -m 0644 "${rules}" /etc/udev/rules.d/99-obsensor-libusb.rules
  fi
fi
sudo udevadm control --reload-rules
sudo udevadm trigger

if [[ ! -d "${workspace_dir}/.venv" ]]; then
  python3 -m venv --system-site-packages "${workspace_dir}/.venv"
fi
source_environment "${workspace_dir}/.venv/bin/activate"
python -m pip install --upgrade pip wheel
python -m pip install -r "${repo_dir}/requirements.txt"

# A previously interrupted PyPI install may have left CUDA packages in this
# dedicated venv. CPU PyTorch does not use them, so remove them to recover disk.
mapfile -t cuda_packages < <(
  python -m pip list --format=freeze \
    | sed -n 's/^\(nvidia-[^=]*-cu13\)==.*/\1/p'
)
if (( ${#cuda_packages[@]} > 0 )); then
  echo "Removing unused CUDA 13 packages from the Pi venv: ${cuda_packages[*]}"
  python -m pip uninstall -y "${cuda_packages[@]}"
fi

python - <<'PY'
import torch

if torch.version.cuda is not None:
    raise SystemExit(f"Expected CPU-only PyTorch, got CUDA build {torch.version.cuda}")
print(f"CPU-only PyTorch ready: {torch.__version__}")
PY

if ! rosdep db >/dev/null 2>&1; then
  sudo rosdep init 2>/dev/null || true
  rosdep update
fi
rosdep install --from-paths "${workspace_dir}/src" --ignore-src -r -y \
  --rosdistro "${ROS_DISTRO}"

cd "${workspace_dir}"
colcon build --symlink-install --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

if [[ "${SKIP_MODEL_EXPORT:-0}" != "1" ]]; then
  "${repo_dir}/scripts/prepare_model.sh" "${workspace_dir}" 320
fi

echo
echo "Installation complete. Reconnect the Gemini 336L, then run:"
echo "  ${repo_dir}/scripts/run.sh all ${workspace_dir}"
