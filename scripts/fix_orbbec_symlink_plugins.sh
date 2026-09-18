#!/usr/bin/env bash
set -euo pipefail

workspace_dir="${1:-${HOME}/gemini336l_ws}"
driver_dir="${workspace_dir}/src/OrbbecSDK_ROS2"

case "$(uname -m)" in
  aarch64|arm64) architecture="arm64" ;;
  x86_64|amd64) architecture="x64" ;;
  *)
    echo "Unsupported architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

source_dir="${driver_dir}/orbbec_camera/SDK/lib/${architecture}/extensions/filters"
destination_dir="${workspace_dir}/install/orbbec_camera/lib/extensions/filters"

for directory in "${source_dir}" "${destination_dir}"; do
  if [[ ! -d "${directory}" ]]; then
    echo "Required directory not found: ${directory}" >&2
    exit 1
  fi
done

for plugin in libFilterProcessor.so libob_priv_filter.so; do
  source_file="${source_dir}/${plugin}"
  destination_file="${destination_dir}/${plugin}"
  temporary_file="${destination_dir}/.${plugin}.regular.$$"

  if [[ ! -e "${source_file}" ]]; then
    echo "Required plugin not found: ${source_file}" >&2
    exit 1
  fi

  cp -L -p "${source_file}" "${temporary_file}"
  mv -f -- "${temporary_file}" "${destination_file}"
done

echo "Orbbec filter plugin file types:"
for plugin in libFilterProcessor.so libob_priv_filter.so; do
  destination_file="${destination_dir}/${plugin}"
  if [[ ! -f "${destination_file}" || -L "${destination_file}" ]]; then
    echo "Failed to materialize regular file: ${destination_file}" >&2
    exit 1
  fi
  echo "  regular file: ${destination_file}"
done

echo "Fix complete. Restart the Orbbec camera process."
