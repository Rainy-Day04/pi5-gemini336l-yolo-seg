#!/usr/bin/env bash
set -euo pipefail

readonly firmware_version="1.8.10"
readonly sdk_version="2.9.3"
readonly sdk_archive="OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_arm64.tar.gz"
readonly sdk_url="https://gitee.com/orbbecdeveloper/OrbbecSDK_v2/releases/download/v${sdk_version}/${sdk_archive}"
readonly sdk_sha256="ce2c476c283b932181b04daf44debadff9e4a743344bd69eecf34f8c18009ac1"
readonly firmware_archive="Gemini330_Release_${firmware_version}.zip"
readonly firmware_url="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/${firmware_archive}"
readonly firmware_sha256="2dc3eea4496ddd97f271fd63f26197e2c24f256d1eecc8d632fcbc99af895eca"
readonly firmware_md5="4a84ea72103b5a255ded76cba2f873b8"

serial_number="${1:-}"
if [[ -z "${serial_number}" ]]; then
  echo "Usage: $0 CAMERA_SERIAL" >&2
  echo "Example: $0 CPC8763000VT" >&2
  exit 2
fi

case "$(uname -m)" in
  aarch64|arm64) ;;
  *)
    echo "This helper uses the official Linux ARM64 SDK and must run on the Raspberry Pi." >&2
    exit 1
    ;;
esac

for command_name in curl unzip tar cmake c++ sha256sum md5sum lsusb; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Missing command: ${command_name}" >&2
    echo "Install prerequisites with: sudo apt install build-essential cmake curl unzip usbutils" >&2
    exit 1
  fi
done

camera_count="$(lsusb -d 2bc5:0807 2>/dev/null | wc -l)"
if [[ "${camera_count}" -ne 1 ]]; then
  echo "Expected exactly one Gemini 336L (USB 2bc5:0807), found ${camera_count}." >&2
  echo "Connect only the target camera before updating." >&2
  exit 1
fi

cache_base="${XDG_CACHE_HOME:-${HOME}/.cache}/pi5-gemini336l-yolo-seg/orbbec-firmware"
download_dir="${cache_base}/downloads"
sdk_dir="${cache_base}/sdk-${sdk_version}"
build_dir="${cache_base}/build-${sdk_version}"
firmware_dir="${cache_base}/firmware-${firmware_version}"
mkdir -p "${download_dir}" "${firmware_dir}"

download_and_verify() {
  local url="$1"
  local destination="$2"
  local expected_sha256="$3"

  if [[ -f "${destination}" ]] && ! echo "${expected_sha256}  ${destination}" | sha256sum --check --status; then
    echo "Discarding incomplete or unexpected download: ${destination}"
    rm -f -- "${destination}"
  fi
  if [[ ! -f "${destination}" ]]; then
    curl --fail --location --retry 5 --retry-all-errors \
      --continue-at - --output "${destination}" "${url}"
  fi
  echo "${expected_sha256}  ${destination}" | sha256sum --check
}

sdk_path="${download_dir}/${sdk_archive}"
firmware_zip="${download_dir}/${firmware_archive}"
download_and_verify "${sdk_url}" "${sdk_path}" "${sdk_sha256}"
download_and_verify "${firmware_url}" "${firmware_zip}" "${firmware_sha256}"

if [[ ! -f "${sdk_dir}/examples/CMakeLists.txt" ]]; then
  rm -rf -- "${sdk_dir}"
  mkdir -p "${sdk_dir}"
  tar -xzf "${sdk_path}" -C "${sdk_dir}" --strip-components=1
fi

firmware_bin="${firmware_dir}/Gemini330_Release_${firmware_version}.bin"
if [[ ! -f "${firmware_bin}" ]]; then
  unzip -j -o "${firmware_zip}" \
    "*/Gemini330_Release_${firmware_version}.bin" -d "${firmware_dir}"
fi
echo "${firmware_md5}  ${firmware_bin}" | md5sum --check

cmake -S "${sdk_dir}/examples" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DOB_BUILD_LINUX=ON \
  -DOB_BUILD_PCL_EXAMPLES=OFF \
  -DOB_BUILD_OPEN3D_EXAMPLES=OFF
cmake --build "${build_dir}" --target ob_device_firmware_update --parallel 2

updater="${build_dir}/bin/ob_device_firmware_update"
if [[ ! -x "${updater}" ]]; then
  echo "Firmware updater was not built: ${updater}" >&2
  exit 1
fi

export LD_LIBRARY_PATH="${sdk_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo
echo "Connected Orbbec devices:"
"${updater}" --list_devices

cat <<EOF

Target serial:   ${serial_number}
Target firmware: Gemini 330 series ${firmware_version} (includes Gemini 336L)

Before continuing:
  1. Stop only the Orbbec camera launch; the chassis nodes may keep running.
  2. Use a stable Raspberry Pi power supply and a direct, reliable USB 3 cable.
  3. Do not disconnect power or USB until this program reports completion.
  4. One or two USB disconnect/reconnect cycles during the update are expected.
EOF

read -r -p "Type the camera serial (${serial_number}) to start the firmware update: " confirmation
if [[ "${confirmation}" != "${serial_number}" ]]; then
  echo "Serial confirmation did not match; no firmware was written." >&2
  exit 1
fi

"${updater}" --serial_number "${serial_number}" --file "${firmware_bin}"

echo
echo "Firmware updater finished. Wait for the camera to reconnect, then verify with:"
echo "  ${updater} --list_devices"
