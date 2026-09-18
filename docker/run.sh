#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${GEMINI336L_DOCKER_IMAGE:-gemini336l-perception:local}"

docker run --rm -it \
  --network host \
  --ipc host \
  --privileged \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  -v "${repo_dir}/models:/models:ro" \
  "${image}" "$@"
