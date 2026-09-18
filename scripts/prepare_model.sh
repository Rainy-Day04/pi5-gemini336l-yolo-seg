#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${1:-$(cd "${repo_dir}/../.." 2>/dev/null && pwd)}"
image_size="${2:-320}"

if [[ ! -d "${workspace_dir}/.venv" ]]; then
  echo "Missing ${workspace_dir}/.venv; run scripts/install.sh first." >&2
  exit 1
fi

source "${workspace_dir}/.venv/bin/activate"
mkdir -p "${repo_dir}/models"
cd "${repo_dir}/models"

if [[ -d "yolo26n-seg_ncnn_model" ]]; then
  echo "Model already exists: ${repo_dir}/models/yolo26n-seg_ncnn_model"
  exit 0
fi

echo "Downloading YOLO26n-seg and exporting NCNN (${image_size}x${image_size})..."
yolo export model=yolo26n-seg.pt format=ncnn imgsz="${image_size}" batch=1 device=cpu
echo "Model ready: ${repo_dir}/models/yolo26n-seg_ncnn_model"
