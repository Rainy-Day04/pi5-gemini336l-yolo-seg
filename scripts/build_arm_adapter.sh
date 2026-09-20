#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${1:-${HOME}/gemini336l_ws}"
# Build the new Action definitions too; rebuilding only the Python adapter is
# insufficient for installations with the old gemini336l_msgs package.
exec bash "${repo_dir}/scripts/build_team_interfaces.sh" "${workspace_dir}"
