#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
port="${SCOUT_UI_PORT:-8092}"

if [[ ! -f "${workspace_root}/install/setup.bash" ]]; then
  echo "Scout workspace is not built: ${workspace_root}/install/setup.bash" >&2
  exit 1
fi
if ss -ltn "( sport = :${port} )" | grep -q LISTEN; then
  echo "Port ${port} is already in use; choose another SCOUT_UI_PORT." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${SCOUT_UI_ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SCOUT_UI_ALLOW_VEHICLE_ACTIONS=0

set +u
source "${workspace_root}/setup_env.bash"
set -u

echo "Scout UI: http://127.0.0.1:${port}/index.html"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; vehicle actions disabled"
exec bash "${workspace_root}/src/launcher/autoware_launch/ui/start_ui_server.sh" \
  127.0.0.1 "${port}" "${SCOUT_UI_OPEN_BROWSER:-false}"
