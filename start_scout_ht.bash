#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
imported_ele="$(dirname -- "$repo_dir")/scout_dependencies/ele_scouts_kit"
if [[ -n "${SCOUT_HT_ELE_WORKSPACE:-}" ]]; then
  ele_ws="$SCOUT_HT_ELE_WORKSPACE"
elif [[ -f "$imported_ele/install/local_setup.bash" ]]; then
  ele_ws="$imported_ele"
else
  ele_ws="/home/agx/ele_scout_kit_github_20260827"
fi
if [[ ! -f "$ele_ws/install/local_setup.bash" ]]; then
  echo "Selected ELE Scout workspace is not built: $ele_ws" >&2
  exit 1
fi
source "$repo_dir/setup_env.bash"
export SCOUT_HT_MAP_ROOT="${SCOUT_HT_MAP_ROOT:-$(dirname -- "$repo_dir")/scout_elevation_maps}"
source "$ele_ws/install/local_setup.bash"
if [[ -f "$ele_ws/setup_runtime_env.sh" ]]; then
  source "$ele_ws/setup_runtime_env.sh"
fi
set -u
expected="$ele_ws/install/ht_traversability_ros2"
actual="$(ros2 pkg prefix ht_traversability_ros2)"
if [[ "$(realpath "$actual")" != "$(realpath "$expected")" ]]; then
  echo "Wrong HT package selected: $actual (expected $expected)" >&2
  exit 1
fi
echo "Observation only: no chassis, sensor drivers or planner/control remappings."
echo "ELE source: $ele_ws; inference and elevation are opt-in launch arguments."
exec ros2 launch scout_ht_traversability observe.launch.py "$@"
