#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ./setup_env.bash
exec ros2 launch autoware_launch real_scout.launch.xml "$@"
