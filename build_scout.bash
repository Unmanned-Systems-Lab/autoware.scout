#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec env -i HOME="$HOME" USER="${USER:-user}" LANG=C.UTF-8 \
  SCOUT_UNDERLAY="${SCOUT_UNDERLAY:-}" \
  PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    if [ -n "$SCOUT_UNDERLAY" ]; then source "$SCOUT_UNDERLAY/setup.bash"; fi
    export CMAKE_PREFIX_PATH="$PWD/.deps/usr/local:/opt/acados:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$PWD/.deps/usr/local/lib:/opt/acados/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
    exec colcon build --symlink-install --parallel-workers 4 "$@" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  ' bash "$@"
