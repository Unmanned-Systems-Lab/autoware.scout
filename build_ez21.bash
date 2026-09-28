#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec env -i HOME="$HOME" USER="${USER:-agx}" LANG=C.UTF-8 \
  PATH=/usr/local/cuda-12.6/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    export CMAKE_PREFIX_PATH="$PWD/.deps/usr/local:/opt/acados:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$PWD/.deps/usr/local/lib:/opt/acados/lib:/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}"
    export ACADOS_SOURCE_DIR="$PWD/src/universe/external/acados"
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
    exec colcon build --symlink-install --parallel-workers 4 "$@" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DCMAKE_CUDA_ARCHITECTURES=87
  ' bash "$@"
