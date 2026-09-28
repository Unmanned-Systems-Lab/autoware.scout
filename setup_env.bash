# Source this file before using the EZ21 workspace.
_ez21_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$_ez21_ws/install/setup.bash"
export PATH="/usr/local/cuda-12.6/bin:$PATH"
export ACADOS_SOURCE_DIR="$_ez21_ws/src/universe/external/acados"
export LD_LIBRARY_PATH="$_ez21_ws/.deps/usr/local/lib:$_ez21_ws/install/acados/lib:/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$_ez21_ws/.deps/usr/local:${CMAKE_PREFIX_PATH:-}"
_ez21_cupti="$HOME/.local/lib/python3.10/site-packages/nvidia/cuda_cupti/lib"
if [ -d "$_ez21_cupti" ]; then
  export LD_LIBRARY_PATH="$_ez21_cupti:$LD_LIBRARY_PATH"
fi
unset _ez21_cupti _ez21_ws
