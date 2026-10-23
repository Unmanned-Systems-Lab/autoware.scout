#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
can_interface="${1:-can10}"
if [[ $# -gt 1 || ! "$can_interface" =~ ^can[0-9]+$ ]]; then
  echo "用法: $0 [can接口，默认can10]" >&2
  exit 2
fi

source "$repo_dir/setup_env.bash"
zed_ws="/mnt/nvme0n1/autoware/zed_ros2_ws"
export ZED_DIR="/mnt/nvme0n1/autoware/zed_sdk_5.5.1"
export SCOUT_DATASET_ZED_CONFIG="$repo_dir/config/zed2i_640x360_15hz.yaml"
if [[ ! -f "$zed_ws/install/local_setup.bash" || ! -f "$SCOUT_DATASET_ZED_CONFIG" || ! -d "$ZED_DIR" ]]; then
  echo "缺少 ZED SDK、官方 ROS 2 wrapper 或相机配置，请查看 SCOUT_DATASET.md。" >&2
  exit 1
fi
source "$zed_ws/install/local_setup.bash"
set -u

existing_nodes="$(ros2 node list 2>/dev/null || true)"
conflicts="$(printf '%s\n' "$existing_nodes" | grep -E '^(/scout_base|/fixposition_driver|/rslidar_sdk|/zed2i/zed_node)$' || true)"
if [[ -n "$conflicts" ]]; then
  echo "已有驱动节点运行，不能重复占用 CAN、雷达、RTK 或相机：" >&2
  printf '%s\n' "$conflicts" >&2
  echo "若这些节点正在正常发布，可直接运行 ./record_scout_dataset.bash。" >&2
  exit 1
fi

if ! ip -brief link show "$can_interface" 2>/dev/null | grep -Eq '[[:space:]]UP[[:space:]]'; then
  if [[ "$can_interface" != "can10" ]]; then
    echo "$can_interface 未启用；自动引脚配置只支持已验证的 can10。" >&2
    exit 1
  fi
  echo "can10 未启用，配置 Jetson 原生 CAN0 引脚及 500 kbps（需要 sudo）。"
  sudo -v
  sudo ip link set can10 down
  sudo busybox devmem 0x0c303010 32 0x0000C400 >/dev/null
  sudo busybox devmem 0x0c303018 32 0x0000C458 >/dev/null
  sudo ip link set can10 up type can bitrate 500000
fi
if ! ip -details link show "$can_interface" | grep -q 'bitrate 500000'; then
  echo "$can_interface 比特率不是已验证的 500 kbps；未启动底盘节点。" >&2
  exit 1
fi

echo "启动独立数据采集驱动：Scout($can_interface)、Fixposition、RSHELIOS、ZED 2i。"
echo "终端保持打开；停止时按 Ctrl+C。此启动不包含 Autoware 规划或控制。"
exec ros2 launch "$repo_dir/launch/scout_dataset.launch.py" "can_interface:=$can_interface"
