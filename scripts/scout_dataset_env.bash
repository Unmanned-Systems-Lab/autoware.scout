#!/usr/bin/env bash
# Source only the packages needed for collection, without loading Autoware.
dataset_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
zed_ws="/mnt/nvme0n1/autoware/zed_ros2_ws"
export ZED_DIR="/mnt/nvme0n1/autoware/zed_sdk_5.5.1"
export SCOUT_DATASET_ZED_CONFIG="$dataset_repo_dir/config/zed2i_640x360_15hz.yaml"

if [[ ! -f "$zed_ws/install/local_setup.bash" || ! -f "$SCOUT_DATASET_ZED_CONFIG" || ! -d "$ZED_DIR" ]]; then
  echo "缺少 ZED SDK、官方 ROS 2 wrapper 或相机配置，请查看 SCOUT_DATASET.md。" >&2
  return 1
fi

source /opt/ros/humble/setup.bash
for dataset_package in scout_msgs scout_base fixposition_driver_msgs \
  fixposition_driver_ros2 scout_sensor_kit_launch scout_vehicle_description; do
  source "$dataset_repo_dir/install/$dataset_package/share/$dataset_package/local_setup.bash"
done
source /mnt/nvme0n1/autoware/autoware.ez21/install/rslidar_sdk/share/rslidar_sdk/local_setup.bash
source "$zed_ws/install/local_setup.bash"
unset dataset_package dataset_repo_dir zed_ws
