# Autoware Scout 迁移交接与启动手册

交接日期：2026-10-30。适用于本次导出的 Autoware Scout + ELE Scout 源码快照。

## 1. 交接内容及当前状态

请同时带走以下文件，不要只带 Autoware：

| 文件 | 用途 |
|---|---|
| `autoware.scout_source_20261030_165858.zip` | Autoware、Scout 模型/底盘接口、Fix 定位桥、UI、原始 costmap、HT 融合与可选规划约束 |
| `ele_scouts_kit_source_20261030_165858.zip` | CUDA 高程建图、全局多帧积累与射线清除、HT 八方向节点及模型 |
| `SCOUT_HANDOVER_20261030.zip` | 本手册、原机环境参考、启动占位地图 |
| `SHA256SUMS_20261030_165858.txt` | 两个源码包的校验和 |

Autoware 和 ELE 是两个独立工作区，分别编译、分别维护仓库。ELE 快照已经包含最新修改，**不要再重复应用 Autoware 包里的 ELE 补丁**。
`repositories/scout-external.repos` 仍指向旧基线，不能用它替代本次 ELE 快照。以后上传 ELE 新提交后，再把其中的 `version` 更新为实际提交 SHA。

这两个包是源码交接，不是装好即用的系统镜像：不含 `build/ install/ log/ .deps/`、完整 GPU 环境、历史录包和高程数据库。源码快照的基线提交及文件校验在各包的 `SOURCE_SNAPSHOT.json`、`MANIFEST_SHA256.txt`。

本机 Autoware 的 `install/setup.bash` 还串接了 `/mnt/nvme0n1/autoware/autoware.ez21/install`。因此**不能把原机 install 复制过去就当作独立安装完成**。新机需要完整构建，或有意提供匹配的 underlay；本手册采用完整构建路线。此路线尚未在第二台干净电脑上验收。

已完成的相关验证包括 66 项 ELE/CUDA 测试和 93 项 Scout 回归。现场行人残影复测主增高区域 69 格恢复，仍有 5 个零散格存在邻近实时回波；不代表所有现场高点均已解决。HT 方向阈值、滑移、制动和整车运动验收仍需接收方完成。当前原始 costmap 的 4 cm 分辨率带来较高 GPU、CPU 和传输负载。

## 2. 新电脑环境

原机参考值（不是跨平台强制安装清单）：

| 项目 | 原机 |
|---|---|
| 机器 | NVIDIA Jetson AGX Orin，aarch64 |
| 系统 | Ubuntu 22.04.5，L4T R36.5.0 |
| ROS | ROS 2 Humble，Python 3.10 |
| CUDA | `/usr/local/cuda` 指向 CUDA 12.6，nvcc 12.6.68 |
| TensorRT | 10.3.0.30，软件包版本后缀 cuda12.5 |
| PyTorch / CuPy / NumPy | 2.8.0 / cupy-cuda12x 13.6.0 / 1.23.5 |

如果新机为 x86_64 + NVIDIA 独显，需要安装对应架构的驱动、CUDA、TensorRT、PyTorch 和 CuPy；不要复制 Jetson 的 wheel、`.so` 或 CAN 引脚配置。没有 NVIDIA CUDA 环境时，当前默认 CUDA 感知、高程和 HT 入口不能按原配置直接运行。

先准备 Ubuntu 22.04 / ROS 2 Humble 及该机器匹配的 GPU 开发环境。仓库的 `setup-dev-env.sh` 和 `ansible/` 可作为系统依赖安装入口，但 `amd64.env` / `arm64.env` 的 CUDA/TensorRT 预设与上述原机实际版本不同，执行前先核对；不要为了照抄预设覆盖 Jetson 已有的 JetPack 驱动。

已经具备 ROS 和 GPU 环境后，补齐构建及调试工具：

```bash
sudo apt update
sudo apt install -y build-essential cmake git unzip python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  can-utils netcat-openbsd tcpdump \
  ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-grid-map-rviz-plugin
# 首次安装 rosdep 才执行 sudo rosdep init；已初始化时不要重复执行。
rosdep update
```

完整 Autoware 还使用 acados、spconv/cumm 等平台依赖。本机 `.deps/usr/local` 中有额外 C++ spconv/cumm；ZIP 未带这些库。按编译器实际缺失项和仓库 Ansible 配置安装与目标 CUDA/架构匹配的版本，**pip 的 spconv 不自动等于 CMake 需要的 C++ spconv 包**。构建成功前不要把“已解压”当作“已迁移完成”。

## 3. 解压与目录约定

下面统一使用 `$HOME/scout_ws`，不要求新电脑有 `/mnt/nvme0n1`。把三个 ZIP 先放到新机 `~/Downloads`。

```bash
mkdir -p "$HOME/scout_ws"
unzip "$HOME/Downloads/autoware.scout_source_20261030_165858.zip" -d "$HOME/scout_ws"
unzip "$HOME/Downloads/ele_scouts_kit_source_20261030_165858.zip" -d "$HOME/scout_ws"
unzip "$HOME/Downloads/SCOUT_HANDOVER_20261030.zip" -d "$HOME/scout_ws"
```

预期目录：

```text
~/scout_ws/
  autoware.scout/
  ele_scouts_kit/
  SCOUT_HANDOVER_20261030/
    SCOUT_HANDOVER.md
    ORIGINAL_MACHINE_REFERENCE.json
    bootstrap_map/
```

源码 ZIP 不含 `.git` 历史；部署不需要先 `git init`。不要对已经完整提供的 Autoware `src/` 再做 `vcs import`，否则可能覆盖 Scout 修改。也不要直接 clone 旧 ELE 分支覆盖本次快照。

`bootstrap_map` 是原机启动说明引用的小型生成地图，仅用于让加载链路有初始文件，**不是新场地的已对齐地图**。保留其 `map_projector_info.yaml` 原值；不要手工改原点文件来冒充整张地图已经转换坐标。

## 4. 首次编译

### 4.1 Autoware

在未 source 其他工作区的新终端运行：

```bash
cd "$HOME/scout_ws/autoware.scout"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# 检查 rosdep 的退出状态；有失败项时先补齐依赖，再编译。
unset SCOUT_UNDERLAY
./build_scout.bash
source ./setup_env.bash
ros2 pkg prefix autoware_launch
ros2 pkg prefix scout_ht_traversability
ros2 pkg prefix scout_ht_planning
```

`build_scout.bash` 已隔离其他工作区环境，默认 4 个包并行、每包 2 个编译线程；内存不足可传 `--executor sequential`。它不是依赖安装器，遇到缺库仍需要安装对应库后重试。

首次构建不要使用 README 中依赖旧 EZ21 的 `--packages-select` 增量命令。构建成功后，检查当前 `install/setup.bash` 不再指向原机的 `/mnt/.../autoware.ez21/install`。

### 4.2 ELE 高程与 HT

同样在新终端运行，不要先 source Autoware：

```bash
cd "$HOME/scout_ws/ele_scouts_kit"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "ament_python cupy-cuda12x simple-parsing python3-torch python3-torchvision"
python3 -m pip install --user -r requirements-runtime.txt
# 另行安装与本机 CUDA/架构匹配的 PyTorch、CuPy；requirements 不会代装它们。
python3 -c 'import cupy, torch, numpy, scipy, cv2; print("torch", torch.__version__, "CUDA", torch.cuda.is_available()); print("CuPy devices", cupy.cuda.runtime.getDeviceCount())'
./build_workspace.sh
source install/local_setup.bash
source setup_runtime_env.sh
ros2 run ht_traversability_ros2 ht_preflight --device cuda
```

`ht_preflight` 必须通过，再进入联合运行。两套工作区包含同名传感器包：主系统由 Autoware 负责驱动，HT 终端只启动建图、模型和融合。**联合运行时不要再执行 ELE 的 `start_real_car.sh`**，它会重复启动驱动和另一套 TF。

## 5. 每个终端共用的环境文件

编译完成后创建一次；新电脑实际 CAN 口、网卡名需按第 6 节确认：

```bash
cat > "$HOME/scout_ws/scout_env.bash" <<'ENV'
export SCOUT_WS="$HOME/scout_ws/autoware.scout"
export SCOUT_HT_ELE_WORKSPACE="$HOME/scout_ws/ele_scouts_kit"
export SCOUT_HT_MAP_ROOT="$HOME/scout_ws/scout_elevation_maps"
export MAP_DIR="$HOME/scout_ws/SCOUT_HANDOVER_20261030/bootstrap_map"
export AUTOWARE_DATA_DIR="$HOME/autoware_data"
export SCOUT_CAN_INTERFACE=can0
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/setup_env.bash"
ENV
```

原车机用 `can10`，新电脑不保证同名，请编辑环境文件后再启动。正式地图准备好后把 `MAP_DIR` 改为该地图目录。

本手册不额外设置 `ROS_DOMAIN_ID` 或强制切换 DDS，沿用新电脑环境。同机所有启动和查看终端必须保持相同 ROS 域、兼容 RMW；仅 source 工作区不会自动统一已经导出的域变量。可比较：

```bash
source "$HOME/scout_ws/scout_env.bash"
printf 'domain=%s localhost=%s rmw=%s\n' "${ROS_DOMAIN_ID:-0}" "${ROS_LOCALHOST_ONLY:-0}" "${RMW_IMPLEMENTATION:-ROS默认}"
```

若要把 RViz 放在另一台电脑，`ROS_LOCALHOST_ONLY=1` 会阻止跨机发现；还需另行配置域、网络和防火墙。本文三个 RViz 在运行程序的同一台电脑打开。

## 6. 硬件网络、CAN 与坐标

### 6.1 传感器网络

Fixposition 当前地址为 `192.168.1.103:21000`；原机传感器网卡曾使用 `192.168.1.102/24`。雷达目的 IP 必须与新机网卡地址相符，RSHELIOS MSOP/DIFOP UDP 端口为 `3344/5566`。

```bash
ip -br addr
# eno1 仅为示例；确认网段没有地址冲突后替换为实际传感器网卡名。
export SCOUT_SENSOR_NIC=eno1
sudo ip addr add 192.168.1.102/24 dev "$SCOUT_SENSOR_NIC"
sudo ip link set "$SCOUT_SENSOR_NIC" up
ping -c 3 192.168.1.103
nc -vz -w 3 192.168.1.103 21000
sudo timeout 5s tcpdump -ni "$SCOUT_SENSOR_NIC" 'udp port 3344 or udp port 5566'
```

地址已经存在时不重复 `ip addr add`。这只是临时配置，重启持久化应通过新机 NetworkManager/netplan 设置；不要覆盖联网用的默认路由。

对应配置：

- `src/sensor_component/scout_sensor_kit_launch/config/fixposition.yaml`
- `src/sensor_component/scout_sensor_kit_launch/config/rslidar.yaml`

修改后需让实际驱动加载新配置；确认构建安装目录已更新。

### 6.2 底盘 CAN

```bash
source "$HOME/scout_ws/scout_env.bash"
ip -br link | grep can
sudo ip link set "$SCOUT_CAN_INTERFACE" down
sudo ip link set "$SCOUT_CAN_INTERFACE" type can bitrate 500000
sudo ip link set "$SCOUT_CAN_INTERFACE" up
ip -details link show "$SCOUT_CAN_INTERFACE"
timeout 5s candump -n 5 "$SCOUT_CAN_INTERFACE"
```

原车曾收到 `0x211/0x221/0x241/0x251` 等反馈帧。没有报文先查设备、驱动、终端电阻和接线。
原机文档中的 `busybox devmem 0x0c303010 ...` 是 Jetson 板级引脚设置，**不能当作通用新机命令执行**；USB-CAN 和其他主板使用对应硬件配置。

### 6.3 定位及 TF 约定

当前使用 Fixposition，不是原 EZ21 CAN INS。文件名 `ins_driver_ez21` 是沿用的定位桥包名。
`map` 按当前 Fix `FP_ENU0` 定义，桥将 POI 位姿补偿至 Autoware `base_link`。Autoware 的 `base_link` 位于后轴中点地面处，x 前、y 左、z 上。

| 从 base_link 看 | 平移 xyz（m） | 旋转 rpy（rad） |
|---|---|---|
| Fixposition POI | `(0.4495158179, 0, 0.65899)` | `(0,0,0)` |
| rslidar | `(0.4215, 0, 0.78499)` | `(0,0,0)` |

车辆或安装位置变化时必须重测外参，并同步定位桥与 URDF；不能照抄。原车两根 GNSS 天线间距约 0.62 m。驱动有话题不代表定位有效：还要查看 `/fixposition/fpa/odomstatus`，确认融合初始化与 GNSS 状态。

Autoware 定位桥发布 `map -> base_link`，车辆模型发布 `base_link -> rslidar/FP_POI`。Fix 原始 `/tf` 和 `/tf_static` 被隔离到 `/fixposition/tf`、`/fixposition/tf_static`，供原点检查使用，避免同一 child 两个父亲。

## 7. 整套启动指令（主程序、UI、高程、HT、三个 RViz）

所有命令在图形桌面的终端执行。首次默认是启动占位图，等待 Fix 就绪后用 UI 生成/加载与实时 ENU0 对齐的图。

### 终端 A：Autoware + 传感器 + 底盘 + UI

```bash
source "$HOME/scout_ws/scout_env.bash"
cd "$SCOUT_WS"
# 确认地图文件齐全。
for f in lanelet2_map.osm pointcloud_map.pcd pointcloud_map_metadata.yaml map_projector_info.yaml; do
  test -f "$MAP_DIR/$f" || { echo "缺地图文件: $MAP_DIR/$f"; break; }
done
SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1 ./start_scout.bash \
  map_path:="$MAP_DIR" data_path:="$AUTOWARE_DATA_DIR" \
  start_sensor_drivers:=true \
  launch_vehicle_interface:=true \
  vehicle_interface_can_interface:="$SCOUT_CAN_INTERFACE" \
  publish_lane_driving_costmap:=true \
  launch_ui:=true rviz:=false
```

这里开放底盘与 UI 操作。启动驱动本身不等于已进入自动驾驶，进入自动仍由 UI/Autoware 模式流程完成。
网页地址：**http://127.0.0.1:8090/index.html**，浏览器手动打开。

`data_path` 中的运行资源不随源码包交付；若所用感知模块报告模型缺失，需要按该模块配置下载匹配资源。当前 Scout 默认用雷达聚类，ELE HT 模型在其源码包内。

### 终端 B：全局高程 + HT 八方向 + 融合

```bash
source "$HOME/scout_ws/scout_env.bash"
cd "$SCOUT_WS"
./start_scout_ht.bash start_elevation:=true start_ht:=true rviz:=false
```

终端 B 不重复启动雷达、Fix 或底盘。Fix 未初始化时，高程管理器等待，不能靠伪造 map TF 绕过等待。

**是否参与规划：**上面的主程序默认 `scout_ht_enforce:=false`，HT 只生成可视化/诊断。要测试 HT 对路径和速度的实际约束，在终端 A 命令末尾增加 `scout_ht_enforce:=true`，仍需运行终端 B；此配置方向、阈值和制动仍未完成整车标定。不要将“融合图在发布”等同于“规划已使用融合图”。

### 终端 C：同时打开三个 RViz

```bash
source "$HOME/scout_ws/scout_env.bash"
source "$SCOUT_HT_ELE_WORKSPACE/install/local_setup.bash"
ros2 run rviz2 rviz2 -d "$SCOUT_WS/config/scout_costmap_raw.rviz" \
  --ros-args -r __node:=scout_raw_costmap_rviz &
RAW_RVIZ_PID=$!
ros2 run rviz2 rviz2 -d "$SCOUT_WS/config/scout_costmap_fused.rviz" \
  --ros-args -r __node:=scout_fused_costmap_rviz &
FUSED_RVIZ_PID=$!
ros2 run rviz2 rviz2 -d "$SCOUT_WS/config/scout_elevation_view.rviz" \
  --ros-args -r __node:=scout_elevation_rviz &
ELEVATION_RVIZ_PID=$!
wait
```

三个窗口分别是原始 costmap、HT 融合 costmap、全局高程。主程序设 `rviz:=false`，因此总共是这三个窗口。需要 Autoware 路线/车辆界面时可额外将主程序 `rviz:=true`，会多开一个窗口。

仅关这三个窗口：直接逐个关闭窗口；或在同一终端 C 中中断 `wait` 后执行：

```bash
kill -INT "$RAW_RVIZ_PID" "$FUSED_RVIZ_PID" "$ELEVATION_RVIZ_PID"
```

## 8. 只开 RSHELIOS、Fixposition 和 TF

本模式替代第 7 节，不与整套程序同时启动。不启动底盘、costmap 或规划；检查原始点云时 RViz Fixed Frame 可先设为 `rslidar`。

四个终端分别先 `source "$HOME/scout_ws/scout_env.bash"`，再执行对应命令：

终端 1（两种传感器驱动）：

```bash
ros2 launch scout_sensor_kit_launch sensing.launch.xml start_drivers:=true
```

终端 2（完整车辆/传感器静态模型 TF，用 YAML 传 URDF，避免 shell 字符串截断）：

```bash
python3 - <<'PY'
from pathlib import Path
import subprocess, yaml
from ament_index_python.packages import get_package_share_directory
urdf = Path(get_package_share_directory('scout_vehicle_description')) / 'urdf/vehicle.xacro'
robot = subprocess.check_output(['xacro', str(urdf)], text=True)
p = Path.home() / 'scout_ws/robot_description.yaml'
p.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': robot}}}))
print(p)
PY
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "$HOME/scout_ws/robot_description.yaml"
```

终端 3（定位桥，固定外参适用于当前原车）：

```bash
ros2 run ins_driver_ez21 ins_localization_bridge --ros-args \
  -p input_odom_topic:=/fixposition/odometry_enu \
  -p output_odom_topic:=/localization/kinematic_state \
  -p output_frame_id:=map -p output_child_frame_id:=base_link \
  -p publish_tf:=true \
  -p base_link_to_ins_x:=0.4495158179045505 \
  -p base_link_to_ins_y:=0.0 -p base_link_to_ins_z:=0.65899 \
  -p base_link_to_ins_roll:=0.0 -p base_link_to_ins_pitch:=0.0 \
  -p base_link_to_ins_yaw:=0.0
```

终端 4（显式表示本模式 map 与实时 FP_ENU0 同系）：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id FP_ENU0
```

第 8 节不会产生 `/sensing/lidar/concatenated/pointcloud`；那是完整 Autoware 预处理的输出。因此不要在此模式期待第 7 节的 costmap 和高程链路自动工作。传感器状态未初始化时，即便 TF 看似存在也不能把零定位当成有效位姿。

## 9. 启动后的验证指令

在新终端：

```bash
source "$HOME/scout_ws/scout_env.bash"
source "$SCOUT_HT_ELE_WORKSPACE/install/local_setup.bash"
ros2 topic list
ros2 topic hz /rslidar_points
# 上条持续输出；Ctrl+C 后再执行下一条。
ros2 topic echo --once /fixposition/fpa/odomstatus
ros2 topic echo --once /localization/kinematic_state
ros2 run tf2_ros tf2_echo map rslidar
```

TF 检查也持续运行，用 Ctrl+C 退出。完整模式继续检查：

```bash
ros2 topic echo --once /planning/scenario_planning/parking/costmap_generator/occupancy_grid --field info
ros2 topic echo --once /scout/elevation/status
ros2 topic echo --once /scout/traversability/status
ros2 topic echo --once /scout/traversability/fused_costmap --field info
ros2 topic info -v /elevation_mapping_node/elevation_map_global
curl -fsS http://127.0.0.1:8090/health
curl -fsS http://127.0.0.1:8090/autoware_status
```

正常几何与含义：

| 话题 | 内容 |
|---|---|
| `/rslidar_points` | 原始雷达，frame=rslidar，原车约 10 Hz |
| `/sensing/lidar/concatenated/pointcloud` | 完整启动的自车裁切后点云，frame=base_link |
| `/sensing/imu/imu_data` | Fixposition POI IMU |
| `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` | 原始 costmap，0.04 m，70×70 m，1750×1750 格；车道也发布 |
| `/elevation_mapping_node/elevation_map_global` | 随车 15×15 m 的全局高程查看窗口，不是全局库的范围上限 |
| `/elevation_mapping_node/elevation_map_global_local` | 从全局图裁剪的 10×10 m HT 输入 |
| `/scout/ht/raw` | HT 八方向原始结果 |
| `/scout/traversability/fused_costmap` | 二值融合图，0 白、100 黑，几何跟随原始 costmap |

融合以原 costmap 为第一关，原始障碍/未知不由 HT 清除；原始允许区按“车到该格”的角度选择 HT 通道。
未观测/无有效 HT 结论/HT 覆盖外按用户指定的可通行先验处理；已有有效 HT 低评分仍黑。输入过期、非法或 TF 故障另外报告并使输出失效，不能把全黑一概解释为真实障碍。

原因图 `/scout_costmap_ht_fusion/reason_map`：0 实测通过，1 原始禁止，2 原始未知，3 HT 无效，4 HT 低分，5 过近，6 过期，7 TF 错误，8 输入错误，9 等待，10 先验放行。
高程先验为 **-1 m、方差 10000 m²、观测次数 0**；不代表实测地面就是 -1 m。实际扫描后由多帧后验更新。

## 10. 地图更换与历史高程迁移

Fix 重启后 `FP_ENU0` 原点可能变化。UI 生成地图时应使用当前有效 Fix 原点，不能沿用 EZ21 的 INS 原点；旧地图有文件并不等于与本次定位对齐。

运行中换地图通过 UI“加载地图”或生成后切换完成：车辆静止、STOP 模式、Autoware 控制关闭；清旧路线并更新投影/地图，再重新下发路线。正常换图无需重启整套系统。地图生成的默认转弯半径为 1.25 m，默认速度 3.6 km/h，起点车道自动延长到车后 5 m；已有地图不会自动重写。

历史高程不在源码 ZIP 中。需要续建时，在原机正常停止高程进程后，另行拷贝整个高程根目录：

```text
原机 /mnt/nvme0n1/autoware/scout_elevation_maps/
新机 ~/scout_ws/scout_elevation_maps/
```

每个会话保留 `session.json`、`mapping.params.yaml`、`tiles.sqlite3`。正常退出并刷盘后再复制；不要运行时只复制 sqlite 主文件而遗漏 WAL。会话校验实际 ECEF/ENU0 原点、外参和算法配置：匹配才能恢复，不同则新建，不能强行合并。没有迁移历史时从先验重新积累是正常行为。

## 11. 常见故障

| 现象 | 优先检查 |
|---|---|
| `topic list` 只有 rosout/parameter_events | 先 source 同一环境，比较 ROS 域/RMW，确认进程还在；必要时在相同环境 `ros2 daemon stop` 后重查 |
| RViz 点云空白 | 原始模式选 `/rslidar_points`；完整模式可选裁切话题；查 map→rslidar、QoS、相机位置。仅开雷达没有车辆 TF 时用 Fixed Frame=rslidar |
| 高程没数据 | Fix 初始化、GNSS、裁切点云、时间和 ray_origin_frame=rslidar TF；看 `/scout/elevation/status` |
| 融合图无尺寸 | 还没有收到有效原始 costmap；查地图、点云、TF 和生成器组件 |
| 融合图全黑 | 看 status 和 reason_map，分清原始障碍、HT 低分、STALE、TF 错误，别先改阈值 |
| 高程重影或旧人影 | 查单一发布者、原点/外参、时间戳；已实现射线证据 v2，只有真实新射线或新落点支持才能消除，未重新观测不会凭时间清空 |
| 8090 拒绝连接 | `ss -ltnp '( sport = :8090 )'`；查 launch_ui 与启动终端错误 |
| UI 操作 disabled | 查旧服务是否占端口，确认当前进程继承 `SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1` |
| 路线 empty / 原点不对齐 | 地图是否为当前场地、是否对齐实时 ENU0、车辆和目标是否在连通车道上 |
| 不能 Auto | 看 operation mode、CAN 实际反馈、地图路线和诊断；开放网页权限不代表底盘已接受自动模式 |
| 雷达重名/重复发布者 | 不同时开 ELE 全套和 Autoware；SDK 只重命名 param_handle，不能全局重命名其所有内部节点 |
| 缺 spconv / acados / TensorRT | 安装对应平台开发库；不要依赖原机 .deps 或 EZ21 install |
| GridMap 插件加载失败 | 安装并 source `grid_map_rviz_plugin` 所在环境，确认与 Humble 对应 |

启动耗时包括大量 ROS launch 展开、组件创建、GPU 初始化和地图读取；原机一次启动曾在约 94 秒后才批量拉起节点，不能把这个时间当成新机固定值。首次编译、首次 GPU 内核编译也会更慢。保留启动终端，先看错误和资源使用，不要因为等待就重复开第二套驱动。

## 12. 正常停止与交接验收

先在 UI 停止自动控制、保持车辆停止，再在终端 B 按 Ctrl+C 等待高程落盘；终端 A 按 Ctrl+C 停止主程序，最后关闭 RViz。独立传感器模式则逐一 Ctrl+C 停止第 8 节终端。不要用 `pkill -9 python` 之类广泛杀进程的方法。

交接验收至少记录：完整编译结果、GPU preflight、传感器有频率、Fix 有效初始化、TF 连通、原始与融合图持续更新、地图与当前原点对齐。涉及执行 HT 规划时，另行验收方向通道、阈值、延迟与制动能力。

更多实现细节见 Autoware 根目录 `SCOUT_STARTUP.md`、`SCOUT_HT_STARTUP.md`、`SCOUT_GLOBAL_ELEVATION.md`、`SCOUT_HT_PLANNING.md`。这些较早文档含原机绝对路径和历史测试结果，本手册的目录约定用于新机部署。
