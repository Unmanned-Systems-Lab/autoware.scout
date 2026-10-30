# Autoware Scout 启动速查（2026-10-30 源码交接）

主入口是 `./start_scout.bash`；全局高程和 HT 是另一个终端里的 `./start_scout_ht.bash`。主程序默认启动真实传感器和底盘接口，但启动完成不等于进入自动驾驶。

本文依据本次交接源码与[完整交接手册](docs/handover/20261030/SCOUT_HANDOVER.md)整理。它是启动与调用关系说明，不是新机编译或实车验收报告；此次 Windows 端整理未运行 ROS、CUDA 或车辆测试。原手册也说明完整迁移路线尚未在第二台干净电脑上验收。

## 1. 仓库和目录

两套工作区分别维护、分别构建，交接分支均为 `handover/20261030-source`：

- [Unmanned-Systems-Lab/autoware.scout](https://github.com/Unmanned-Systems-Lab/autoware.scout)：主程序、Scout 模型与底盘接口、Fix 定位桥、UI、原始 costmap、HT 融合和可选规划约束。
- [CQULzz/eletranskit_scout](https://github.com/CQULzz/eletranskit_scout)：CUDA 高程建图、全局积累与射线清除、HT 八方向推理及模型。

取得交接分支（本机已配置 GitHub SSH 访问时）：

```bash
mkdir -p "$HOME/scout_ws"
cd "$HOME/scout_ws"
git clone --branch handover/20261030-source git@github.com:Unmanned-Systems-Lab/autoware.scout.git
git clone --branch handover/20261030-source git@github.com:CQULzz/eletranskit_scout.git ele_scouts_kit
```

ELE 的 GitHub 仓库已重命名为 `eletranskit_scout`，本地仍使用 `ele_scouts_kit` 目录名以匹配环境配置。

本文使用以下 Linux 目录：

```text
~/scout_ws/
  autoware.scout/
  ele_scouts_kit/
  scout_env.bash
  scout_elevation_maps/      # 运行时生成，不随源码交接
```

本次 ELE 源码已包含本次修改，不要再应用 Autoware 中留存的 ELE 补丁。不要对已经完整提供的 Autoware `src/` 再执行 `vcs import`，以免覆盖 Scout 修改。

## 2. 编译前提和首次构建

目标运行环境是 Ubuntu 22.04、ROS 2 Humble，以及与机器架构匹配的 NVIDIA GPU/CUDA 开发环境。原机为 Jetson AGX Orin；x86_64 机器不能直接复制 Jetson 的 wheel、动态库和 CAN 引脚配置。

源码不包含 `build/`、`install/`、`.deps/`、历史高程库和所有感知运行资源。完整构建还可能需要平台匹配的 TensorRT、acados、spconv/cumm 等开发依赖；`build_scout.bash` 不是依赖安装器。不要复制原机串接 EZ21 underlay 的 `install/` 作为独立安装。依赖版本、网络和 CAN 准备详见[完整手册第 2、4、6 节](docs/handover/20261030/SCOUT_HANDOVER.md)。

Autoware：在没有 source 其他工作区的新终端执行。

```bash
cd "$HOME/scout_ws/autoware.scout"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# rosdep 有失败项时先处理依赖，再继续。
unset SCOUT_UNDERLAY
./build_scout.bash
source ./setup_env.bash
ros2 pkg prefix autoware_launch
ros2 pkg prefix scout_ht_traversability
ros2 pkg prefix scout_ht_planning
```

ELE：另开新终端，不要先 source Autoware。

```bash
cd "$HOME/scout_ws/ele_scouts_kit"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "ament_python cupy-cuda12x simple-parsing python3-torch python3-torchvision"
python3 -m pip install --user -r requirements-runtime.txt
# 另行安装与本机架构、CUDA 匹配的 PyTorch 和 CuPy。
python3 -c 'import cupy, torch, numpy, scipy, cv2; print("torch", torch.__version__, "CUDA", torch.cuda.is_available()); print("CuPy devices", cupy.cuda.runtime.getDeviceCount())'
./build_workspace.sh
source install/local_setup.bash
source setup_runtime_env.sh
ros2 run ht_traversability_ros2 ht_preflight --device cuda
```

确认构建成功、GPU preflight 通过后再启动。首次构建应使用完整构建流程，不套用依赖旧 EZ21 安装的增量构建命令。

## 3. 创建共用环境文件

构建完成后创建一次：

```bash
cat > "$HOME/scout_ws/scout_env.bash" <<'ENV'
export SCOUT_WS="$HOME/scout_ws/autoware.scout"
export SCOUT_HT_ELE_WORKSPACE="$HOME/scout_ws/ele_scouts_kit"
export SCOUT_HT_MAP_ROOT="$HOME/scout_ws/scout_elevation_maps"
export MAP_DIR="$SCOUT_WS/docs/handover/20261030/bootstrap_map"
export AUTOWARE_DATA_DIR="$HOME/autoware_data"
export SCOUT_CAN_INTERFACE=can0
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/setup_env.bash"
ENV
```

将 `SCOUT_CAN_INTERFACE` 改为实际 CAN 口；原车使用 `can10`，新机不保证同名。CAN 速率为 500000，先确认接口已启用且能收到反馈。传感器网络需匹配 Fixposition `192.168.1.103:21000` 和 RSHELIOS UDP `3344/5566` 的配置。

`bootstrap_map` 只用于启动加载链路，不是新场地已对齐地图。正式使用时将 `MAP_DIR` 改为与当前有效 Fix `FP_ENU0` 原点对齐的地图目录，其中应有 `lanelet2_map.osm`、`pointcloud_map.pcd`、`pointcloud_map_metadata.yaml` 和 `map_projector_info.yaml`。不要只改原点 YAML 冒充整张地图已转换坐标。

所有终端应使用相同 ROS 域和兼容的 RMW；source 工作区不会覆盖已有的 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 或 `RMW_IMPLEMENTATION` 设置。

## 4. 两个终端启动

### 终端 A：Autoware、传感器、底盘和 UI

```bash
source "$HOME/scout_ws/scout_env.bash"
cd "$SCOUT_WS"
SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1 ./start_scout.bash \
  map_path:="$MAP_DIR" data_path:="$AUTOWARE_DATA_DIR" \
  start_sensor_drivers:=true \
  launch_vehicle_interface:=true \
  vehicle_interface_can_interface:="$SCOUT_CAN_INTERFACE" \
  publish_lane_driving_costmap:=true \
  launch_ui:=true rviz:=false
```

手动打开 <http://127.0.0.1:8090/index.html>。该命令连接真实底盘并开放 UI 车辆操作；进入自动驾驶仍由 UI/Autoware 模式流程完成。地图、定位和现场运动能力需要独立验收，UI 操作可用不代表验收完成。只检查系统而不连接底盘时，将 `launch_vehicle_interface:=true` 改为 `false`。

### 终端 B：全局高程、HT 八方向和融合

```bash
source "$HOME/scout_ws/scout_env.bash"
cd "$SCOUT_WS"
./start_scout_ht.bash start_elevation:=true start_ht:=true rviz:=false
```

终端 B 复用 A 的点云、定位和 TF，不重复启动传感器或底盘。高程管理器等待有效 Fix 初始化和 ENU0 原点；不能用伪造 TF 绕过等待。联合运行时不要再启动 ELE 的 `start_real_car.sh`。

两个命令都设 `rviz:=false`，因此不会自行打开 RViz。三个独立查看窗口的命令见[完整手册第 7 节](docs/handover/20261030/SCOUT_HANDOVER.md)；需要主 Autoware RViz 时可将 A 的参数改为 `rviz:=true`。

## 5. 真实启动调用链

```text
start_scout.bash
 ├─ setup_env.bash → 本工作区 install/setup.bash、CUDA/本地库环境
 └─ ros2 launch autoware_launch real_scout.launch.xml
     ├─ autoware.launch.xml
     │   ├─ 全局参数、点云组件容器、UI
     │   ├─ vehicle.launch.xml
     │   │   ├─ robot_state_publisher → 车辆/传感器静态 TF
     │   │   └─ Scout vehicle_interface.launch.xml
     │   │       ├─ scout_base_node → SocketCAN
     │   │       └─ scout_autoware_interface → 控制/反馈转换
     │   ├─ system、map
     │   ├─ ins_localization_component → ins_localization_bridge
     │   ├─ planning、control、API
     │   └─ RViz（按参数）
     ├─ Scout sensing.launch.xml → Fixposition、RSHELIOS 驱动
     ├─ robosense_m1_pointcloud_preprocessor.launch.py → 自车裁切
     └─ tier4_perception_component.launch.xml → CUDA 地面分割、雷达聚类

start_scout_ht.bash
 ├─ Autoware 环境 + 指定 ELE 工作区环境
 └─ scout_ht_traversability/observe.launch.py
     ├─ global_elevation_mapper → 等定位有效后启动 elevation_mapping_cupy
     ├─ ht_traversability_node → HT 八方向推理
     └─ costmap_ht_fusion → 原始 costmap 与 HT 融合
```

这是调用关系；节点启动后并行运行，不是按图顺序逐项完成。主 launch 关闭通用 sensing/perception 入口，再添加 Scout 专用入口，避免重复启动。

核心数据链路为：

```text
RSHELIOS → /rslidar_points → 自车裁切 → /sensing/lidar/concatenated/pointcloud
                                    ├─ 感知/规划 → 原始 costmap ────┐
                                    └─ 高程 → 局部裁剪 → HT 八方向 ┤
                                                                 ↓
                                      /scout/traversability/fused_costmap

Fixposition /fixposition/odometry_enu → 定位桥 → /localization/kinematic_state
                                             + map → base_link TF

规划轨迹 → 控制器 → /control/command/control_cmd
         → scout_autoware_interface → /scout/cmd_vel → scout_base_node → CAN
```

实现入口可直接查看：[主脚本](start_scout.bash)、[主 launch](src/launcher/autoware_launch/autoware_launch/launch/real_scout.launch.xml)、[HT 脚本](start_scout_ht.bash)、[HT launch](src/perception/scout_ht_traversability/launch/observe.launch.py)。

## 6. HT 是否参与规划

默认 `scout_ht_enforce:=false`，终端 B 只提供感知、可视化和诊断。融合图有数据并不表示规划正在使用它。

要启用 HT 路径/速度约束，在终端 A 命令末尾增加 `scout_ht_enforce:=true`，同时保留终端 B。该开关切换到 `scout_ht_terrain` preset，启用 `path_sampler` 和 HT 规划模块，并关闭 parking 与 freespace bypass，防止绕过车道 HT 约束。方向通道、阈值、延迟、制动和整车运动能力仍需实际标定与验收。

另外，单独执行不带参数的 `./start_scout_ht.bash` 默认只起融合器：`start_elevation` 和 `start_ht` 都默认是 `false`，因此完整链路要保留上面的两个 `true`。

## 7. 启动后检查与停止

新终端执行：

```bash
source "$HOME/scout_ws/scout_env.bash"
source "$SCOUT_HT_ELE_WORKSPACE/install/local_setup.bash"
ros2 topic echo --once /fixposition/fpa/odomstatus
ros2 topic echo --once /localization/kinematic_state
ros2 topic echo --once /planning/scenario_planning/parking/costmap_generator/occupancy_grid --field info
ros2 topic echo --once /scout/elevation/status
ros2 topic echo --once /scout/traversability/status
ros2 topic echo --once /scout/traversability/fused_costmap --field info
curl -fsS http://127.0.0.1:8090/health
curl -fsS http://127.0.0.1:8090/autoware_status
```

原始 costmap 默认是 0.04 m、70×70 m、1750×1750 格，话题路径虽然含 `parking`，车道行驶也会发布。融合图几何跟随原始图；全黑时查看状态和原因图，区分真实障碍、HT 低分、过期和 TF/输入故障。

定位桥包名仍叫 `ins_driver_ez21`，但当前输入是 Fixposition，旧 CAN INS 驱动已关闭；`robosense_m1_*` 也是复用命名，实际雷达是 RSHELIOS。不要根据旧文件名再额外启动另一套驱动。

停止时先退出自动控制、保持车辆停止，再在 B 按 Ctrl+C 等待高程落盘，然后在 A 按 Ctrl+C，最后关闭 RViz。Fix 重启后 ENU0 原点可能变化，复用地图和历史高程前要重新核对原点。
