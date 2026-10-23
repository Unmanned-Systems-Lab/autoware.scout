# Autoware Scout 启动说明

以下命令在 Scout 车机本机运行。当前示例地图是 UI 根据两个路径点生成的测试地图，**尚未验证真实道路和 Fixposition FP_ENU0 的现场对齐情况**。下面会启动底盘接口，并开放 UI 的地图生成、路线下发和自动驾驶操作；启动前确认急停、人工接管、车轮周围及 SocketCAN 配置。若遥控器仍无法接管车辆，先不要使用 `launch_vehicle_interface:=true`。开放操作权限不代表地图和定位已经达到实车行驶条件。

## 原生 CAN 接口准备

这台 Jetson AGX Orin 将原生 CAN0 命名为 `can10`、CAN1 命名为 `can11`。现场 Scout V2 底盘反馈帧出现在 `can10`；`can11` 没有收到报文。当前系统的引脚默认是 GPIO 模式，重启后需先将原生 CAN0 引脚切到 CAN 模式，再以 500 kbps 启用接口：

```bash
sudo ip link set can10 down
sudo busybox devmem 0x0c303010 32 0x0000C400  # CAN0 TX
sudo busybox devmem 0x0c303018 32 0x0000C458  # CAN0 RX
sudo ip link set can10 up type can bitrate 500000
timeout 5s candump -n 5 can10
```

`candump` 应显示 `0x211`、`0x221`、`0x241`、`0x251` 等底盘反馈帧。若无报文，先检查底盘供电、CAN 收发器和接线；遥控器能驱动车辆不代表 Jetson 到底盘的 CAN 通路正常。以上 `devmem` 修改只在本次开机有效。

## 一体化启动

先确认 `8090` 端口没有被独立 `ez21_ui` 或其他网页服务占用：

```bash
ss -ltnp '( sport = :8090 )'
```

然后在一个终端运行：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
MAP_DIR=/mnt/nvme0n1/autoware/autoware.scout/src/launcher/autoware_launch/ui/generated_maps/scout_fp_enu0_1792142980046

SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1 ./start_scout.bash map_path:="$MAP_DIR" \
  start_sensor_drivers:=true \
  launch_vehicle_interface:=true \
  vehicle_interface_can_interface:=can10 \
  launch_ui:=true \
  rviz:=true
```

`MAP_DIR` 是启动占位图，不保证与本次 Fixposition 初始化后的 `FP_ENU0` 原点相同。原点不一致时，UI 会阻止路线和自动控制；等 Fixposition 就绪后，在网页的“Autoware 状态”面板选择兼容地图并点“加载地图”，或用路径点生成新图并在线切换。

`start_scout.bash` 会自行加载该工作区的 `setup_env.bash`。上面的命令不额外指定 `ROS_DOMAIN_ID` 或 DDS 实现，沿用终端环境；在其他终端查看 ROS 话题时应使用相同的 ROS 环境。

当前默认使用 CUDA 地面分割。本机已补编 `autoware_cuda_utils`、`cuda_blackboard`、`autoware_cuda_pointcloud_preprocessor`、`autoware_probabilistic_occupancy_grid_map` 和 `autoware_ground_segmentation_cuda`；如果在新的工作区重装，需先编好这些包，否则感知组件会因包或插件缺失而无法加载。

Scout 点云预处理会在 `base_link` 坐标系内裁掉车身包络及少量余量：前后左右各外扩 0.10 米、上下各外扩 0.05 米，用于去除车身边缘回波。当前裁切范围为 `x=[-0.3135, 0.8115]`、`y=[-0.449795, 0.449795]`、`z=[-0.05, 1.00]` 米；范围外的点继续参与感知和避障。可通过启动参数 `pointcloud_self_crop_horizontal_margin_m` 和 `pointcloud_self_crop_vertical_margin_m` 调整，设为 `0.0` 可恢复原始车身包络。不要用扩大裁切范围来消除尚未确认来源的障碍点；裁切范围内的真实物体也会被删除。原始 `/rslidar_points` 保留不变，裁切后的话题是 `/sensing/lidar/concatenated/pointcloud`。

裁切参数在 CUDA 组件初始化时读取，后续调整需重新启动点云容器才能生效，仅执行 `ros2 param set` 不会更新 GPU 裁切范围。本机曾在单独卸载 CUDA 预处理组件时触发 ROS wait-set 异常，因此不要以单独卸载该组件的方式更新参数。

雷达 SDK 在一个进程内包含多个 ROS 节点。Scout 启动入口仅将参数节点 `param_handle` 重命名为 `/rslidar_sdk`，点云发布节点保持 `/rslidar_points_destination_0`；这两个不同名称属于同一驱动。不要给整个驱动进程设置全局 `__node:=rslidar_sdk`，否则会把内部节点全部改成同名，触发重复节点诊断。

Scout UI 地址：**http://127.0.0.1:8090/index.html**。集成启动默认不会自动打开浏览器，需要手动访问。`launch_vehicle_interface:=true` 会连接真实底盘；它本身不等于进入自动驾驶，但其他已运行的进程仍可能发送控制命令。`SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1` 开放 UI 的地图生成、路线下发和进入自动等操作；这些操作不会自动核验地图与现场是否对齐。

若仅需测试 UI 的地图生成和路线下发、但不需要连接底盘，可停止上述启动（在启动终端按 `Ctrl+C`），将 `launch_vehicle_interface:=true` 改为 `launch_vehicle_interface:=false` 后重新运行。要关闭 UI 操作权限，则把 `SCOUT_UI_ALLOW_VEHICLE_ACTIONS=1` 改为 `SCOUT_UI_ALLOW_VEHICLE_ACTIONS=0`。在现场地图、定位原点和行驶边界未经核验前，不要用这张测试地图开启自动驾驶。

## 启动后检查

```bash
curl -fsS http://127.0.0.1:8090/health
curl -fsS http://127.0.0.1:8090/autoware_status | python3 -c 'import json,sys; print(json.load(sys.stdin)["ui_actions_enabled"])'
```

第二条应输出 `True`。如果输出 `False`，检查并停止占用 `8090` 的旧 UI 进程，再重新启动 Scout；仅重启启动脚本可能会复用旧服务。

在第二个终端查看话题和定位数据：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
ros2 topic list
ros2 topic echo --once /localization/kinematic_state
ros2 topic echo --once /scout/status
ros2 topic echo --once /scout/wheel_odometry
```

若 `8090` 已有健康的其他 UI 服务，Scout 的 UI 启动脚本可能复用该服务，而不会启动本仓库的页面。此时先确认端口对应的进程，再处理端口冲突。若 RViz 没有图形桌面可用，可把命令中的 `rviz:=true` 改为 `rviz:=false`；网页地址不变。

地图目录必须包含 `lanelet2_map.osm`、`pointcloud_map.pcd`、`pointcloud_map_metadata.yaml` 和 `map_projector_info.yaml`。每次传感器重启后，都需要核对实际 Fixposition FP_ENU0 原点与所用地图原点；路线能下发不代表真实场地已经验证可安全行驶。

## ZED 2i 官方相机与多传感器录包

以后只录数据集时，直接按 [SCOUT_DATASET.md](SCOUT_DATASET.md) 使用独立传感器启动和一键录包命令；无需启动本页的 Autoware。下面保留 Autoware 已在运行时单独启动相机和手工录包的说明。

本机已安装与 Jetson L4T 36.5 匹配的 ZED SDK 5.5.1，安装目录为 /mnt/nvme0n1/autoware/zed_sdk_5.5.1；官方 ROS 2 wrapper 单独构建在 /mnt/nvme0n1/autoware/zed_ros2_ws。相机配置为 [zed2i_640x360_15hz.yaml](config/zed2i_640x360_15hz.yaml)：HD720 原生采集、官方节点将彩色图和深度图缩小 2 倍发布为 640×360，目标 15 Hz。深度图由 ZED SDK 根据双目图像生成，无需另起深度模型节点；当前选用 SDK 内置的 NEURAL_LIGHT 模式，其内部模型已完成首次下载和优化。ZED 的位姿跟踪及动态 TF 已关闭，避免与 Autoware 定位产生冲突。

在 Autoware 运行的同时，另开终端启动官方相机节点：

~~~bash
source /opt/ros/humble/setup.bash
source /mnt/nvme0n1/autoware/zed_ros2_ws/install/local_setup.bash
export ZED_DIR=/mnt/nvme0n1/autoware/zed_sdk_5.5.1
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i camera_name:=zed2i \
  ros_params_override_path:=/mnt/nvme0n1/autoware/autoware.scout/config/zed2i_640x360_15hz.yaml \
  publish_tf:=false publish_map_tf:=false
~~~

已验证的彩色图话题是 /zed2i/zed_node/rgb/color/rect/image（640×360、bgra8），深度图话题是 /zed2i/zed_node/depth/depth_registered（640×360、32FC1，单位米），两路实测约 15 Hz。/zed2i/zed_node/rgb/color/rect/camera_info 含有效相机内参；/zed2i/zed_node/imu/data 是 ZED 自带 IMU。深度图中无效像素可为 NaN，应按 32 位浮点距离数据处理。底盘电池电压、各电机电流、RPM 和脉冲计数在 /scout/status；底盘轮速里程计在 /scout/wheel_odometry。Fixposition 的 /fixposition/fpa/odomstatus 可判断 RTK 状态。

要把彩色图、深度图、雷达、RTK、IMU 和底盘数据一起录到 NVMe 的一个 MCAP，在另一个终端运行：

~~~bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
mkdir -p /mnt/nvme0n1/autoware/bags
ros2 bag record -s mcap --max-bag-size 4294967296 \
  -o "/mnt/nvme0n1/autoware/bags/scout_$(date +%Y%m%d_%H%M%S)" \
  /zed2i/zed_node/rgb/color/rect/image \
  /zed2i/zed_node/depth/depth_registered \
  /zed2i/zed_node/rgb/color/rect/camera_info \
  /zed2i/zed_node/depth/camera_info \
  /zed2i/zed_node/imu/data \
  /rslidar_points /sensing/lidar/concatenated/pointcloud \
  /fixposition/odometry_llh /fixposition/odometry_enu \
  /fixposition/fpa/odomstatus /sensing/imu/imu_data \
  /scout/status /scout/wheel_odometry
~~~

按 Ctrl+C 结束录制，再运行 ros2 bag info <录包目录> 检查各话题消息数。2026-10-21 用当前配置录制的 18 秒样本中，彩色图和深度图各 267 帧（按首末消息间隔计算均为 14.82 Hz），ZED IMU、雷达、RTK、底盘状态和轮速里程计均有数据；包大小约 795 MiB，应持续关注 NVMe 空间。ZED 安装器曾将接收缓冲上限改小，导致 Cyclone DDS 无法新建节点；本机已通过 /etc/sysctl.d/99-scout-cyclone.conf 恢复原来的接收缓冲上限。

## 运行中换图

新地图的默认最小转弯半径为 **1.25 米**，不再使用 EZ21 的 5.5 米默认值。计算取 `max(轴距 / tan(等效最大转角), 默认速度 / 最大角速度)`：Scout 轴距 `0.498 m`、等效最大转角 `0.7 rad` 给出约 `0.59 m`，当前底盘接口最高速度 `1.0 m/s`、最大角速度 `0.8 rad/s` 给出 `1.25 m`。新地图默认限速同步为 `3.6 km/h`，弯道圆弧采样间隔为 `0.2 m`；若转弯前后路径段仍不足以容纳圆弧，生成仍会报错。这是基于当前配置的运动学基准，未包含 Scout 滑移和实际跟踪误差的标定。已有地图的半径和限速不会自动改变；这些默认值也不会重新整定 MPC/PID 参数。

首次安装在线地图投影更新后，**需要重启一次** Autoware 和 UI，使运行中的地图投影节点加载新版程序。以后通过 Scout UI 生成新图，或在“Autoware 状态”面板选择已有地图并点击“加载地图”，无需每次重启整套程序。候选地图必须与**当前实时** Fixposition `FP_ENU0` 原点对齐；不是要求它与旧启动图同原点。两者若不一致，地图下拉项不可选，路线和自动控制也会被拒绝。

换图只允许在车辆静止、运行模式为 STOP、Autoware 控制关闭时进行；会清除旧路线并刷新 Lanelet2、地图投影和点云地图。完成后重新核对 RViz 中车辆、车道、地图与真实场地的位置，再重新下发路线。若加载失败，UI 会锁住路线和自动控制，直到成功加载地图或重新启动。**坐标对齐只是必要条件，不代表地图道路、障碍物或行驶边界已经过实车验证。**

这套启动包含大量 ROS 节点、感知与规划组件以及 RViz，不是只加载一张地图。2026-10-20 的一次启动日志中，ROS launch 开始后约 94 秒才开始批量启动节点，随后地图加载又用了约 5 秒；主要等待发生在整套 launch 初始化，不是地图文件读取。只需网页地图操作时，不必反复重启整套程序；不需要 RViz 时可将 `rviz:=false`，减少图形负载。

Autoware 的路线起点始终取车辆当前定位；UI 中绘制的首点用于定义地图首段，不再作为途经点下发。RViz 中规划轨迹可能保留车后约 5 米供规划器计算，红色 `obstacle_stop` 横线是停止墙，不是路线起点。在线生成 Scout 地图时，若车辆定位有效、与首段车道对齐且首点不超过车前 2 米，程序会沿首段方向把车道起点自动延至车辆后方 5 米；不再要求人工把首点画到车后 5 米。该自动延长段仍须现场核实可行驶；即使长度与规划器可能保留的车后轨迹相当，也不能据此判断轨迹完全在车道内。**已有地图不会自动修改**，更新 UI 进程后需重新绘制并在线生成/切换地图，再下发路线；不必为换图重启整个 Autoware。实车执行前仍需核验新车道边界和障碍物。
