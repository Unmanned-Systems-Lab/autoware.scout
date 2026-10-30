# Scout 独立数据集采集

这套入口只启动 Scout 底盘反馈、Robosense RSHELIOS、Fixposition RTK/IMU、ZED 2i 官方驱动，以及车体静态坐标关系。**不启动 Autoware、地图、规划、控制、UI 或 RViz**。底盘节点在此入口中使用 `read_only=true`：只接收 CAN 反馈，不请求 CAN 控制模式，也不订阅速度/灯光指令，便于遥控驾驶时采集。它不会替你把底盘从历史 CAN 模式切回遥控模式；录制脚本会显示 `control_mode`（`1`=CAN，`3`=遥控器），出车前由现场人员确认遥控器实际能接管。录包是单独的命令，便于先确认传感器正常，再决定何时采集。

## 两个终端使用

车机开机后接好相机、雷达、Fixposition 和底盘 CAN，开启各设备电源。在终端 A：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./start_scout_dataset.bash
```

默认使用 Jetson 原生 `can10`，500 kbps。若该接口尚未启用，脚本会要求 `sudo` 并配置本机已验证的 CAN0 引脚；其他 CAN 接口必须先手工配置，再以 `./start_scout_dataset.bash canX` 启动。终端 A 保持运行。相机用本机已安装的 ZED SDK 5.5.1 和官方 ROS 2 wrapper，彩色图、深度图发布为 640×360、约 15 Hz；深度图由 SDK 产生，无需另起深度估计节点。ZED 内置 IMU 的校正和原始话题配置为 200 Hz，不改变图像帧率。

在终端 B 先检查，然后开始录制：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./record_scout_dataset.bash --check
./record_scout_dataset.bash --require-rtk-fixed
```

脚本会等待 12 路必需话题各收到一条实际消息；任何一路缺失都会报错，且**不会产生空包**。它还要求 `/fixposition/odometry_llh` 是有效定位、经纬度不是设备启动时的 `0/0` 占位值。由于遥控估计速度只在 `control_mode=3` 发布，正式录制前须先让遥控器接管底盘。它同时打印 Fixposition GNSS1/GNSS2 状态码（`8` 表示 RTK Fixed）、底盘电池电压和当前底盘控制模式。上面的正式采集命令要求双天线 RTK Fixed；若需记录有效但未固定的定位片段，可去掉 `--require-rtk-fixed`，数据集报告会标出固定解比例。仅诊断时才用 `--allow-invalid-gnss` 绕过无效坐标检查。录制时终端 B 保持打开，按 `Ctrl+C` 结束；等待“Recording stopped”后再拔电或关机。终端 A 也用 `Ctrl+C` 停止驱动。

若 Autoware 已经启动并占用相同设备，`start_scout_dataset.bash` 会拒绝重复启动。先正常停止 Autoware 和单独启动的 ZED 节点，再运行终端 A 的命令。若坚持复用已启动的话题，必须先用 `./record_scout_dataset.bash --check` 确认全部 12 路（含遥控转换）都有实时数据。

## 会录制什么

录包脚本根据**当前有发布者**的话题生成清单，保存为会话目录内的 `session.json`。必需的 12 路是：

| 数据 | 必需话题 |
| --- | --- |
| 雷达原始点云 | `/rslidar_points` |
| ZED 彩色图 | `/zed2i/zed_node/rgb/color/rect/image` |
| ZED 深度图（`32FC1`，米） | `/zed2i/zed_node/depth/depth_registered` |
| ZED IMU | `/zed2i/zed_node/imu/data` |
| ZED SDK 左相机到内置 IMU 外参 | `/zed2i/zed_node/left_cam_imu_transform` |
| RTK/GNSS 坐标 | `/fixposition/odometry_llh` |
| RTK 状态 | `/fixposition/fpa/odomstatus` |
| Fixposition IMU | `/sensing/imu/imu_data` |
| 底盘电池与四个电机状态 | `/scout/status` |
| 遥控器开关及四轴摇杆原始值 | `/scout/rc_state` |
| 遥控估计线速度和角速度 | `/scout/rc_cmd_vel_estimated` |
| 轮速里程计 | `/scout/wheel_odometry` |

此外，录制**所有有发布者的 `/fixposition/` 话题**，包括原始/融合里程计、双天线 GNSS、状态、NMEA、IMU 和其 TF；ZED 的相机内参、健康状态和其他原始话题；有发布者时的底盘指令、轮 TF、ROS `/tf` 与 `/tf_static`、雷达包/IMU 话题。`/scout/status` 的 `battery_voltage` 是电池电压，`actuator_states[0..3]` 依次对应右前、左前、右后、左后，含各电机 `current`、`rpm`、`pulse_count`、`driver_voltage` 和温度。当前底盘固件的 `motor_id` 字段可能都为 0，分析时应使用数组下标。`/scout/rc_state` 是 CAN `0x241` 的四轴摇杆及开关反馈原始值，节点按 50 Hz 发布最近一次状态，`ScoutRCState` 不带消息头时间戳，bag 的接收时间作为这路数据的时间基准；它没有 `m/s` 或 `rad/s` 的速度单位，也不是 ROS `/scout/cmd_vel`。只读转换节点会额外发布 `/scout/rc_cmd_vel_estimated`（`geometry_msgs/TwistStamped`），用左纵摇杆估算 `linear.x`、右横摇杆估算 `angular.z`；仅在底盘报告遥控模式 `control_mode=3` 且状态新鲜时发布，绝不把该话题发给底盘。比例、方向、死区在 `config/scout_rc_command.yaml`。2026-10-24 的同步遥控/底盘反馈标定得到线速度约 `+0.01287 m/s/原始值`、角速度约 `-0.00489 rad/s/原始值`，配置取 `1.29 m/s` 和 `0.49 rad/s` 对应摇杆 ±100。这是按实测运动拟合的**估计指令**，不是底盘固件公布的目标速度；设备模式、限速档或负载改变时要重新标定。标定原始三路数据保存在 `/mnt/nvme0n1/autoware/bags/scout_rc_calibration_20261024/`。请同时保留原始摇杆与实际车速 `/scout/wheel_odometry.twist.twist` 用于训练和后续标定。遥控驾驶不经 ROS `cmd_vel`，因此只读采集时 `/scout/cmd_vel` 没有发布者。

图像只录原始彩色图和公制深度图；`compressed`、`compressedDepth`、`theora` 是同一图像的重复传输格式，不录这些副本以减少 CPU 和空间。`/rslidar_points` 是点云；现有雷达配置未启用原始 UDP 包的 ROS 发布，因此数据集没有雷达原始包。ZED 的彩色图编码是 `bgra8`，深度图是 `32FC1`，无效深度可能是 NaN。静态 TF 把车体 `zed2i_link` 接到官方相机 `zed2i_camera_link`；安装位置来自现有 `vehicle.xacro`，采集前仍应按实车重新核验外参。

### ZED 相机与内置 IMU 标定参数

相机序列号为 `39989134`。SDK 出厂相机标定文件位于 `/mnt/nvme0n1/autoware/zed_sdk_5.5.1/settings/SN39989134.conf`，包含左右相机在各采集分辨率下的内参、畸变和双目外参；录包脚本会把 SDK `settings/SN*.conf` 复制到每段 `config/`。实际发布的 **640×360、已校正图像**应使用 bag 里的 `/zed2i/zed_node/rgb/color/rect/camera_info` 与 `/zed2i/zed_node/depth/camera_info`，不要直接把配置文件里的 HD720 原始内参当作 640×360 内参。

相机到内置 IMU 的外参不在上述 `SN*.conf` 中，而由 ZED SDK `CameraInformation.sensors_configuration.camera_imu_transform` 给出。官方 wrapper 将它发布为 `/zed2i/zed_node/left_cam_imu_transform`（`zed2i_left_camera_frame` → `zed2i_imu_link`），并发布 IMU TF；新 bag 会记录该话题。2026-10-25 本机实测平移为 `[-0.002000, -0.023061, 0.000217]` 米、四元数 `xyzw=[0.00296353, 0.00031698, -0.00262496, 0.99999213]`。这是**左相机机身坐标系**到 IMU 的关系；图像消息使用 `zed2i_left_camera_frame_optical`，投影时还须经过光学坐标系的 TF。实测校正图像内参 `fx=fy=267.2845`、`cx=311.1378`、`cy=176.5193`；每段仍以本段 `camera_info` 为准。`/zed2i/zed_node/imu/data_raw` 是未校正的 IMU；它与校正后的 `/imu/data` 都配置为约 200 Hz，并会一起录制。

现场查看遥控原始输入、估计指令及实测轮速：

```bash
source ./scripts/scout_dataset_env.bash
ros2 topic echo /scout/rc_state
ros2 topic echo /scout/rc_cmd_vel_estimated
ros2 topic echo /scout/wheel_odometry
```

每个 `ros2 topic echo` 命令单独占用一个终端；用 `Ctrl+C` 退出。估计指令中的 `twist.linear.x` 单位是 m/s，`twist.angular.z` 单位是 rad/s。

## 文件与容量

### 分段录制与 RTK 原点

驱动保持运行时，可以多次启动、停止 `record_scout_dataset.bash`；每次都会生成独立的 `scout_dataset_时间` 目录。**开始新 bag 不会将 RTK 或 `FP_ENU0` 置零。** 2026-10-24 从本机 Fixposition 配置只读查询到 `enu0_mode=auto`：设备依据收到的第一条 ECEF 定位选择 ENU0；它不是每段 bag 的起点。原始经纬度 `/fixposition/odometry_llh` 和地心坐标 `/fixposition/odometry_ecef` 也不因分段而归零。

多段训练数据建议保留上述绝对位置以及 `/fixposition/tf_static` 中的 `FP_ECEF → FP_ENU0` 关系；`quality_report.json` 会列出 `fixposition_enu0_ecef_transform` 供逐段比较，并检测单包内原点变化。每段报告还会列出 `gnss_first_valid_fix`、`gnss_last_valid_fix`（起终点的经纬度、海拔和时间），以及两者离 bag 起止边界的秒数；没有有效定位时为 `null`，绝不把 0/0 当作起点。`segment_local_origin` 采用该段第一条有效 GNSS 坐标，供后处理建立段内局部坐标；bag 仍保留完整原始经纬度序列，不在录包过程中重置设备。若设备重启或 Fusion 重新初始化，自动 ENU0 可能重新选择；合并多段前应比较各段的 ECEF→ENU0 关系，不能只按相同的 `FP_ENU0` 名称直接拼接。Fixposition 也支持在设备端手动指定固定 ECEF 原点，但更改配置可能需要重载 I/O 或重启 Fusion，不适合录制中途切换。参考 [Fixposition ENU 输出说明](https://docs.fixposition.com/fd/fp_a-odomenu) 与 [设备 API 文档](https://docs.fixposition.com/fd/api-documentation)。

默认保存到 `/mnt/nvme0n1/autoware/bags/scout_dataset_时间/`：

```text
scout_dataset_时间/
├── session.json          # 时间、ROS 域、RTK 状态、电压、话题及类型、代码版本
├── config/               # 此次使用的 ZED、雷达、RTK、SN*.conf 和车体 xacro
└── rosbag/               # 一个 ROS 2 MCAP 包；约每 4 GiB 切一个文件
```

默认在剩余空间少于 10 GiB 时拒绝开始；录制期间也每 5 秒检查一次，低于阈值便向录包进程发送正常停止信号并等待写完 MCAP。可用 `--output-root /路径` 和 `--min-free-gb 数值` 调整。录制使用 MCAP、每 4 GiB 自动分片，未启用在线压缩，以减少 Jetson CPU 负载。实际容量取决于图像、点云和录制时长；2026-10-21 的短测约 18 秒、795 MiB，按此负载约 150 GiB/小时。长时间采集前检查 NVMe 空间。

结束后运行训练数据检查，再查看包内各话题：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./verify_scout_dataset.bash /mnt/nvme0n1/autoware/bags/scout_dataset_时间
source ./scripts/scout_dataset_env.bash
ros2 bag info /mnt/nvme0n1/autoware/bags/scout_dataset_时间/rosbag
```

`verify_scout_dataset.bash` 生成 `quality_report.json`，检查关键话题的条数和实测频率（新包 ZED IMU 目标 200 Hz）、相机到 IMU 外参话题、640×360 图像及内参、彩色图与深度图的消息头时间戳匹配率、有效深度像素、GNSS 有效坐标比例、RTK Fixed 比例、底盘四电机状态、遥控摇杆是否变化以及 `base_link` 到雷达、Fixposition 和 ZED 光学坐标系的 TF 连通性。任何关键数据失败时命令返回非零。会话清单里的话题只表示启动录包时发现了发布者；某些 Fixposition 可选话题即使有发布者，设备不输出该报文时仍可能在包中为 0 条。数据集如需精准对时，应结合报告内各路的消息头与入包时间差，确认设备时间源；本脚本不会把接收时间改成统一硬件时间。

**现有几何外参已有来源，但精度等级尚未证实。** 当前 `src/vehicle/scout_vehicle_description/urdf/vehicle.xacro` 已包含 `/home/agx/ele_scout_kit_github_20260827/src/ele_scout_kit/urdf/scout_sensor_rack.urdf.xacro` 的相机、雷达、Fixposition、双天线安装数值；无需另行复制。当前工程将车体原点沿 x 调整了 0.249 m，传感器相对车体的安装数值相同。ELE 文件自己说明支架高度采用网格接触拟合的 0.1162 m，而上游配置为 0.1823 m；所有这些传感器安装旋转均写为零。`FP_POI` 与设备网格原点同位、`zed2i_link → zed2i_camera_link` 零偏移也是模型假设。当前模型给出的 ZED 安装点相对雷达约前方 10.0 cm、下方 18.6 cm；现场目测认为前后距离可能只有几厘米，须量尺确认。2026-10-25 使用已有室内和室外 bag 的投影粗检见 `calibration/lidar_camera_probe_20261025/README.md`：未见大幅错位，但现有场景不能支持厘米级标定，未修改 URDF。若训练任务要精确把图像像素投到点云或融合 RTK 轨迹，应记录标定板/共同可见场景，并用实测外参核验或更新。

## 故障检查

- `can10` 没有报文：检查底盘供电、CAN 收发器和接线；`ip -details link show can10` 应显示 `UP`、500000 bitrate。`SCOUT_STARTUP.md` 有原生 CAN 引脚的手动配置方法。
- 缺少 `/scout/status` 或 `/scout/wheel_odometry`：检查 `can10` 的反馈帧、底盘节点日志和遥控器状态；遥控器能让车走，不代表 Jetson 收得到 CAN 数据。
- 缺少相机图像或深度图：检查 ZED USB 连接及终端 A 的 SDK/驱动日志；不要同时运行两个 ZED 节点。
- RTK 状态不是 `8`：检查天线、改正数据、时间和 Fixposition 设备状态。定位消息存在并不等于 RTK 固定解。
- 缺少雷达点云：检查雷达供电、网口和 `rslidar.yaml` 的端口配置；不要同时运行两个雷达驱动。
