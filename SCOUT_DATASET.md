# Scout 独立数据集采集

这套入口只启动 Scout 底盘反馈、Robosense RSHELIOS、Fixposition RTK/IMU、ZED 2i 官方驱动，以及车体静态坐标关系。**不启动 Autoware、地图、规划、控制、UI 或 RViz**。录包是单独的命令，便于先确认传感器正常，再决定何时采集。

## 两个终端使用

车机开机后接好相机、雷达、Fixposition 和底盘 CAN，开启各设备电源。在终端 A：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./start_scout_dataset.bash
```

默认使用 Jetson 原生 `can10`，500 kbps。若该接口尚未启用，脚本会要求 `sudo` 并配置本机已验证的 CAN0 引脚；其他 CAN 接口必须先手工配置，再以 `./start_scout_dataset.bash canX` 启动。终端 A 保持运行。相机用本机已安装的 ZED SDK 5.5.1 和官方 ROS 2 wrapper，彩色图、深度图发布为 640×360、约 15 Hz；深度图由 SDK 产生，无需另起深度估计节点。

在终端 B 先检查，然后开始录制：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./record_scout_dataset.bash --check
./record_scout_dataset.bash
```

脚本会等待 9 路必需话题各收到一条实际消息；任何一路缺失都会报错，且**不会产生空包**。它同时打印 Fixposition GNSS1/GNSS2 状态码（`8` 表示 RTK Fixed）和底盘电池电压。定位未达 RTK Fixed 时会提示，但允许录制，状态码也写进会话清单，便于后续筛选。录制时终端 B 保持打开，按 `Ctrl+C` 结束；等待“Recording stopped”后再拔电或关机。终端 A 也用 `Ctrl+C` 停止驱动。

若 Autoware 已经启动并占用相同设备，`start_scout_dataset.bash` 会拒绝重复启动。此时可直接运行 `./record_scout_dataset.bash` 录制现有话题；若要测试独立工作流，先正常停止 Autoware 和单独启动的 ZED 节点，再运行终端 A 的命令。

## 会录制什么

录包脚本根据**当前有发布者**的话题生成清单，保存为会话目录内的 `session.json`。必需的 9 路是：

| 数据 | 必需话题 |
| --- | --- |
| 雷达原始点云 | `/rslidar_points` |
| ZED 彩色图 | `/zed2i/zed_node/rgb/color/rect/image` |
| ZED 深度图（`32FC1`，米） | `/zed2i/zed_node/depth/depth_registered` |
| ZED IMU | `/zed2i/zed_node/imu/data` |
| RTK/GNSS 坐标 | `/fixposition/odometry_llh` |
| RTK 状态 | `/fixposition/fpa/odomstatus` |
| Fixposition IMU | `/sensing/imu/imu_data` |
| 底盘电池与四个电机状态 | `/scout/status` |
| 轮速里程计 | `/scout/wheel_odometry` |

此外，录制**所有有发布者的 `/fixposition/` 话题**，包括原始/融合里程计、双天线 GNSS、状态、NMEA、IMU 和其 TF；ZED 的相机内参、健康状态和其他原始话题；有发布者时的底盘指令、轮 TF、ROS `/tf` 与 `/tf_static`、雷达包/IMU 话题。`/scout/status` 的 `battery_voltage` 是电池电压，`actuator_states[0..3]` 依次对应右前、左前、右后、左后，含各电机 `current`、`rpm`、`pulse_count`、`driver_voltage` 和温度。当前底盘固件的 `motor_id` 字段可能都为 0，分析时应使用数组下标。

图像只录原始彩色图和公制深度图；`compressed`、`compressedDepth`、`theora` 是同一图像的重复传输格式，不录这些副本以减少 CPU 和空间。`/rslidar_points` 是点云；现有雷达配置未启用原始 UDP 包的 ROS 发布，因此数据集没有雷达原始包。ZED 的彩色图编码是 `bgra8`，深度图是 `32FC1`，无效深度可能是 NaN。静态 TF 把车体 `zed2i_link` 接到官方相机 `zed2i_camera_link`；安装位置来自现有 `vehicle.xacro`，采集前仍应按实车重新核验外参。

## 文件与容量

默认保存到 `/mnt/nvme0n1/autoware/bags/scout_dataset_时间/`：

```text
scout_dataset_时间/
├── session.json          # 时间、ROS 域、RTK 状态、电压、话题及类型、代码版本
├── config/               # 此次使用的 ZED、雷达、RTK 参数和车体 xacro
└── rosbag/               # 一个 ROS 2 MCAP 包；约每 4 GiB 切一个文件
```

默认在剩余空间少于 10 GiB 时拒绝开始；录制期间也每 5 秒检查一次，低于阈值便向录包进程发送正常停止信号并等待写完 MCAP。可用 `--output-root /路径` 和 `--min-free-gb 数值` 调整。录制使用 MCAP、每 4 GiB 自动分片，未启用在线压缩，以减少 Jetson CPU 负载。实际容量取决于图像、点云和录制时长；2026-10-21 的短测约 18 秒、795 MiB，按此负载约 150 GiB/小时。长时间采集前检查 NVMe 空间。

结束后查看包内每路的实际消息数：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
ros2 bag info /mnt/nvme0n1/autoware/bags/scout_dataset_时间/rosbag
```

会话清单里的话题表示启动录包时发现了发布者；录完后仍应以 `ros2 bag info` 的消息数确认各路真的连续写入。数据集如需精准对时，应另外检查各设备时间源及 ROS 消息头时间戳；本脚本不会把 PC 接收时间自动改成统一硬件时间。

## 故障检查

- `can10` 没有报文：检查底盘供电、CAN 收发器和接线；`ip -details link show can10` 应显示 `UP`、500000 bitrate。`SCOUT_STARTUP.md` 有原生 CAN 引脚的手动配置方法。
- 缺少 `/scout/status` 或 `/scout/wheel_odometry`：检查 `can10` 的反馈帧、底盘节点日志和遥控器状态；遥控器能让车走，不代表 Jetson 收得到 CAN 数据。
- 缺少相机图像或深度图：检查 ZED USB 连接及终端 A 的 SDK/驱动日志；不要同时运行两个 ZED 节点。
- RTK 状态不是 `8`：检查天线、改正数据、时间和 Fixposition 设备状态。定位消息存在并不等于 RTK 固定解。
- 缺少雷达点云：检查雷达供电、网口和 `rslidar.yaml` 的端口配置；不要同时运行两个雷达驱动。
