# Fixposition / RTK 与 Scout 轮速数据包

导出脚本 `scripts/export_fixposition_bags.py` 从 2026-10-24 和 2026-10-25 的每段 Scout 原始 bag 生成一段对应的新 MCAP。原始 bag 不改动。导出**不按 RTK Fixed 状态筛选、不裁剪时间、不降采样**，因此 Fixed、非 Fixed、无效定位报文都会保留，便于之后自行筛选。

保留的 ROS 话题：

- 所有实际有消息的 `/fixposition/*`：GNSS1/GNSS2、经纬度、ECEF/ENU 里程计、融合状态、NMEA、原始/修正 IMU、设备 TF 等。
- `/sensing/imu/imu_data`：Fixposition 驱动把 `/fixposition/poiimu` 重映射后的 IMU，来自 10 Hz 的融合里程计报文，为偏置修正的加速度与角速度，并非高频原始 IMU。
- `/tf_static`：车体到 RTK 等传感器的静态外参。它也可能包含其他传感器的静态关系，保留原始消息以避免改变坐标树。
- `/scout/wheel_odometry`：Scout 轮速推算的车体线速度、角速度与里程计。
- `/scout/status`：四个电机的 RPM、脉冲计数，以及电流、电压和底盘状态。`actuator_states[0..3]` 依次为右前、左前、右后、左后；以数组下标识别电机。
- `/scout/wheel_tf`：轮速里程计对应的动态 TF。
- `/scout/rc_cmd_vel_estimated`：由遥控摇杆换算的估计指令线速度 `twist.linear.x` 和角速度 `twist.angular.z`；这是估计指令，不是底盘实测速度。
- `/scout/rc_state`：遥控器原始摇杆与开关状态，用于核验上述估计指令；它没有消息头时间戳，应使用 bag 接收时间。

**Scout 实测线速度和角速度**同时在 `/scout/wheel_odometry.twist.twist.linear.x`、`.angular.z` 以及 `/scout/status.linear_velocity`、`.angular_velocity` 中；估计遥控指令在 `/scout/rc_cmd_vel_estimated.twist` 中。原包的 `config/scout_rc_command.yaml` 有则复制到导出目录，记录摇杆换算比例。

每段输出的 `rosbag/` 是可由 `ros2 bag info` / `ros2 bag play` 使用的 MCAP；`export_manifest.json` 记录来源、话题和消息数。`source_session.json`、原包质量报告及 `config/fixposition.yaml` 有则随包复制。MCAP 消息序列化内容和 bag 接收时间戳原样复制。该输出没有彩色图、深度图或雷达点云；需要全部传感器数据时使用原始 bag。

这两天原包里实际有消息的 RTK 相关话题为 11 路，另有 3 路 Scout 轮速/底盘话题；遥控原始与估计指令在 14 段中有数据，最早两段原包中为 0 条。`/fixposition/gnss1`、`/fixposition/gnss2`、多路 `/fixposition/nmea/*` 以及若干原始报文话题虽列在 ROS 发布清单中，但原包消息数为 0；导出包不会凭空生成这些数据。双天线 Fixed 状态仍可从 `/fixposition/fpa/odomstatus` 读取，融合经纬度在 `/fixposition/odometry_llh`。

IMU 噪声检查见根目录 `imu_noise_audit.json`：所有 113,924 条 `/sensing/imu/imu_data` 的三组协方差均为 0，表示该 ROS 话题没有提供数值噪声协方差；其 `orientation` 也由驱动保持默认值。`/fixposition/fpa/odomstatus.imu_noise` 有低/中/高三档噪声状态，但不是加速度计/陀螺仪噪声密度或随机游走参数。`/fixposition/fpa/rawimu`、`corrimu` 和 `imubias` 在这些原包中均为 0 条，不能从导出包取得原始高频 IMU 或偏置序列。里程计和经纬度消息中的非零协方差是融合定位估计的不确定度，不是 IMU 本体噪声。

## 状态字段与天线外参

每段的 `rtk_status_fields.csv` 逐条导出 `/fixposition/fpa/odomstatus`，保留 bag 接收时间和报文头时间（均为纳秒），包括 `init_status`、`wheelspeed_status`（原消息 `ws_status`）、`gnss1_status`、`gnss2_status`、`correction_status`（原消息 `corr_status`）、`imu_status`、`imu_noise`，以及融合轮速、改正数、GNSS、IMU 收敛和基线状态。`rtk_status_fields_summary.json` 给出该段各代码计数。代码含义以 [Fixposition FP_A-ODOMSTATUS 文档](https://docs.fixposition.com/fd/fp_a-odomstatus)为准，例如 GNSS `8` 为 Fixed、`5` 为 Float、`0` 为无 Fix；`imu_noise` 的 `1/2/3` 为低/中/高。

根目录 `antenna_extrinsics_device_snapshot.json` 是 2026-10-25 从本车 Fixposition `/api/v2/params/config/get` **只读**取得的当前配置，并复制为每段的 `antenna_extrinsics_reference.json`。设备配置的 GNSS1、GNSS2 相对于 **Fixposition 传感器坐标系** 的平移分别为 `[0.02, +0.31, -0.02]` 米、`[0.02, -0.31, -0.02]` 米，配置基线为 0.620 米。这是设备当前的天线相位中心配置值，**历史每段录制时是否完全相同未被证明**。bag 内 `/tf_static` 另保存车辆 URDF 的天线安装网格位置，其原点和坐标系不等于设备内部的天线相位中心配置，不能直接互相替代。

两天状态报文共 113,858 条；`init_status=2` 占 113,362 条，`wheelspeed_status=0` 占全部 113,858 条，`correction_status=5` 占 113,312 条。`wheelspeed_status=0` 按设备状态定义表示 Fixposition 融合未启用轮速输入；新增的 `/scout/wheel_odometry` 与 `/scout/status` 是供后处理对齐的独立 Scout 数据，不会追溯改变设备内部融合状态。完整分段统计见各目录的 `rtk_status_fields_summary.json`。

默认导出位置：`/mnt/nvme0n1/autoware/bags/fixposition_only_20261024_20261025/`，沿用先前 RTK 导出目录名称。每个子目录名称与原始 `scout_dataset_时间` 对应。更新时替换各子目录的 `rosbag/`；先前 RTK 单独包留在隐藏子目录 `.rtk_only_backup/`，只含 RTK 和轮速的中间版本留在 `.rtk_wheels_backup/`。若导出中断，已完成且消息数匹配的子目录会跳过；`.partial` 子目录表示该段未完成，须检查后再处理。

重新运行或检查：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
python3 scripts/export_fixposition_bags.py
# 如果当前目录仍是旧的 RTK 单独包，第一次加入 Scout 轮速时使用：
python3 scripts/export_fixposition_bags.py --update-existing
source scripts/scout_dataset_env.bash
ros2 bag info /mnt/nvme0n1/autoware/bags/fixposition_only_20261024_20261025/scout_dataset_时间/rosbag
```

`/fixposition/odometry_llh` 中的非零经纬度不保证当前定位有效；须同时查看其 `status.status` 及 `/fixposition/fpa/odomstatus`。已有部分原包在末尾出现 GNSS `0/0`，这些报文同样保留在导出包中。
