# EZ21 Vehicle Interface

## 概述

`ez21_vehicle_launch` 提供 EZ21 车辆的 ROS 2 vehicle interface。

当前实现的定位是：

- 上游接收 Autoware 的控制命令、档位命令、控制模式请求
- 通过 `can1` 向 VCU 下发标准帧 `0x102`
- 从车辆 CAN 总线接收状态反馈，并发布 Autoware 需要的车辆状态 topic

当前版本已经切换为以 VCU 汇总帧为主：

- 控制下发主帧：`0x102`，标准帧
- 控制模式反馈主帧：`0x203`，标准帧

其余状态反馈目前仍保留原有低层反馈来源：

- 转向反馈：`0x220`、`0x201`
- 转向故障：`0x80`
- 驱动反馈：`0x180F0100`、`0x18130100`、`0x181B0100`、`0x181F0100`
- 兼容驱动反馈：`0x180`、`0x181`、`0x18B`、`0x18F`

本实现依据现场协议文件 `201整车通讯协议.xlsx` 整理而来。

## ROS 接口

### 输入

- `input/control_cmd`
  对应 Autoware 的 `/control/command/control_cmd`
- `input/gear_cmd`
  对应 Autoware 的 `/control/command/gear_cmd`
- `input/turn_indicators_cmd`
  对应 Autoware 的 `/control/command/turn_indicators_cmd`
- `input/hazard_lights_cmd`
  对应 Autoware 的 `/control/command/hazard_lights_cmd`
- `input/control_mode_request`
  对应 Autoware 的 `/control/control_mode_request`

### 输出

- `output/velocity_status`
  remap 到 `/vehicle/status/velocity_status`
- `output/steering_status`
  remap 到 `/vehicle/status/steering_status`
- `output/gear_status`
  remap 到 `/vehicle/status/gear_status`
- `output/control_mode`
  remap 到 `/vehicle/status/control_mode`

### Launch 入口

通过 [`launch/vehicle_interface.launch.xml`](./launch/vehicle_interface.launch.xml) 启动时，车辆接口直接使用上游 `control_cmd` 与其他车辆命令，不再依赖 `actuation_cmd`。

## 当前 CAN 设计

### 发送

当前周期性发送的控制帧只有一帧：

- `0x102`
  标准帧，VCU 控制指令

当前 `send_command_frames()` 只发送 `0x102`，不再实际发送旧的 `0x118` 和 `0x70300/0x70400/0x70600/0x70700` 控制帧。

### 接收

- `0x203`
  标准帧，VCU 到自主，当前用于 `/vehicle/status/control_mode`
- `0x220`
  扩展帧，直接转向反馈，当前用于 `/vehicle/status/steering_status`
- `0x201`
  标准帧，遥控器侧转向反馈，当前作为转向反馈兼容来源
- `0x80`
  扩展帧，转向故障反馈，当前影响 `/vehicle/status/control_mode`
- `0x180F0100`、`0x18130100`、`0x181B0100`、`0x181F0100`
  扩展帧，四个驱动器反馈，当前用于 `/vehicle/status/velocity_status`
- `0x180`、`0x181`、`0x18B`、`0x18F`
  标准帧，兼容驱动反馈来源

## `0x102` 字段映射

当前 `0x102` 的 8 字节打包逻辑如下。

### byte0

- `bit3`
  中心转向
  当目标转角绝对值很小的时候置位
- `bit2`
  前进
  当控制模式为自动且档位为前进时置位
- `bit1`
  倒退
  当控制模式为自动且档位为倒车时置位

当前未使用：

- `bit7`
  急停，当前始终为 `0`

### byte1

- `bit0`
  驻车制动
  当档位命令为 `PARK` 时置位

当前未使用：

- `bit1`
  制动排气，当前始终为 `0`

### byte2

- 油门控制
- 按 `0..100` 百分比发送
- 直接使用 `control_cmd.longitudinal.velocity` 换算
- 当前公式为 `throttle = 10 * (V + 0.17)`
- 实现中 `V` 取速度绝对值，并对结果钳制到 `0..100`

### byte3

- 行车制动
- 按 `0..100` 百分比发送
- 基础制动来自 `control_cmd.longitudinal.acceleration` 的负加速度部分，按参数线性归一化
- 另外支持“超速制动”：
  当实际车速高于 `control_cmd.longitudinal.velocity` 时，会按超速比例查表追加刹车
- 这里的“超速”只比较实际车速与参考控制速度，不使用道路限速
- 超速制动生效时，`byte2` 油门会被压为 `0`，避免边给油边刹车

### byte4 / byte5

- `byte4`
  左转，按 `0..100`
- `byte5`
  右转，按 `0..100`

当前下发使用 `control_cmd` 的转向指令，但这里有一层符号转换：

- `Autoware /control/command/control_cmd`：左转为正，右转为负
- 车侧 `0x102 byte4/byte5`：左转为负，右转为正
- `Vehicle interface` 会在两者之间自动反号
- 量程为 `-30 deg .. +30 deg`

转向量由目标前轮转角与 `max_steer_angle_rad` 的比例得到；当前默认 `max_steer_angle_rad = 30 deg`，因此：

- `Autoware +30 deg` 会转换为 `byte4 = 100`、`byte5 = 0`
- `0 deg` 对应 `byte4 = 0`、`byte5 = 0`
- `Autoware -30 deg` 会转换为 `byte4 = 0`、`byte5 = 100`

### byte6

- 限速，按 `0..100`
- 当前实现固定发送 `50`

### byte7

- 心跳
- 当前实现为 8 位递增计数，周期发送时自增

## `0x203` 与 `/vehicle/status/control_mode`

当前已接入的 `0x203` 字段只有一项：

- `byte0 bit0`
  自主行驶状态
  `0` 表示关
  `1` 表示开

当前 `/vehicle/status/control_mode` 的判定优先级为：

1. 若检测到转向故障，则发布 `NOT_READY`
2. 若已收到 `0x203`，则：
   - `byte0 bit0 = 1` 发布 `AUTONOMOUS`
   - `byte0 bit0 = 0` 发布 `MANUAL`
3. 若还未收到 `0x203`，则退回旧逻辑：
   - 尝试从转向反馈模式位推断
   - 再退回到最近一次 `control_mode_request`

## 车辆状态发布逻辑

### `/vehicle/status/velocity_status`

- 当前不是直接来自 VCU 整车速度帧
- 而是用 4 个驱动器反馈的轮速求平均
- 再用 `wheel_radius_m` 换算为纵向速度
- `heading_rate` 通过纵向速度、转角和 `wheel_base_m` 估算

### `/vehicle/status/steering_status`

- 优先来源于 `0x220`
- 兼容来源于 `0x201`
- 原始转向值通过 `steering_center_raw` 和 `steering_counts_per_radian` 换算成前轮转角
- 车侧反馈符号约定为：左转为负，右转为正
- 发布到 `/vehicle/status/steering_status` 前会转换为 Autoware 约定：左转为正，右转为负

### `/vehicle/status/gear_status`

- 当前协议里没有独立档位反馈帧被接入
- 因此此 topic 仍是“推导值”，不是总线原生反馈
- 有速度时：
  - 正速度判为 `DRIVE`
  - 负速度判为 `REVERSE`
- 近零速时：
  - 回退到最近一次 `gear_cmd`

## 参数说明

参数文件见 [`config/vehicle_interface.param.yaml`](./config/vehicle_interface.param.yaml)。

### 运行必需

以下参数对当前实现是实质生效的，建议按实车标定值配置。

- `can_interface`
  车辆 CAN 口，当前默认是 `can1`
- `wheel_radius_m`
  用于：
  - 轮速换算速度
  - `0x102 byte6` 限速比例换算
- `max_steer_angle_rad`
  用于：
  - `0x102 byte4/5` 左右转比例换算
- `drive_max_rpm`
  用于：
  - `0x102 byte6` 限速比例换算
### 强烈建议正确标定

以下参数不会阻止节点启动，但若配置错误，会直接影响状态质量。

- `wheel_base_m`
  影响 `/vehicle/status/velocity_status.heading_rate`
- `steering_center_raw`
  影响转向原始值到弧度的零点
- `steering_counts_per_radian`
  影响转向原始值到弧度的比例
- `velocity_zero_threshold_mps`
  影响近零速时 `gear_status` 的判定边界

### 纵向映射参数

以下参数参与 `control_cmd` 到油门/制动量的映射。

- `fallback_accel_limit_mps2`
  用于把 `control_cmd.longitudinal.acceleration` 映射成 `byte2`
- `fallback_brake_limit_mps2`
  用于把负加速度映射成 `byte3`

### 超速制动参数

- `overspeed_brake_enabled`
  是否启用基于实际车速反馈的超速查表制动
- `overspeed_brake_deadband_mps`
  超速死区，小于该值时不触发额外刹车
- `overspeed_brake_ratio_denominator_min_mps`
  计算超速比例时的最小分母，避免低速目标下比例过大
- `overspeed_brake_ratio_points`
  超速比例查表横轴，要求非负且单调不减
- `overspeed_brake_cmd_points`
  与 `overspeed_brake_ratio_points` 一一对应的刹车指令，范围 `0.0..1.0`

当前超速比例定义为：

- `overspeed_ratio = max(actual_speed - reference_speed - deadband, 0) / max(reference_speed, denominator_min)`

其中：

- `actual_speed` 来自 4 个驱动轮反馈平均车速
- `reference_speed` 来自 `control_cmd.longitudinal.velocity` 的绝对值

### 当前保留但未走主路径

以下参数来自早期低层直驱方案，当前主发送路径已切换为 `0x102`，因此这些参数目前不会影响实际下发。

- `steering_min_speed_deg_per_s`
- `steering_max_speed_deg_per_s`
- `drive_current_limit_a`
- `throttle_ad_max_raw`

对应的低层辅助函数仍保留在代码里，但 `send_command_frames()` 当前不会发送旧的低层控制帧。

## 已知限制

- 当前 `0x102` 只接了最小可用字段集，`急停` 和 `制动排气` 还没有上游明确控制语义，默认保持为 `0`
- 当前 `/vehicle/status/control_mode` 只使用了 `0x203` 的 `byte0 bit0`
- 当前 `/vehicle/status/gear_status` 仍为推导值，不是独立 CAN 原生反馈
- 当前没有接入独立制动反馈帧，因此制动状态没有单独总线上报
- 当前转向和车速状态仍来自低层反馈帧，而不是统一由 VCU 汇总反馈

## 调试建议

### 干跑启动

```bash
source install/setup.bash
ros2 run ez21_vehicle_launch ez21_vehicle_interface_node --ros-args -p enable_can_io:=false
```

### 正常 launch

```bash
ros2 launch ez21_vehicle_launch vehicle_interface.launch.xml
```

### 现场联调优先确认

- `can1` 是否已 up
- `0x102` 是否按 20 ms 周期正常发送
- `0x203` 是否返回，且 `byte0 bit0` 能正确随整车自动状态变化
- `0x220/0x201` 是否能正确反映方向盘或转向器动作
- 四个驱动反馈帧是否都存在，否则 `/vehicle/status/velocity_status` 会偏差较大
