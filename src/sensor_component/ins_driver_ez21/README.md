# ins_driver_ez21

`ins_driver_ez21` 是一个基于 ROS 2 `ament_python` 的 INS 驱动包。
它迁移了 `/media/nvidia/program/xmt.tools_by_can` 中与 GPS/INS CAN 报文读取、经纬高转本地坐标、以及 `nav_msgs/msg/Odometry` 发布相关的实现，并整理成了一个更聚焦的驱动仓库。

当前协议解析保持与来源实现一致，默认使用这些 CAN ID：

- `0x10B`：航向角、俯仰角、横滚角
- `0x20B` / `0x21B`：纬度、经度
- `0x30B` / `0x31B`：海拔、导航状态
- `0x40B` / `0x41B`：东、北、天速度
- `0x60B`：角速度 `wx`、`wy`
- `0x70B`：角速度 `wz`

节点会在拿到第一组有效经纬高后，将该点作为局部 ENU 原点，再持续发布里程计。

## 功能

- 从 Linux `SocketCAN` 读取 INS CAN 报文
- 解析姿态、位置、速度和角速度
- 将 WGS84 经纬高转换为局部 ENU 坐标
- 发布 `nav_msgs/msg/Odometry`
- 发布 `sensor_msgs/msg/NavSatFix`（默认话题 `/sensing/ins/raw_nav_sat_fix`，供 UI 直接读取经纬度）
- 按需记录原始 CAN 文本或 odometry CSV
- 提供离线 GPS 提取工具

## 构建

```bash
cd <your_ros2_workspace>
colcon build --packages-select ins_driver_ez21
source install/setup.bash
```

## 运行驱动

默认启动：

```bash
ros2 launch ins_driver_ez21 ins_driver.launch.py
```

覆盖参数示例：

```bash
ros2 launch ins_driver_ez21 ins_driver.launch.py \
  can_interface:=can0 \
  topic_name:=/ins/odom \
  frame_id:=odom \
  child_frame_id:=base_link \
  publish_raw_nav_sat_fix:=true \
  raw_nav_sat_fix_topic:=/sensing/ins/raw_nav_sat_fix \
  raw_nav_sat_fix_frame_id:=ins_link \
  log_enabled:=true \
  log_format:=odom_csv \
  log_path:=/tmp/ins_logs
```

默认参数文件位于 [config/driver.yaml](config/driver.yaml)。

## 参数

- `can_interface`：SocketCAN 接口名，默认 `can0`
- `topic_name`：里程计发布话题，默认 `/odom`
- `frame_id`：`Odometry.header.frame_id`，默认 `odom`
- `child_frame_id`：`Odometry.child_frame_id`，默认 `base_link`
- `publish_raw_nav_sat_fix`：是否发布原始 GPS 到 `NavSatFix`，默认 `true`
- `raw_nav_sat_fix_topic`：原始 GPS `NavSatFix` 话题，默认 `/sensing/ins/raw_nav_sat_fix`
- `raw_nav_sat_fix_frame_id`：原始 GPS 消息的 `header.frame_id`，默认 `ins_link`
- `socket_timeout_sec`：CAN 接收超时，默认 `0.2`
- `log_enabled`：是否记录文件，默认 `false`
- `log_path`：日志目录或显式 `.txt/.csv` 文件路径
- `log_name`：日志文件名，`auto` 表示按时间戳生成
- `log_name_format`：`log_name=auto` 时的时间格式
- `log_format`：`can` 或 `odom_csv`

## 日志格式

`log_format=can` 时，每一行都是：

```text
10B#D2042D0000000000
```

`log_format=odom_csv` 时，首行为表头，后续每行是一条已发布的里程计样本。

## 离线提取 GPS

如果已有一份原始 CAN TXT，可以直接提取经纬度：

```bash
python3 -m ins_driver_ez21.extract_gps_from_can_txt /tmp/replay_can.txt -o /tmp/gps.csv
```

输入格式与记录出的 `CANID#DATA` 一致。

## 说明

- 当前解码规则直接迁移自 `/media/nvidia/program/xmt.tools_by_can/src/gi5651_can_odom`
- 如果 EZ21 的帧 ID 或比例因子与 GI5651 不同，需要继续按设备协议调整 [ins_driver_node.py](ins_driver_ez21/ins_driver_node.py) 和 [protocol.py](ins_driver_ez21/protocol.py)
