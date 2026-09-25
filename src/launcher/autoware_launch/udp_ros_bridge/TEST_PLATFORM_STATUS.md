# PlatformStatus 消息测试工具

此项目包含用于模拟发送 PlatformStatus 消息的测试脚本，支持从配置文件加载消息内容。

## 文件说明

### 1. test_config.yaml
配置文件模板，定义了 PlatformStatus 消息各字段的值：
```yaml
PlatformStatus:
  msg_id: 1                    # 消息ID
  timestamp: 1234567890        # 时间戳
  msg_from: 1                  # 消息来源
  car_pose: 0                  # 车辆姿态
  run_mode: 1                  # 运行模式
  ctrl_mode: 2                 # 控制模式
  work_state: 3                # 工作状态
  center_turn: 0               # 中心转向
  estop_state: 0               # 紧急停止状态
  power_off: 0                 # 断电状态
  motor_mode: 1                # 电机模式
  warn_level: 2                # 警告等级
  platform_state: 0            # 平台状态
  silence_mode: 0              # 静音模式
  parking_state: 1             # 停车状态
  power_max: 100               # 最大功率
  speed_max: 50                # 最大速度
  speed_current: 25            # 当前速度
  current_lon: 123456789       # 当前经度
  current_lat: 987654321       # 当前纬度
  steer_angle: 500             # 转向角度
  brake_pressure: 200          # 制动压力
  check_sum: 0                 # 校验和
```

### 2. test_platform_status_publisher.py
基础测试脚本，以固定频率持续发送 PlatformStatus 消息。

#### 使用方法：
```bash
cd /home/nvidia/udp_ros_bridge
source install/setup.bash
ros2 run udp_ros_bridge test_platform_status_publisher.py [配置文件路径]
```

### 3. advanced_platform_status_publisher.py
高级测试脚本，支持以下功能：
- 自定义发布频率
- 限制发送次数
- 更详细的日志输出

#### 使用方法：
```bash
cd /home/nvidia/udp_ros_bridge
source install/setup.bash
python3 src/udp_ros_bridge/advanced_platform_status_publisher.py --config [配置文件路径] --frequency [发布频率] --number [发送次数]
```

#### 参数说明：
- `-c`, `--config`: 配置文件路径（默认：test_config.yaml）
- `-f`, `--frequency`: 发布频率（Hz，默认：1.0）
- `-n`, `--number`: 发送次数（-1 表示无限次，默认：-1）

#### 示例：
```bash
# 使用默认配置，以2Hz频率发送10次消息
python3 src/udp_ros_bridge/advanced_platform_status_publisher.py -f 2.0 -n 10

# 指定自定义配置文件，以0.5Hz频率持续发送
python3 src/udp_ros_bridge/advanced_platform_status_publisher.py -c /path/to/custom_config.yaml -f 0.5
```

## 注意事项

1. 在运行脚本之前，请确保 ROS2 环境已正确设置。
2. 可以根据实际需求修改配置文件中的值。
3. 校验和(check_sum)会在运行时自动计算更新。
4. 高级脚本会自动使用当前时间戳替换配置文件中的时间戳值。