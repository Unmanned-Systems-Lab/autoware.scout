#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
可配置PlatformStatus消息发布测试脚本
该脚本允许通过命令行参数配置消息字段并发布PlatformStatus消息
支持连续发送、指定发送次数、调整发送频率等功能
"""

import rclpy
from rclpy.node import Node
from udp_ros_bridge.msg import PlatformStatus
from std_msgs.msg import Header
import sys
import argparse
from datetime import datetime


class ConfigurablePlatformStatusPublisher(Node):
    def __init__(self, config, publish_frequency=1.0, send_count=-1):
        super().__init__('configurable_platform_status_publisher')
        
        # 保存配置
        self.config = config
        
        # 记录发送次数
        self.send_count = 0
        self.max_send_count = send_count
        
        # 创建发布者
        self.publisher = self.create_publisher(PlatformStatus, '/udp/platform_status', 10)
        
        # 设置发布频率
        timer_period = 1.0 / publish_frequency  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)
        
        self.get_logger().info(
            f'可配置PlatformStatus测试发布节点已启动\n'
            f'- 发布频率: {publish_frequency} Hz\n'
            f'- 发送次数限制: {"无限制" if send_count == -1 else send_count}\n'
            f'- 当前速度: {config["speed_current"]} m/s'
        )

    def publish_message(self):
        """发布PlatformStatus消息"""
        msg = PlatformStatus()
        
        # 设置头部信息
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 根据配置设置各个字段
        msg.msg_id = self.config['msg_id']
        msg.timestamp = int(datetime.now().timestamp() * 1000000)  # 使用当前时间戳（微秒）
        msg.msg_from = self.config['msg_from']
        msg.ctrl_mode = self.config['ctrl_mode']
        msg.run_mode = self.config['run_mode']
        msg.estop_state = self.config['estop_state']
        msg.warn_level = self.config['warn_level']
        msg.platform_state = self.config['platform_state']
        msg.parking_state = self.config['parking_state']
        msg.power_max = self.config['power_max']
        msg.speed_max = self.config['speed_max']
        msg.speed_current = int(self.config['speed_current'])  # Ensure it's an integer
        msg.current_lon = self.config['current_lon']
        msg.current_lat = self.config['current_lat']
        msg.current_high = self.config['current_high']
        msg.steer_angle = self.config['steer_angle']
        msg.steer_mode = self.config['steer_mode']
        msg.steer_faultcode = self.config['steer_faultcode']
        msg.steer_current = self.config['steer_current']
        msg.brake_pressure = self.config['brake_pressure']
        msg.brake_status = self.config['brake_status']
        msg.brake_fault_level = self.config['brake_fault_level']
        
        # UCM相关字段
        msg.ucmfault_lf = self.config['ucmfault_lf']
        msg.ucmfault_lb = self.config['ucmfault_lb']
        msg.ucmfault_rf = self.config['ucmfault_rf']
        msg.ucmfault_rb = self.config['ucmfault_rb']
        
        msg.ucmfaultcode_lf = self.config['ucmfaultcode_lf']
        msg.ucmfaultcode_lb = self.config['ucmfaultcode_lb']
        msg.ucmfaultcode_rf = self.config['ucmfaultcode_rf']
        msg.ucmfaultcode_rb = self.config['ucmfaultcode_rb']
        
        msg.ucmctemp_lf = self.config['ucmctemp_lf']
        msg.ucmctemp_lb = self.config['ucmctemp_lb']
        msg.ucmctemp_rf = self.config['ucmctemp_rf']
        msg.ucmctemp_rb = self.config['ucmctemp_rb']
        
        # 发布消息
        self.publisher.publish(msg)
        self.send_count += 1
        
        self.get_logger().info(
            f'[第{self.send_count}次] 已发布PlatformStatus消息: '
            f'ID={msg.msg_id}, Speed={msg.speed_current}, '
            f'Lat={msg.current_lat}, Lon={msg.current_lon}'
        )
        
        # 检查是否达到最大发送次数
        if self.max_send_count != -1 and self.send_count >= self.max_send_count:
            self.get_logger().info(f'已达到设定的发送次数 ({self.max_send_count})，停止发布')
            self.timer.cancel()


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='可配置PlatformStatus消息发布器')
    
    # 添加各种可配置参数
    parser.add_argument('--msg_id', type=int, default=0x00020001, help='消息ID (默认: 0x00020001)')
    parser.add_argument('--msg_from', type=int, default=0, help='消息来源 (默认: 0)')
    parser.add_argument('--ctrl_mode', type=int, default=2, help='控制模式 (默认: 2)')
    parser.add_argument('--run_mode', type=int, default=1, help='运行模式 (默认: 1)')
    parser.add_argument('--estop_state', type=int, default=0, help='急停状态 (默认: 0)')
    parser.add_argument('--warn_level', type=int, default=2, help='警告等级 (默认: 2)')
    parser.add_argument('--platform_state', type=int, default=0, help='平台状态 (默认: 0)')
    parser.add_argument('--parking_state', type=int, default=1, help='停车状态 (默认: 1)')
    parser.add_argument('--power_max', type=int, default=100, help='最大功率 (默认: 100)')
    parser.add_argument('--speed_max', type=int, default=50, help='最大速度 (默认: 50)')
    parser.add_argument('--speed_current', type=float, default=25.0, help='当前速度 (默认: 25.0)')
    parser.add_argument('--current_lon', type=int, default=123456789, help='经度 (默认: 123456789)')
    parser.add_argument('--current_lat', type=int, default=987654321, help='纬度 (默认: 987654321)')
    parser.add_argument('--current_high', type=int, default=100, help='高度 (默认: 100)')
    parser.add_argument('--steer_angle', type=int, default=500, help='转向角 (默认: 500)')
    parser.add_argument('--steer_mode', type=int, default=1, help='转向模式 (默认: 1)')
    parser.add_argument('--steer_faultcode', type=int, default=0, help='转向故障码 (默认: 0)')
    parser.add_argument('--steer_current', type=int, default=50, help='转向电流 (默认: 50)')
    parser.add_argument('--brake_pressure', type=int, default=200, help='刹车压力 (默认: 200)')
    parser.add_argument('--brake_status', type=int, default=1, help='刹车状态 (默认: 1)')
    parser.add_argument('--brake_fault_level', type=int, default=0, help='刹车故障等级 (默认: 0)')
    parser.add_argument('--ucmfault_lf', type=int, default=0, help='左前UCM故障 (默认: 0)')
    parser.add_argument('--ucmfault_lb', type=int, default=0, help='左后UCM故障 (默认: 0)')
    parser.add_argument('--ucmfault_rf', type=int, default=0, help='右前UCM故障 (默认: 0)')
    parser.add_argument('--ucmfault_rb', type=int, default=0, help='右后UCM故障 (默认: 0)')
    parser.add_argument('--ucmfaultcode_lf', type=int, default=0, help='左前UCM故障码 (默认: 0)')
    parser.add_argument('--ucmfaultcode_lb', type=int, default=0, help='左后UCM故障码 (默认: 0)')
    parser.add_argument('--ucmfaultcode_rf', type=int, default=0, help='右前UCM故障码 (默认: 0)')
    parser.add_argument('--ucmfaultcode_rb', type=int, default=0, help='右后UCM故障码 (默认: 0)')
    parser.add_argument('--ucmctemp_lf', type=int, default=25, help='左前UCM温度 (默认: 25)')
    parser.add_argument('--ucmctemp_lb', type=int, default=25, help='左后UCM温度 (默认: 25)')
    parser.add_argument('--ucmctemp_rf', type=int, default=25, help='右前UCM温度 (默认: 25)')
    parser.add_argument('--ucmctemp_rb', type=int, default=25, help='右后UCM温度 (默认: 25)')
    
    parser.add_argument(
        '-f', '--frequency', 
        type=float, 
        default=1.0,
        help='发布频率 (Hz, 默认: 1.0)'
    )
    parser.add_argument(
        '-n', '--number', 
        type=int, 
        default=-1,
        help='发送次数 (-1表示无限, 默认: -1)'
    )
    
    args = parser.parse_args(argv)
    
    # 将参数转换为字典
    config = {k: v for k, v in vars(args).items() 
              if k not in ['frequency', 'number']}
    
    rclpy.init(args=[sys.argv[0]] + argv)
    
    # 创建并运行可配置的测试发布者节点
    publisher_node = ConfigurablePlatformStatusPublisher(
        config=config,
        publish_frequency=args.frequency,
        send_count=args.number
    )
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info('接收到终止信号')
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()