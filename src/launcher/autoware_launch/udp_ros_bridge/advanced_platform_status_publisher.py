#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
高级PlatformStatus消息发布测试脚本
该脚本从配置文件中读取参数并发布PlatformStatus消息
支持连续发送、指定发送次数、调整发送频率等功能
"""

import rclpy
from rclpy.node import Node
from udp_ros_bridge.msg import PlatformStatus
from std_msgs.msg import Header
import yaml
import sys
import argparse
from datetime import datetime
import os


class AdvancedPlatformStatusPublisher(Node):
    def __init__(self, config_file_path, publish_frequency=1.0, send_count=-1):
        super().__init__('advanced_platform_status_publisher')
        
        # 保存配置文件路径
        self.config_file_path = config_file_path
        
        # 记录发送次数
        self.send_count = 0
        self.max_send_count = send_count
        
        # 创建发布者
        self.publisher = self.create_publisher(PlatformStatus, '/udp/platform_status', 10)
        
        # 设置发布频率
        timer_period = 1.0 / publish_frequency  # 秒
        self.timer = self.create_timer(timer_period, self.publish_message)
        
        self.get_logger().info(
            f'高级PlatformStatus测试发布节点已启动\n'
            f'- 配置文件: {config_file_path}\n'
            f'- 发布频率: {publish_frequency} Hz\n'
            f'- 发送次数限制: {"无限制" if send_count == -1 else send_count}'
        )

    def load_config(self):
        """加载配置文件"""
        try:
            with open(self.config_file_path, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
                return config['PlatformStatus']
        except FileNotFoundError:
            self.get_logger().error(f'配置文件不存在: {self.config_file_path}')
            sys.exit(1)
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML解析错误: {e}')
            sys.exit(1)
        except KeyError:
            self.get_logger().error('配置文件格式错误，缺少PlatformStatus键')
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {e}')
            sys.exit(1)

    def publish_message(self):
        """发布PlatformStatus消息"""
        # 每次发布前都重新加载配置文件
        config = self.load_config()
        
        msg = PlatformStatus()
        
        # 设置头部信息
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 根据配置文件设置各个字段
        msg.msg_id = config['msg_id']
        msg.timestamp = int(datetime.now().timestamp() * 1000000)  # 使用当前时间戳（微秒）
        msg.msg_from = config['msg_from']
        msg.ctrl_mode = config['ctrl_mode']
        msg.run_mode = config['run_mode']
        msg.estop_state = config['estop_state']
        msg.warn_level = config['warn_level']
        msg.platform_state = config['platform_state']
        msg.parking_state = config['parking_state']
        msg.power_max = config['power_max']
        msg.speed_max = config['speed_max']
        msg.speed_current = config['speed_current']
        msg.current_lon = config['current_lon']
        msg.current_lat = config['current_lat']
        msg.current_high = config['current_high']  # 新增字段
        msg.steer_angle = config['steer_angle']
        msg.steer_mode = config['steer_mode']  # 新增字段
        msg.steer_faultcode = config['steer_faultcode']  # 新增字段
        msg.steer_current = config['steer_current']  # 新增字段
        msg.brake_pressure = config['brake_pressure']
        msg.brake_status = config['brake_status']  # 新增字段
        msg.brake_fault_level = config['brake_fault_level']  # 新增字段
        
        # UCM相关字段
        msg.ucmfault_lf = config['ucmfault_lf']
        msg.ucmfault_lb = config['ucmfault_lb']
        msg.ucmfault_rf = config['ucmfault_rf']
        msg.ucmfault_rb = config['ucmfault_rb']
        
        msg.ucmfaultcode_lf = config['ucmfaultcode_lf']
        msg.ucmfaultcode_lb = config['ucmfaultcode_lb']
        msg.ucmfaultcode_rf = config['ucmfaultcode_rf']
        msg.ucmfaultcode_rb = config['ucmfaultcode_rb']
        
        msg.ucmctemp_lf = config['ucmctemp_lf']
        msg.ucmctemp_lb = config['ucmctemp_lb']
        msg.ucmctemp_rf = config['ucmctemp_rf']
        msg.ucmctemp_rb = config['ucmctemp_rb']
        
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


def main(argv=sys.argv[1:]):  # Changed to accept argv parameter
    parser = argparse.ArgumentParser(description='高级PlatformStatus消息发布器')
    parser.add_argument(
        '-c', '--config', 
        default='src/udp_ros_bridge/test_config.yaml',  # Changed default to relative path
        help='配置文件路径 (默认: src/udp_ros_bridge/test_config.yaml)'
    )
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
    
    args = parser.parse_args(argv)  # Parse the passed arguments
    
    rclpy.init(args=[sys.argv[0]] + argv)  # Pass the correct format to rclpy.init()
    
    # 创建并运行高级测试发布者节点
    publisher_node = AdvancedPlatformStatusPublisher(
        config_file_path=args.config,
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