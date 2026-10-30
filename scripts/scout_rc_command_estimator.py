#!/usr/bin/env python3
"""Publish a configurable, receive-time estimate of RC velocity commands.

This is a read-only ROS node. The chassis itself handles RC commands over CAN;
this node never sends a command to the robot.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from scout_msgs.msg import ScoutRCState, ScoutStatus


class RCCommandEstimator(Node):
    def __init__(self):
        super().__init__('scout_rc_command_estimator')
        self.declare_parameter('max_linear_mps', 1.29)
        self.declare_parameter('max_angular_radps', 0.49)
        self.declare_parameter('stick_full_scale', 100.0)
        self.declare_parameter('deadzone_raw', 3.0)
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('status_timeout_s', 0.5)
        self.declare_parameter('rc_mode_value', 3)
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_radps').value)
        self.full_scale = float(self.get_parameter('stick_full_scale').value)
        self.deadzone = float(self.get_parameter('deadzone_raw').value)
        self.linear_sign = float(self.get_parameter('linear_sign').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.status_timeout = float(self.get_parameter('status_timeout_s').value)
        self.rc_mode = int(self.get_parameter('rc_mode_value').value)
        if self.full_scale <= 0 or self.deadzone < 0 or self.status_timeout <= 0:
            raise ValueError('Invalid RC conversion parameters')
        self.last_status_time = None
        self.control_mode = None
        self.pub = self.create_publisher(TwistStamped, '/scout/rc_cmd_vel_estimated', 50)
        self.create_subscription(ScoutStatus, '/scout/status', self.on_status, 50)
        self.create_subscription(ScoutRCState, '/scout/rc_state', self.on_rc, 50)

    def on_status(self, message):
        self.control_mode = message.control_mode
        self.last_status_time = self.get_clock().now()

    def axis(self, raw, limit, sign):
        if abs(raw) <= self.deadzone:
            return 0.0
        return max(-limit, min(limit, sign * raw * limit / self.full_scale))

    def on_rc(self, message):
        now = self.get_clock().now()
        if (self.control_mode != self.rc_mode or self.last_status_time is None or
                (now - self.last_status_time).nanoseconds > self.status_timeout * 1e9):
            return
        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = 'scout_chassis_link'
        twist.twist.linear.x = self.axis(message.stick_left_v, self.max_linear,
                                         self.linear_sign)
        twist.twist.angular.z = self.axis(message.stick_right_h, self.max_angular,
                                          self.angular_sign)
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = RCCommandEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
