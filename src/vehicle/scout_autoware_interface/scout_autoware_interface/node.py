import math
import signal
import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scout_msgs.msg import ScoutStatus

from .kinematics import command_twist, equivalent_steering


class ScoutInterface(Node):
    def __init__(self):
        super().__init__("scout_autoware_interface")
        defaults = {
            "wheelbase": 0.498, "max_speed": 1.0, "max_yaw_rate": 0.8,
            "max_steering": 0.7, "command_timeout": 0.3, "status_timeout": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.limits = {name: float(self.get_parameter(name).value) for name in defaults}
        if any(not math.isfinite(v) or v <= 0 for v in self.limits.values()):
            raise ValueError("All Scout limits must be finite and positive")
        if self.limits["max_steering"] >= math.pi / 2:
            raise ValueError("max_steering must be smaller than pi/2")
        self.command = None
        self.status = None
        self.command_time = self.status_time = self.gear_time = -math.inf
        self.gear = GearCommand.PARK
        self.autonomous_requested = False
        self.cmd_pub = self.create_publisher(Twist, "/scout/cmd_vel", 1)
        self.velocity_pub = self.create_publisher(VelocityReport, "/vehicle/status/velocity_status", 10)
        self.steering_pub = self.create_publisher(SteeringReport, "/vehicle/status/steering_status", 10)
        self.gear_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 10)
        self.mode_pub = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 10)
        self.create_subscription(Control, "/control/command/control_cmd", self.on_command, 1)
        self.create_subscription(GearCommand, "/control/command/gear_cmd", self.on_gear, 1)
        self.create_subscription(ScoutStatus, "/scout/status", self.on_status, 10)
        self.create_service(ControlModeCommand, "/control/control_mode_request", self.on_mode)
        self.create_timer(0.02, self.on_timer)

    def on_command(self, msg):
        self.command = msg
        self.command_time = time.monotonic()

    def on_gear(self, msg):
        self.gear = msg.command
        self.gear_time = time.monotonic()

    def status_valid(self):
        return (
            self.status is not None
            and time.monotonic() - self.status_time < self.limits["status_timeout"]
            and self.status.error_code == 0 and self.status.vehicle_state == 0
            and math.isfinite(self.status.linear_velocity)
            and math.isfinite(self.status.angular_velocity)
        )

    def on_mode(self, request, response):
        if request.mode == ControlModeCommand.Request.MANUAL:
            self.autonomous_requested = False
            self.cmd_pub.publish(Twist())
            response.success = True
        elif request.mode == ControlModeCommand.Request.AUTONOMOUS:
            response.success = self.status_valid() and self.status.control_mode == 1
            self.autonomous_requested = response.success
        else:
            response.success = False
        return response

    def on_status(self, msg):
        self.status, self.status_time = msg, time.monotonic()
        if not self.status_valid() or msg.control_mode != 1:
            self.autonomous_requested = False
        if not self.status_valid():
            return
        stamp = self.get_clock().now().to_msg()
        velocity = VelocityReport()
        velocity.header.stamp, velocity.header.frame_id = stamp, "base_link"
        velocity.longitudinal_velocity = float(msg.linear_velocity)
        velocity.heading_rate = float(msg.angular_velocity)
        # Scout reports velocity at chassis center; Autoware base_link is at the rear axle.
        velocity.lateral_velocity = -0.5 * self.limits["wheelbase"] * msg.angular_velocity
        self.velocity_pub.publish(velocity)
        steering = SteeringReport()
        steering.stamp = stamp
        steering.steering_tire_angle = equivalent_steering(
            msg.linear_velocity, msg.angular_velocity, self.limits["wheelbase"])
        self.steering_pub.publish(steering)
        gear = GearReport()
        gear.stamp = stamp
        # Scout has no gearbox feedback. Report travel direction, or the stopped command.
        gear.report = (GearReport.DRIVE if msg.linear_velocity > 0.01 else
                       GearReport.REVERSE if msg.linear_velocity < -0.01 else self.gear)
        self.gear_pub.publish(gear)

    def on_timer(self):
        now = time.monotonic()
        valid = self.status_valid()
        if not valid or self.status.control_mode != 1:
            self.autonomous_requested = False
        automatic = valid and self.status.control_mode == 1 and self.autonomous_requested
        mode = ControlModeReport()
        mode.stamp = self.get_clock().now().to_msg()
        mode.mode = (ControlModeReport.NOT_READY if not valid else
                     ControlModeReport.AUTONOMOUS if automatic else ControlModeReport.MANUAL)
        self.mode_pub.publish(mode)
        command = Twist()
        if (automatic and self.command is not None
                and now - self.command_time < self.limits["command_timeout"]
                and now - self.gear_time < self.limits["status_timeout"]):
            velocity = float(self.command.longitudinal.velocity)
            direction_matches = (
                self.gear == GearCommand.DRIVE and velocity >= 0
                or self.gear == GearCommand.REVERSE and velocity <= 0)
            if direction_matches:
                command.linear.x, command.angular.z = command_twist(
                    velocity, float(self.command.lateral.steering_tire_angle),
                    self.limits["wheelbase"], self.limits["max_speed"],
                    self.limits["max_yaw_rate"], self.limits["max_steering"])
        self.cmd_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ScoutInterface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward SIGINT after the process group already received it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
