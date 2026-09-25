import math
import time
import unittest

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand
from autoware_vehicle_msgs.srv import ControlModeCommand
from scout_msgs.msg import ScoutStatus
import rclpy

from scout_autoware_interface.kinematics import command_twist, equivalent_steering
from scout_autoware_interface.node import ScoutInterface


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class InterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ScoutInterface()
        for name in ("cmd_pub", "mode_pub", "velocity_pub", "steering_pub", "gear_pub"):
            setattr(self.node, name, Publisher())

    def tearDown(self):
        self.node.destroy_node()

    def ready(self, speed=0.5):
        status = ScoutStatus()
        status.control_mode = 1
        status.linear_velocity, status.angular_velocity = speed, 0.2
        self.node.on_status(status)
        request = ControlModeCommand.Request()
        request.mode = request.AUTONOMOUS
        response = self.node.on_mode(request, ControlModeCommand.Response())
        self.assertTrue(response.success)
        gear = GearCommand()
        gear.command = gear.DRIVE if speed >= 0 else gear.REVERSE
        self.node.on_gear(gear)
        control = Control()
        control.longitudinal.velocity = speed
        control.lateral.steering_tire_angle = 0.2
        self.node.on_command(control)

    def test_command_uses_wheelbase_and_limits(self):
        v, w = command_twist(0.5, 0.2, 0.498, 1.0, 0.8, 0.7)
        self.assertAlmostEqual(w, v * math.tan(0.2) / 0.498)
        self.assertEqual(command_twist(float("nan"), 0.2, 0.498, 1, 0.8, 0.7), (0, 0))
        self.assertEqual(command_twist(5, 1.5, 0.498, 1, 0.8, 0.7), (1, 0.8))
        self.assertAlmostEqual(equivalent_steering(-0.5, -w, 0.498), 0.2)

    def test_no_feedback_does_not_claim_ready(self):
        self.node.on_timer()
        self.assertEqual(self.node.mode_pub.messages[-1].mode, ControlModeReport.NOT_READY)
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)

    def test_stale_command_and_feedback_stop(self):
        self.ready()
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0.5)
        self.node.command_time = time.monotonic() - 1
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)
        self.node.status_time = time.monotonic() - 1
        self.node.on_timer()
        self.assertEqual(self.node.mode_pub.messages[-1].mode, ControlModeReport.NOT_READY)
        self.node.on_status(self.node.status)
        self.node.command_time = self.node.gear_time = time.monotonic()
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)
        self.assertFalse(self.node.autonomous_requested)

    def test_remote_takeover_stops(self):
        self.ready()
        self.node.status.control_mode = 3
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)
        self.assertEqual(self.node.mode_pub.messages[-1].mode, ControlModeReport.MANUAL)
        self.node.status.control_mode = 1
        self.node.on_status(self.node.status)
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)
        self.assertFalse(self.node.autonomous_requested)

    def test_feedback_axes_and_reverse_steering(self):
        self.ready(-0.5)
        velocity = self.node.velocity_pub.messages[-1]
        self.assertAlmostEqual(velocity.heading_rate, 0.2, places=6)
        self.assertAlmostEqual(velocity.lateral_velocity, -0.249 * 0.2, places=6)
        self.assertLess(self.node.steering_pub.messages[-1].steering_tire_angle, 0)
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, -0.5)

    def test_wrong_gear_and_manual_request_stop(self):
        self.ready()
        self.node.gear = GearCommand.REVERSE
        self.node.on_timer()
        self.assertEqual(self.node.cmd_pub.messages[-1].linear.x, 0)
        request = ControlModeCommand.Request()
        request.mode = request.MANUAL
        self.node.on_mode(request, ControlModeCommand.Response())
        self.assertFalse(self.node.autonomous_requested)


if __name__ == "__main__":
    unittest.main()
