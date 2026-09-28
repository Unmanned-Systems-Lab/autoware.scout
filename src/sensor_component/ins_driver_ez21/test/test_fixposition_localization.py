import time
import unittest

from fixposition_driver_msgs.msg import FpaOdomstatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ins_driver_ez21.ins_localization_bridge import InsLocalizationBridgeNode


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FixpositionLocalizationTest(unittest.TestCase):
    def make_bridge(self, calibrated, aligned):
        rclpy.init(args=[
            "--ros-args",
            "-p", "fixposition_status_topic:=/fixposition/fpa/odomstatus",
            "-p", f"fixposition_extrinsics_calibrated:={str(calibrated).lower()}",
            "-p", f"fixposition_map_aligned:={str(aligned).lower()}",
            "-p", "publish_tf:=false",
        ])
        bridge = InsLocalizationBridgeNode()
        bridge.ros_publisher = bridge.publisher
        bridge.publisher = CapturePublisher()
        bridge.acceleration_publisher = CapturePublisher()
        bridge.pose_publisher = CapturePublisher()
        bridge.localization_init_state_publisher = CapturePublisher()
        return bridge

    def odometry(self):
        msg = Odometry()
        msg.header.frame_id = "FP_ENU0"
        msg.child_frame_id = "FP_POI"
        msg.pose.pose.orientation.w = 1.0
        return msg

    def valid_status(self):
        status = FpaOdomstatus()
        status.init_status = 2
        status.gnss1_status = 8
        return status

    def close_bridge(self, bridge):
        bridge.destroy_node()
        rclpy.shutdown()

    def test_unconfirmed_geometry_and_map_block_odom(self):
        bridge = self.make_bridge(False, False)
        try:
            bridge._on_fixposition_status(self.valid_status())
            bridge._on_odometry(self.odometry())
            self.assertEqual(len(bridge.publisher.messages), 0)
        finally:
            self.close_bridge(bridge)

    def test_invalid_or_stale_fixposition_state_blocks_odom(self):
        bridge = self.make_bridge(True, True)
        try:
            msg = self.odometry()
            bridge._on_odometry(msg)
            self.assertEqual(len(bridge.publisher.messages), 0)

            status = FpaOdomstatus()
            bridge._on_fixposition_status(status)
            bridge._on_odometry(msg)
            self.assertEqual(len(bridge.publisher.messages), 0)

            bridge._on_fixposition_status(self.valid_status())
            bridge.fixposition_status_time = time.monotonic() - 1.0
            bridge._on_odometry(msg)
            self.assertEqual(len(bridge.publisher.messages), 0)
        finally:
            self.close_bridge(bridge)

    def test_valid_state_forwards_then_revokes_initialization(self):
        bridge = self.make_bridge(True, True)
        try:
            bridge._on_fixposition_status(self.valid_status())
            bridge._on_odometry(self.odometry())
            self.assertEqual(len(bridge.publisher.messages), 1)
            self.assertTrue(bridge.localization_initialized)

            bridge._on_fixposition_status(FpaOdomstatus())
            self.assertFalse(bridge.localization_initialized)
            self.assertEqual(bridge.localization_init_state_publisher.messages[-1].state, 1)
            bridge._on_odometry(self.odometry())
            self.assertEqual(len(bridge.publisher.messages), 1)
        finally:
            self.close_bridge(bridge)

    def test_best_effort_status_and_odometry_are_received(self):
        bridge = self.make_bridge(True, True)
        probe = Node("fixposition_localization_probe", use_global_arguments=False)
        status_pub = probe.create_publisher(
            FpaOdomstatus, "/fixposition/fpa/odomstatus", qos_profile_sensor_data
        )
        odom_pub = probe.create_publisher(
            Odometry, "/odom_ins_at_base_link", qos_profile_sensor_data
        )
        received = []
        probe.create_subscription(
            Odometry, "/localization/kinematic_state", received.append, 10
        )
        # Use the actual ROS publisher for this transport test.
        bridge.publisher = bridge.ros_publisher
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(probe)
        try:
            deadline = time.monotonic() + 5.0
            while not received and time.monotonic() < deadline:
                status_pub.publish(self.valid_status())
                odom = self.odometry()
                odom.header.stamp = probe.get_clock().now().to_msg()
                odom_pub.publish(odom)
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(received, "best-effort Fixposition input did not reach bridge")
            self.assertEqual(received[-1].header.frame_id, "map")
        finally:
            executor.shutdown()
            probe.destroy_node()
            self.close_bridge(bridge)


if __name__ == "__main__":
    unittest.main()
