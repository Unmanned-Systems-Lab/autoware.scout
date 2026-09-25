import time
import unittest

from ins_driver_ez21.ins_localization_bridge import InsLocalizationBridgeNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FixpositionBridgeTest(unittest.TestCase):
    def test_best_effort_odometry_and_lever_arm(self):
        rclpy.init(args=[
            "--ros-args", "-p", "input_odom_topic:=/test/fp_odom",
            "-p", "output_odom_topic:=/test/base_odom", "-p", "publish_tf:=false",
            "-p", "base_link_to_ins_x:=0.4495158179045505",
            "-p", "base_link_to_ins_z:=0.65899",
        ])
        bridge = InsLocalizationBridgeNode()
        probe = Node("scout_localization_test", use_global_arguments=False)
        received = []
        publisher = probe.create_publisher(Odometry, "/test/fp_odom", qos_profile_sensor_data)
        subscription = probe.create_subscription(Odometry, "/test/base_odom", received.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(probe)
        try:
            message = Odometry()
            message.header.frame_id, message.child_frame_id = "FP_ENU0", "FP_POI"
            message.pose.pose.position.x, message.pose.pose.position.z = 10.0, 1.0
            message.pose.pose.orientation.w = 1.0
            message.twist.twist.linear.x, message.twist.twist.angular.z = 1.0, 0.2
            deadline = time.monotonic() + 8.0
            while not received and time.monotonic() < deadline:
                message.header.stamp = probe.get_clock().now().to_msg()
                publisher.publish(message)
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(received, "Fixposition best-effort odometry was not received")
            output = received[-1]
            self.assertEqual((output.header.frame_id, output.child_frame_id), ("map", "base_link"))
            self.assertAlmostEqual(output.pose.pose.position.x, 10 - 0.4495158179045505)
            self.assertAlmostEqual(output.pose.pose.position.z, 1 - 0.65899)
            self.assertAlmostEqual(output.twist.twist.linear.y, -0.2 * 0.4495158179045505)
        finally:
            executor.shutdown()
            probe.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
