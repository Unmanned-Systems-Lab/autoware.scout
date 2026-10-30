from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "src/perception/scout_ht_traversability"))
from scout_ht_traversability.core import (Geometry, Policy, Pose2, Reason, Terrain,
                                          fresh, fuse, fuse_local_window, nearest_direction)
from scout_ht_traversability.codec import costmap_geometry, decode_terrain


def terrain(probability=0.9, shape=(40, 40), resolution=0.1):
    return Terrain(np.full((8,) + shape, probability, dtype=np.float32),
                   np.ones(shape), resolution)


def ht_message(source, order="F", start=(0, 0), offset=0):
    msg = GridMap()
    msg.header.frame_id = "map"
    msg.info.resolution = source.resolution
    rows, cols = source.valid.shape
    msg.info.length_x = rows * source.resolution
    msg.info.length_y = cols * source.resolution
    msg.info.pose.orientation.w = 1.0
    msg.outer_start_index, msg.inner_start_index = start
    layers = {f"ht_dir_{k}": source.probabilities[k] for k in range(8)}
    layers["ht_valid"] = source.valid
    if source.direction_valid is not None:
        layers.update({f"ht_dir_valid_{k}": source.direction_valid[k] for k in range(8)})
    if source.observed is not None:
        layers["ht_observed"] = source.observed
    for name, matrix in layers.items():
        array = Float32MultiArray()
        if order == "F":
            array.layout.dim = [
                MultiArrayDimension(label="column_index", size=cols, stride=rows * cols),
                MultiArrayDimension(label="row_index", size=rows, stride=rows)]
        else:
            array.layout.dim = [
                MultiArrayDimension(label="row_index", size=rows, stride=rows * cols),
                MultiArrayDimension(label="column_index", size=cols, stride=cols)]
        array.layout.data_offset = offset
        array.data = [0.0] * offset + np.roll(matrix, start, axis=(0, 1)).ravel(order=order).tolist()
        msg.layers.append(name)
        msg.data.append(array)
    return msg


class FusionTest(unittest.TestCase):
    def test_optimistic_initial_map_is_free_including_outside_ht_and_robot_cell(self):
        geometry = Geometry(81, 81, .1, Pose2(-4.05, -4.05))
        base = np.zeros((81, 81), np.int8)
        base[0, 0] = 100
        base[0, 1] = -1
        ht = terrain(probability=np.nan, shape=(20, 20))
        ht.valid[:] = 0
        ht.observed = np.zeros(ht.valid.shape, np.float32)
        policy = Policy(unobserved_is_free=True)
        result, reasons, _ = fuse_local_window(base, geometry, ht, (0, 0), policy)
        self.assertTrue((result[base == 0] == 0).all())
        self.assertTrue((reasons[base == 0] == Reason.PRIOR_FREE).all())
        self.assertEqual(result[0, 0], 100)
        self.assertEqual(result[0, 1], 100)

    def test_optimistic_prior_does_not_clear_observed_low_invalid_or_partial_edge(self):
        geometry = Geometry(1, 1, .2, Pose2(1.85, -.1))
        ht = terrain()
        ht.observed = np.zeros(ht.valid.shape, np.float32)
        ht.observed[0, 19] = 1
        policy = Policy(front_reference_offset=0., unobserved_is_free=True)
        base = np.zeros((1, 1), np.int8)
        ht.probabilities[0, 0, 19] = .01
        result, reasons, _ = fuse(base, geometry, ht, (0, 0), policy)
        self.assertEqual(result[0, 0], 100)
        self.assertEqual(reasons[0, 0], Reason.HT_LOW)
        ht.probabilities[0, 0, 19] = np.nan
        self.assertEqual(fuse(base, geometry, ht, (0, 0), policy)[1][0, 0], Reason.HT_INVALID)
        ht.valid[:] = 0  # No HT conclusion yet: retain the prior instead of inventing a block.
        self.assertEqual(fuse(base, geometry, ht, (0, 0), policy)[1][0, 0], Reason.PRIOR_FREE)
        ht.observed[:] = 0
        self.assertEqual(fuse(base, geometry, ht, (0, 0), policy)[1][0, 0], Reason.PRIOR_FREE)

    def test_optimistic_mode_requires_explicit_observation_mask(self):
        with self.assertRaisesRegex(ValueError, "observation mask"):
            fuse(np.zeros((1, 1), np.int8), Geometry(1, 1, .1), terrain(),
                 (0, 0), Policy(unobserved_is_free=True))

    def test_optimistic_window_matches_full_reference_with_rotations(self):
        rng = np.random.default_rng(76)
        for angle in (0., .4, -1.3):
            geometry = Geometry(170, 150, .1, Pose2(-8, -7, angle))
            base = rng.choice(np.array([-1, 0, 100], np.int8), (150, 170))
            ht = terrain(shape=(31, 39))
            ht.center = Pose2(.6, -.2, .3)
            ht.observed = (rng.random(ht.valid.shape) > .7).astype(np.float32)
            ht.probabilities[:] = rng.random(ht.probabilities.shape)
            policy = Policy(unobserved_is_free=True)
            transform = Pose2(.2, -.1, -.2)
            expected = fuse(base, geometry, ht, (0, 0), policy, transform)
            actual = fuse_local_window(base, geometry, ht, (0, 0), policy, transform)
            np.testing.assert_array_equal(actual[0], expected[0])
            np.testing.assert_array_equal(actual[1], expected[1])

    def test_local_window_matches_reference_with_rotations_and_vetoes(self):
        rng = np.random.default_rng(42)
        for yaw in (0.0, 0.43, -1.7):
            geometry = Geometry(213, 195, 0.1, Pose2(-10, -9, yaw))
            base = rng.choice(np.array([-1, 0, 35, 100], dtype=np.int8),
                              size=(geometry.height, geometry.width))
            ht = terrain(shape=(37, 43))
            ht.center = Pose2(0.7, -0.8, -yaw / 2)
            ht.probabilities[:] = rng.uniform(0, 1, ht.probabilities.shape)
            ht.valid[rng.random(ht.valid.shape) < 0.1] = 0
            ht.probabilities[2, 15, 15] = np.nan
            transform = Pose2(0.2, -0.4, yaw / 3)
            for robot in ((0.0, 0.0), (8.1, -7.2), (100.0, 100.0)):
                policy = Policy(near_robot_distance=0.25)
                expected = fuse(base, geometry, ht, robot, policy, transform)
                actual = fuse_local_window(base, geometry, ht, robot, policy, transform)
                np.testing.assert_array_equal(actual[0], expected[0])
                np.testing.assert_array_equal(actual[1], expected[1])
                queried = actual[2] >= 0
                np.testing.assert_array_equal(actual[2][queried], expected[2][queried])

    def test_four_centimetre_full_map_preserves_extent_and_thin_obstacles(self):
        geometry = Geometry(1750, 1750, 0.04, Pose2(-35, -35))
        base = np.zeros((1750, 1750), dtype=np.int8)
        base[:, 877] = 100
        ht = terrain(shape=(200, 200), resolution=0.04)
        result, reason, _ = fuse_local_window(base, geometry, ht, (0, 0))
        self.assertEqual(result.shape, (1750, 1750))
        self.assertTrue((result[:, 877] == 100).all())
        self.assertTrue((reason[:, 877] == Reason.BASE_BLOCKED).all())
        self.assertEqual(result[0, 0], 100)  # No HT support outside its local crop.
        self.assertGreater(np.count_nonzero(result == 0), 0)

    def setUp(self):
        self.geometry = Geometry(1, 1, 0.1, Pose2(0.5, 0.0))
        self.base = np.array([[0]], dtype=np.int8)
        self.ht = terrain()
        self.policy = Policy(front_reference_offset=0.0)

    def run_fusion(self, robot=(0, 0), **kwargs):
        return fuse(self.base, self.geometry, self.ht, robot,
                    kwargs.pop("policy", self.policy), **kwargs)

    def test_costmap_black_cannot_be_cleared_by_ht(self):
        self.base[0, 0] = 100
        result, reasons, selected = self.run_fusion()
        self.assertEqual(result[0, 0], 100)
        self.assertEqual(reasons[0, 0], Reason.BASE_BLOCKED)
        self.assertEqual(selected[0, 0], -1)

    def test_costmap_threshold_is_explicit_and_inclusive(self):
        self.base[0, 0] = 30
        self.assertEqual(self.run_fusion()[0][0, 0], 0)
        policy = Policy(obstacle_threshold=30, front_reference_offset=0.0)
        self.assertEqual(self.run_fusion(policy=policy)[1][0, 0], Reason.BASE_BLOCKED)

    def test_bearing_changes_black_to_white_without_mutating_inputs(self):
        self.ht.probabilities[4] = 0.01
        before = self.base.copy()
        self.assertEqual(self.run_fusion(robot=(1.2, 0.05))[0][0, 0], 100)
        self.assertEqual(self.run_fusion(robot=(0, 0.05))[0][0, 0], 0)
        np.testing.assert_array_equal(before, self.base)

    def test_all_eight_robot_to_target_directions(self):
        self.geometry = Geometry(1, 1, 0.1, Pose2(-0.05, -0.05))
        for k in range(8):
            a = k * math.pi / 4
            _, _, selected = self.run_fusion(robot=(-math.cos(a), -math.sin(a)))
            self.assertEqual(selected[0, 0], k)

    def test_wrap_and_ties_are_deterministic(self):
        angles = np.array([0, -0.01, 2 * math.pi - 0.01, math.pi / 8,
                           3 * math.pi / 8, -math.pi / 8 - 1e-6])
        np.testing.assert_array_equal(nearest_direction(angles), [0, 0, 0, 1, 2, 7])

    def test_unknown_and_malformed_base_values_never_pass(self):
        for value in (-1, 101):
            self.base = np.array([[value]], dtype=np.int16)
            self.assertEqual(self.run_fusion()[1][0, 0], Reason.BASE_UNKNOWN)

    def test_near_robot_does_not_get_arbitrary_direction(self):
        _, reason, selected = self.run_fusion(robot=(0.55, 0.05))
        self.assertEqual(reason[0, 0], Reason.NEAR_ROBOT)
        self.assertEqual(selected[0, 0], -1)

    def test_missing_or_out_of_range_selected_probability_is_invalid(self):
        for value in (float("nan"), -0.1, 1.1):
            self.ht.probabilities[0] = value
            self.assertEqual(self.run_fusion()[1][0, 0], Reason.HT_INVALID)

    def test_other_directions_do_not_veto_selected_direction(self):
        self.ht.probabilities[4] = np.nan
        self.assertEqual(self.run_fusion()[0][0, 0], 0)

    def test_direction_support_is_not_all_direction_intersection(self):
        self.ht.direction_valid = np.ones_like(self.ht.probabilities)
        self.ht.direction_valid[4] = 0
        self.assertEqual(self.run_fusion()[0][0, 0], 0)
        self.ht.direction_valid[0] = 0
        self.assertEqual(self.run_fusion()[1][0, 0], Reason.HT_INVALID)

    def test_reference_offset_is_applied_once(self):
        self.ht.probabilities[0] = 0.01
        self.ht.probabilities[0, 8:11, 19:21] = 0.9
        policy = Policy(front_reference_offset=0.4825)
        self.assertEqual(self.run_fusion(policy=policy)[0][0, 0], 0)
        self.assertEqual(self.run_fusion()[0][0, 0], 100)

    def test_costmap_cell_checks_more_than_just_ht_center(self):
        self.geometry = Geometry(1, 1, 0.3, Pose2(0.4, -0.1))
        self.ht.probabilities[0, 13, 18] = 0.01
        self.assertEqual(self.run_fusion()[1][0, 0], Reason.HT_LOW)

    def test_partial_ht_coverage_does_not_get_clipped_and_passed(self):
        self.geometry = Geometry(1, 1, 0.3, Pose2(1.8, 0))
        self.assertEqual(self.run_fusion()[1][0, 0], Reason.HT_INVALID)

    def test_geometry_and_tf_transform_both_bearing_and_position(self):
        self.ht.probabilities[0] = 0.01
        self.geometry = Geometry(1, 1, 0.1, Pose2(-0.05, -0.05, math.pi / 2))
        _, _, selected = self.run_fusion(robot=(-0.5, 0), base_to_ht=Pose2(0, 0, math.pi / 2))
        self.assertEqual(selected[0, 0], 2)
        self.ht.center = Pose2(0, 0, math.pi / 2)
        self.assertEqual(self.run_fusion(robot=(-0.5, 0),
                                       base_to_ht=Pose2(0, 0, math.pi / 2))[2][0, 0], 0)

    def test_nonidentity_channel_order(self):
        self.ht.probabilities[1] = 0.01
        policy = Policy(channel_order=(1, 0, 2, 3, 4, 5, 6, 7), front_reference_offset=0.0)
        self.assertEqual(self.run_fusion(policy=policy)[1][0, 0], Reason.HT_LOW)

    def test_bad_configuration_is_rejected(self):
        for params in ({"ht_threshold": float("nan")}, {"channel_order": (0,) * 8},
                       {"obstacle_threshold": 0}, {"near_robot_distance": 0.0}):
            with self.assertRaises(ValueError):
                Policy(**params)
        with self.assertRaises(ValueError):
            Geometry(0, 1, 0.1)
        with self.assertRaises(ValueError):
            self.run_fusion(robot=(np.nan, 0))

    def test_freshness_checks_header_receipt_and_future(self):
        self.assertTrue(fresh(10, 100, 10.1, 100.1, 1))
        for values in ((8, 100, 10, 100.1), (10, 98, 10.1, 100.1),
                       (12, 100, 10.1, 100.1), (0, 100, 0, 100.1)):
            self.assertFalse(fresh(*values, 1))


class CodecTest(unittest.TestCase):
    def test_row_column_layout_offset_and_circular_buffer(self):
        source = terrain(shape=(6, 10))
        source.probabilities[0] = np.arange(60).reshape(6, 10) / 60
        for order in ("F", "C"):
            decoded = decode_terrain(ht_message(source, order, start=(2, 7), offset=3))
            np.testing.assert_array_equal(decoded.probabilities, source.probabilities)

    def test_missing_probability_does_not_use_binary_or_best_layer(self):
        msg = ht_message(terrain())
        msg.layers[0] = "ht_cost_dir_0"
        with self.assertRaisesRegex(ValueError, "Missing HT layer"):
            decode_terrain(msg)

    def test_corrupt_layout_missing_support_and_tilt_are_rejected(self):
        original = ht_message(terrain())
        bad = deepcopy(original)
        bad.data[0].layout.dim[0].stride = 1
        with self.assertRaisesRegex(ValueError, "size/stride"):
            decode_terrain(bad)
        bad = deepcopy(original)
        bad.outer_start_index = 1000
        with self.assertRaisesRegex(ValueError, "buffer"):
            decode_terrain(bad)
        bad = deepcopy(original)
        bad.info.pose.orientation.x = 0.2
        with self.assertRaises(ValueError):
            decode_terrain(bad)
        bad = deepcopy(original)
        bad.layers.append("ht_dir_valid_0")
        bad.data.append(deepcopy(bad.data[0]))
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            decode_terrain(bad)

    def test_costmap_rotated_origin_and_data_size(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.width, msg.info.height, msg.info.resolution = 2, 3, 0.5
        msg.info.origin.orientation.z = math.sin(math.pi / 4)
        msg.info.origin.orientation.w = math.cos(math.pi / 4)
        msg.data = [0] * 6
        geometry = costmap_geometry(msg)
        x, y = geometry.centers()
        self.assertAlmostEqual(x[0, 0], -0.25)
        self.assertAlmostEqual(y[0, 0], 0.25)
        msg.data = [0]
        with self.assertRaisesRegex(ValueError, "length"):
            costmap_geometry(msg)


class NodeTest(unittest.TestCase):
    optimistic = False
    def setUp(self):
        import rclpy
        from rclpy.parameter import Parameter
        from scout_ht_traversability.node import FusionNode
        rclpy.init()
        self.node = FusionNode(parameter_overrides=[
            Parameter("max_stamp_skew", value=0.2),
            Parameter("base_costmap_topic", value="/scout_ht_test/base"),
            Parameter("ht_topic", value="/scout_ht_test/ht"),
            Parameter("odometry_topic", value="/scout_ht_test/odom"),
            Parameter("output_topic", value="/scout_ht_test/fused"),
            Parameter("unobserved_is_free", value=self.optimistic),
        ])
        class Recorder:
            def __init__(self):
                self.messages = []
            def publish(self, message):
                self.messages.append(deepcopy(message))
        self.node.output = Recorder()
        self.node.reasons = Recorder()
        self.node.directions = Recorder()
        self.node.diagnostics = Recorder()
        self.base = OccupancyGrid()
        self.base.header.frame_id = "map"
        self.base.info.width = self.base.info.height = 1
        self.base.info.resolution = 0.1
        self.base.info.origin.position.x = 0.5
        self.base.info.origin.orientation.w = 1.0
        self.base.data = [0]
        source = terrain()
        if self.optimistic:
            source.observed = np.zeros(source.valid.shape, np.float32)
        self.ht = ht_message(source)
        from nav_msgs.msg import Odometry
        self.odom = Odometry()
        self.odom.header.frame_id = "map"
        self.odom.pose.pose.orientation.w = 1.0

    def tearDown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()

    def feed(self):
        stamp = self.node.get_clock().now().to_msg()
        for msg in (self.base, self.ht, self.odom):
            msg.header.stamp = deepcopy(stamp)
        self.node.on_base(self.base)
        self.node.on_ht(self.ht)
        self.node.on_odom(self.odom)
        self.node.tick()

    def test_actual_ros_messages_publish_binary_result_and_no_motion(self):
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [0])
        self.assertEqual(self.node.diagnostics.messages[-1].status[0].message, "READY")
        topics = [p.topic_name for p in self.node.publishers]
        self.assertFalse(any("cmd_vel" in topic or "control_cmd" in topic for topic in topics))

    def test_stale_ht_fails_closed_then_recovers(self):
        self.feed()
        self.ht.header.stamp.sec -= 5
        self.node.on_ht(self.ht)
        self.node.tick()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.assertEqual(self.node.diagnostics.messages[-1].status[0].message, "STALE_HT")
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [0])

    def test_invalid_ht_does_not_reuse_previous_good_map(self):
        self.feed()
        self.ht.layers[0] = "ht_cost_dir_0"
        self.node.on_ht(self.ht)
        self.node.tick()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])

    def test_invalid_base_clears_previous_white_display(self):
        self.feed()
        self.base.data = []
        self.node.on_base(self.base)
        self.node.tick()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.assertIn("INVALID_BASE", self.node.diagnostics.messages[-1].status[0].message)

    def test_missing_tf_fails_closed(self):
        self.odom.header.frame_id = "missing_frame"
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.assertIn("TF_ERROR", self.node.diagnostics.messages[-1].status[0].message)

    def test_map_projection_change_discards_cached_inputs(self):
        from autoware_map_msgs.msg import MapProjectorInfo
        self.feed()
        original = MapProjectorInfo()
        self.node.on_projector(original)
        changed = deepcopy(original)
        changed.map_origin.latitude = 1.0
        self.node.on_projector(changed)
        self.node.tick()
        self.assertEqual(self.node.inputs, {})
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.node.on_ht(self.ht)
        self.assertNotIn("ht", self.node.inputs)

    def test_global_elevation_session_switch_rejects_cached_and_unavailable_data(self):
        from std_msgs.msg import String
        self.feed()
        self.node.on_elevation_session(String(data="first-origin"))
        self.assertEqual(self.node.inputs, {})
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.node.on_ht(self.ht)
        self.assertNotIn("ht", self.node.inputs)
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [0])
        self.node.on_elevation_session(String(data="unavailable"))
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        self.assertEqual(self.node.diagnostics.messages[-1].status[0].message, "WAITING_ELEVATION_SESSION")
        self.node.on_elevation_session(String(data="new-origin"))
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [0])

    def test_stamp_skew_does_not_launder_old_observation(self):
        from rclpy.time import Time
        self.feed()
        stamp = self.node.get_clock().now().nanoseconds * 1e-9 - 0.4
        self.ht.header.stamp = Time(seconds=stamp).to_msg()
        self.node.on_ht(self.ht)
        self.node.tick()
        self.assertEqual(self.node.diagnostics.messages[-1].status[0].message, "STAMP_SKEW")
        self.assertEqual(list(self.node.output.messages[-1].data), [100])

    def test_dds_inputs_to_fused_output(self):
        import time
        from nav_msgs.msg import Odometry
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        real_output = next(p for p in self.node.publishers if p.topic_name == "/scout_ht_test/fused")
        self.node.output = real_output
        probe = rclpy.create_node("scout_ht_transport_test")
        received = []
        subscription = probe.create_subscription(OccupancyGrid, "/scout_ht_test/fused",
                                                   received.append, 1)
        publishers = [probe.create_publisher(kind, topic, 1) for kind, topic in (
            (OccupancyGrid, "/scout_ht_test/base"), (GridMap, "/scout_ht_test/ht"),
            (Odometry, "/scout_ht_test/odom"))]
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(probe)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                stamp = probe.get_clock().now().to_msg()
                for publisher, message in zip(publishers, (self.base, self.ht, self.odom)):
                    message.header.stamp = deepcopy(stamp)
                    publisher.publish(message)
                executor.spin_once(timeout_sec=0.02)
                if received and list(received[-1].data) == [0]:
                    break
            self.assertTrue(received)
            self.assertEqual(list(received[-1].data), [0])
            self.assertEqual(received[-1].header.frame_id, "map")
        finally:
            executor.remove_node(probe)
            executor.remove_node(self.node)
            executor.shutdown()
            probe.destroy_node()

    def test_feedback_remapping_is_rejected(self):
        from rclpy.parameter import Parameter
        from scout_ht_traversability.node import FusionNode
        candidate = FusionNode.__new__(FusionNode)
        try:
            with self.assertRaisesRegex(ValueError, "own base costmap"):
                candidate.__init__(parameter_overrides=[
                    Parameter("base_costmap_topic", value="/scout_ht_test/base"),
                    Parameter("output_topic", value="/scout_ht_test/fused")],
                    cli_args=["--ros-args", "-r", "__node:=scout_fusion_feedback_test",
                              "-r", "/scout_ht_test/fused:=/scout_ht_test/base"])
        finally:
            candidate.destroy_node()


class OptimisticNodeTest(NodeTest):
    optimistic = True

    def test_real_low_scores_replace_prior_and_base_obstacles_always_veto(self):
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [0])
        metrics = {v.key: v.value for v in self.node.diagnostics.messages[-1].status[0].values}
        self.assertEqual(metrics["prior_free_cells"], "1")
        source = terrain(probability=.01)
        source.observed = np.ones(source.valid.shape, np.float32)
        self.ht = ht_message(source)
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])
        source.observed[:] = 0
        self.ht = ht_message(source)
        self.base.data = [100]
        self.feed()
        self.assertEqual(list(self.node.output.messages[-1].data), [100])


if __name__ == "__main__":
    unittest.main()
