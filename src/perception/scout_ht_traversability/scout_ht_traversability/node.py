"""Observation-only fusion node. No motion publishers, services or remappings."""
from array import array
from copy import deepcopy
import math
import time

from autoware_map_msgs.msg import MapProjectorInfo
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .codec import costmap_geometry, decode_terrain, planar_pose
from .core import Policy, Pose2, Reason, fresh, fuse_local_window


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class FusionNode(Node):
    def __init__(self, **kwargs):
        super().__init__("scout_costmap_ht_fusion", **kwargs)
        defaults = {
            "base_costmap_topic": "/planning/scenario_planning/parking/costmap_generator/occupancy_grid",
            "ht_topic": "/scout/ht/raw",
            "odometry_topic": "/localization/kinematic_state",
            "robot_frame": "",
            "output_topic": "/scout/traversability/fused_costmap",
            "obstacle_threshold": 100, "ht_threshold": 0.15,
            "heading_offset": 0.0, "front_reference_offset": 0.4825,
            "near_robot_distance": 0.05, "channel_order": list(range(8)),
            "unobserved_is_free": False,
            "base_timeout": 1.0, "ht_timeout": 1.0, "odometry_timeout": 0.5,
            "max_stamp_skew": 0.75, "update_rate": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        c = self.config
        self.policy = Policy(c["obstacle_threshold"], c["ht_threshold"], c["heading_offset"],
                             c["front_reference_offset"], c["near_robot_distance"],
                             tuple(c["channel_order"]), c["unobserved_is_free"])
        for key in ("base_timeout", "ht_timeout", "odometry_timeout", "max_stamp_skew", "update_rate"):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        if c["update_rate"] > 50:
            raise ValueError("update_rate must not exceed 50 Hz")
        for key in ("base_costmap_topic", "ht_topic", "odometry_topic", "output_topic"):
            if not c[key].startswith("/"):
                raise ValueError(key + " must be an absolute topic")
        if c["robot_frame"].startswith("/"):
            raise ValueError("robot_frame must be a relative TF frame name")
        self.required_inputs = ("base", "ht") if c["robot_frame"] else ("base", "ht", "odometry")
        self.inputs = {}
        self.last_base = None
        self.errors = {}
        self.projector = None
        self.elevation_session = None
        self.epoch_stamp = 0.0
        self.last_time = None
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.tf, self)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.output = self.create_publisher(OccupancyGrid, c["output_topic"], 1)
        self.reasons = self.create_publisher(OccupancyGrid, "~/reason_map", 1)
        self.directions = self.create_publisher(OccupancyGrid, "~/selected_direction", 1)
        if self.resolve_topic_name(c["base_costmap_topic"]) in {
                self.output.topic_name, self.reasons.topic_name, self.directions.topic_name}:
            raise ValueError("Fusion output must not feed its own base costmap input")
        self.diagnostics = self.create_publisher(DiagnosticArray, "/scout/traversability/status", 1)
        self.subscriptions_ = [
            self.create_subscription(OccupancyGrid, c["base_costmap_topic"], self.on_base, sensor_qos),
            self.create_subscription(GridMap, c["ht_topic"], self.on_ht, sensor_qos),
            self.create_subscription(MapProjectorInfo, "/map/map_projector_info", self.on_projector,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(String, "/scout/elevation/session", self.on_elevation_session,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        if not c["robot_frame"]:
            self.subscriptions_.append(self.create_subscription(
                Odometry, c["odometry_topic"], self.on_odom, sensor_qos))
        # Geometry, channel meaning and source topics form one snapshot contract.
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason="Fusion parameters are startup-only; restart this node"))
        self.timer = self.create_timer(1 / c["update_rate"], self.tick)
        self.get_logger().info("Observation only: publishing a separate costmap; no planner or control changes")

    def store(self, key, message, parser):
        try:
            parsed = parser(message)
            if seconds(message.header.stamp) < self.epoch_stamp:
                raise ValueError("Observation predates the map/clock reset")
            self.inputs[key] = (message, parsed, time.monotonic())
            self.errors.pop(key, None)
        except (ValueError, TypeError, OverflowError) as error:
            self.inputs.pop(key, None)
            self.errors[key] = str(error)

    def on_base(self, message):
        self.store("base", message, costmap_geometry)
        if "base" in self.inputs:
            self.last_base = self.inputs["base"]

    def on_ht(self, message):
        self.store("ht", message, decode_terrain)

    def on_odom(self, message):
        def validate(msg):
            p = msg.pose.pose.position
            if not msg.header.frame_id or not np.isfinite([p.x, p.y, p.z]).all():
                raise ValueError("Invalid odometry frame/position")
            return (p.x, p.y)
        self.store("odometry", message, validate)

    def on_projector(self, message):
        if self.projector is not None and message != self.projector:
            self.inputs.clear()
            self.errors.clear()
            self.epoch_stamp = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn("Map projection changed; discarding all cached inputs")
        self.projector = message

    def on_elevation_session(self, message):
        if message.data != self.elevation_session:
            self.elevation_session = message.data
            self.inputs.clear()
            self.errors.clear()
            self.epoch_stamp = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn("Global elevation session changed; waiting for new observations")
            self.tick()

    def transform(self, target, source, stamp):
        if target == source:
            return Pose2()
        transform = self.tf.lookup_transform(target, source, Time.from_msg(stamp),
                                             timeout=Duration(seconds=0)).transform
        return planar_pose(transform.translation, transform.rotation)

    def tick(self):
        now = self.get_clock().now()
        current = now.nanoseconds * 1e-9
        if self.last_time is not None and current < self.last_time:
            self.inputs.clear()
            self.errors.clear()
            self.epoch_stamp = current
        self.last_time = current
        started = time.monotonic()
        state, code = "READY", Reason.PASS
        if self.elevation_session == "unavailable":
            state, code = "WAITING_ELEVATION_SESSION", Reason.WAITING
        for key in self.required_inputs if code == Reason.PASS else ():
            if key in self.errors:
                state, code = f"INVALID_{key.upper()}: {self.errors[key]}", Reason.INVALID_INPUT
                break
            if key not in self.inputs:
                state, code = "WAITING_" + key.upper(), Reason.WAITING
                break
            message, _, receipt = self.inputs[key]
            if not fresh(seconds(message.header.stamp), receipt, current, started,
                         self.config[key + "_timeout"]):
                state, code = "STALE_" + key.upper(), Reason.STALE
                break
        if code == Reason.PASS:
            stamps = [seconds(self.inputs[k][0].header.stamp) for k in self.required_inputs]
            if max(stamps) - min(stamps) > self.config["max_stamp_skew"]:
                state, code = "STAMP_SKEW", Reason.STALE
        # Keep only the last valid geometry for publishing an all-black invalid
        # result; never leave a previously white image visible after bad input.
        template = self.inputs.get("base", self.last_base)
        if template is not None:
            base, geometry, _ = template
            if code == Reason.PASS:
                try:
                    ht_msg, ht, _ = self.inputs["ht"]
                    if self.config["robot_frame"]:
                        robot = self.transform(base.header.frame_id, self.config["robot_frame"],
                                               base.header.stamp).apply(0.0, 0.0)
                    else:
                        odom, robot, _ = self.inputs["odometry"]
                        robot = self.transform(base.header.frame_id, odom.header.frame_id,
                                               odom.header.stamp).apply(*robot)
                    tf = self.transform(ht_msg.header.frame_id, base.header.frame_id,
                                        ht_msg.header.stamp)
                    values = np.asarray(base.data, dtype=np.int16).reshape(geometry.height, geometry.width)
                    result, reasons, selected = fuse_local_window(
                        values, geometry, ht, robot, self.policy, tf)
                except TransformException as error:
                    state, code = "TF_ERROR: " + str(error), Reason.TF_ERROR
                except (ValueError, TypeError, OverflowError) as error:
                    state, code = "INVALID_INPUT: " + str(error), Reason.INVALID_INPUT
            if code != Reason.PASS:
                shape = (geometry.height, geometry.width)
                result = np.full(shape, 100, dtype=np.int8)
                reasons = np.full(shape, code, dtype=np.int8)
                selected = np.full(shape, -1, dtype=np.int8)
            output = OccupancyGrid()
            output.header = deepcopy(base.header)
            # READY retains the oldest observation timestamp, never laundering old data.
            output.header.stamp = (Time(seconds=min(stamps)).to_msg()
                                   if code == Reason.PASS else now.to_msg())
            output.info = deepcopy(base.info)
            for pub, data in ((self.output, result), (self.reasons, reasons),
                              (self.directions, selected)):
                payload = array("b")
                payload.frombytes(np.ascontiguousarray(data, dtype=np.int8).tobytes())
                output.data = payload
                pub.publish(output)
            passed = int(np.count_nonzero(result == 0))
            prior_free = int(np.count_nonzero(reasons == Reason.PRIOR_FREE))
        else:
            passed = 0
            prior_free = 0
        status = DiagnosticStatus()
        status.name = "scout_ht_traversability/fusion"
        status.hardware_id = "observation_only"
        status.level = DiagnosticStatus.OK if code == Reason.PASS else DiagnosticStatus.WARN
        status.message = state
        metrics = {"mode": "observe", "base_topic": self.config["base_costmap_topic"],
                   "obstacle_threshold": self.policy.obstacle_threshold,
                   "unobserved_is_free": self.policy.unobserved_is_free,
                   "prior_free_cells": prior_free,
                   "white_cells": passed, "compute_ms": round((time.monotonic() - started) * 1000, 3)}
        for key, (message, _, _) in self.inputs.items():
            metrics[key + "_age_s"] = round(current - seconds(message.header.stamp), 3)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in metrics.items()]
        self.diagnostics.publish(DiagnosticArray(header=deepcopy(status_header(now)), status=[status]))


def status_header(now):
    from std_msgs.msg import Header
    return Header(stamp=now.to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
