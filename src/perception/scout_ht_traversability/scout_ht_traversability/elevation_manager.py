"""Supervise the unchanged ELE global mapper with coordinate-bound persistence."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fixposition_driver_msgs.msg import FpaOdomstatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

from .elevation_session import identity_for_origin, load_mapping_parameters, prepare_session


class ElevationManager(Node):
    def __init__(self, **kwargs):
        super().__init__("scout_global_elevation_manager", **kwargs)
        self.declare_parameter("storage_root", os.path.expanduser("~/autoware_data/scout_elevation_maps"))
        self.parameters, self.contract = load_mapping_parameters()
        self.parameters["use_sim_time"] = self.get_parameter("use_sim_time").value
        self.storage_root = self.get_parameter("storage_root").value
        self.origin = None
        self.init_status = None
        self.status_received = -float("inf")
        self.process = None
        self.active_key = None
        self.database = None
        self.retry_after = 0.0
        self.last_state = None
        self.session_pub = self.create_publisher(
            String, "/scout/elevation/session", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(DiagnosticArray, "/scout/elevation/status", 1)
        self.tf_sub = self.create_subscription(
            TFMessage, "/fixposition/tf_static", self.on_tf, qos_profile_sensor_data)
        self.fix_sub = self.create_subscription(
            FpaOdomstatus, "/fixposition/fpa/odomstatus", self.on_fix, qos_profile_sensor_data)
        self.timer = self.create_timer(0.25, self.tick)

    def on_tf(self, message):
        for tf in message.transforms:
            if tf.header.frame_id.strip("/") == "FP_ECEF" and tf.child_frame_id.strip("/") == "FP_ENU0":
                p = tf.transform.translation
                self.origin = (p.x, p.y, p.z)

    def on_fix(self, message):
        self.init_status = message.init_status
        self.status_received = time.monotonic()
        if message.init_status != 2:
            self.origin = None

    def publish_status(self, state, error=None):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "scout_global_elevation/session"
        status.message = state
        status.level = DiagnosticStatus.OK if state == "MAPPING" else DiagnosticStatus.WARN
        data = {"session_id": self.active_key or "", "database": str(self.database or ""),
                "resume_from_disk": "true", "crop_topic": self.parameters["global_map_local_topic"]}
        if error:
            data["error"] = error
        status.values = [KeyValue(key=k, value=v) for k, v in data.items()]
        msg.status = [status]
        self.status_pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f"Global elevation: {state}; database={self.database}; {error or ''}")
            self.last_state = state

    def stop_mapper(self):
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=18)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                self.get_logger().error("Mapper forced to stop; SQLite recovery will run on resume")

    def tick(self):
        if self.init_status != 2 or time.monotonic() - self.status_received > 3 or self.origin is None:
            if self.process is not None:
                self.session_pub.publish(String(data="unavailable"))
                self.stop_mapper()
                self.active_key = None
            self.publish_status("WAITING_GLOBAL_LOCALIZATION")
            return
        try:
            identity = identity_for_origin(self.origin, self.contract)
            key, directory, database = prepare_session(self.storage_root, identity)
            if self.active_key != key:
                self.session_pub.publish(String(data="unavailable"))
                self.stop_mapper()
                self.active_key, self.database = key, database
                self.retry_after = 0.0
            if self.process is not None and self.process.poll() is not None:
                code = self.process.returncode
                self.process = None
                self.retry_after = time.monotonic() + 5
                self.session_pub.publish(String(data="unavailable"))
                self.publish_status("MAPPER_EXITED", str(code))
                return
            if self.process is None:
                if time.monotonic() < self.retry_after:
                    return
                parameters = dict(self.parameters)
                parameters["global_map_storage_path"] = str(database)
                filename = directory / "mapping.params.yaml"
                filename.write_text(yaml.safe_dump(
                    {"/elevation_mapping_node": {"ros__parameters": parameters}}, sort_keys=False))
                executable = Path(get_package_prefix("elevation_mapping_cupy")) / "lib/elevation_mapping_cupy/elevation_mapping_node.py"
                self.process = subprocess.Popen(
                    [sys.executable, str(executable), "--ros-args", "--params-file", str(filename)],
                    start_new_session=True)
                self.session_pub.publish(String(data=key))
                self.get_logger().info(f"ELE mapper started pid={self.process.pid}, session={key}, resume=true")
            self.publish_status("MAPPING")
        except (ValueError, OSError, KeyError) as error:
            self.session_pub.publish(String(data="unavailable"))
            self.stop_mapper()
            self.publish_status("SESSION_ERROR", str(error))


def main(args=None):
    rclpy.init(args=args)
    node = ElevationManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_mapper()
        node.destroy_node()
        rclpy.try_shutdown()
