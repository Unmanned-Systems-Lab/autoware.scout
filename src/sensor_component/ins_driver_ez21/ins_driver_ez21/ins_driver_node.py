import fcntl
import math
from pathlib import Path
import socket
import struct
import threading
from typing import Optional
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import yaml

from ins_driver_ez21.protocol import ALTITUDE_FRAME_IDS
from ins_driver_ez21.protocol import CAN_READ_SIZE
from ins_driver_ez21.protocol import CanFrame
from ins_driver_ez21.protocol import LOG_FORMAT_CAN
from ins_driver_ez21.protocol import LOG_FORMAT_ODOM_CSV
from ins_driver_ez21.protocol import NavigationState
from ins_driver_ez21.protocol import POSITION_FRAME_IDS
from ins_driver_ez21.protocol import decode_can_frame
from ins_driver_ez21.protocol import format_can_line
from ins_driver_ez21.protocol import geodetic_to_ecef
from ins_driver_ez21.protocol import geodetic_to_enu
from ins_driver_ez21.protocol import normalize_angle_deg
from ins_driver_ez21.protocol import normalize_log_format
from ins_driver_ez21.protocol import open_log_file
from ins_driver_ez21.protocol import quaternion_from_euler
from ins_driver_ez21.protocol import resolve_log_output_path
from ins_driver_ez21.protocol import rotate_world_to_body
from ins_driver_ez21.protocol import update_navigation_state


SIOCGIFFLAGS = 0x8913
IFF_UP = 0x1


class CanInterfaceNotReadyError(RuntimeError):
    """Raised when the requested SocketCAN interface does not exist or is down."""


class Ez21InsDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("ins_driver_ez21")

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("topic_name", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_raw_nav_sat_fix", True)
        self.declare_parameter("raw_nav_sat_fix_topic", "/sensing/ins/raw_nav_sat_fix")
        self.declare_parameter("raw_nav_sat_fix_frame_id", "ins_link")
        self.declare_parameter("map_projector_info_path", "")
        self.declare_parameter("reference_origin_active", False)
        self.declare_parameter("reference_origin_latitude_deg", 0.0)
        self.declare_parameter("reference_origin_longitude_deg", 0.0)
        self.declare_parameter("reference_origin_altitude_m", 0.0)
        self.declare_parameter("socket_timeout_sec", 0.2)
        self.declare_parameter("log_enabled", False)
        self.declare_parameter("log_path", "~/ins_driver_ez21/logs")
        self.declare_parameter("log_name", "auto")
        self.declare_parameter("log_name_format", "%y%m%d%H%M%S.txt")
        self.declare_parameter("log_format", LOG_FORMAT_CAN)

        self.can_interface = str(self.get_parameter("can_interface").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.publish_raw_nav_sat_fix = bool(
            self.get_parameter("publish_raw_nav_sat_fix").value
        )
        self.raw_nav_sat_fix_topic = str(self.get_parameter("raw_nav_sat_fix_topic").value)
        self.raw_nav_sat_fix_frame_id = str(
            self.get_parameter("raw_nav_sat_fix_frame_id").value
        )
        self.map_projector_info_path = str(
            self.get_parameter("map_projector_info_path").value
        ).strip()
        self.reference_origin_active = bool(
            self.get_parameter("reference_origin_active").value
        )
        self.reference_origin_latitude_deg = float(
            self.get_parameter("reference_origin_latitude_deg").value
        )
        self.reference_origin_longitude_deg = float(
            self.get_parameter("reference_origin_longitude_deg").value
        )
        self.reference_origin_altitude_m = float(
            self.get_parameter("reference_origin_altitude_m").value
        )
        self.socket_timeout_sec = float(self.get_parameter("socket_timeout_sec").value)
        self.log_enabled = bool(self.get_parameter("log_enabled").value)
        self.log_path_text = str(self.get_parameter("log_path").value)
        self.log_name = str(self.get_parameter("log_name").value)
        self.log_name_format = str(self.get_parameter("log_name_format").value)
        self.log_format = normalize_log_format(str(self.get_parameter("log_format").value))

        if not self.can_interface:
            raise ValueError("Parameter 'can_interface' must not be empty.")
        if self.socket_timeout_sec <= 0.0:
            raise ValueError("Parameter 'socket_timeout_sec' must be greater than zero.")
        if self.publish_raw_nav_sat_fix and not self.raw_nav_sat_fix_topic:
            raise ValueError(
                "Parameter 'raw_nav_sat_fix_topic' must not be empty when "
                "'publish_raw_nav_sat_fix' is true."
            )
        if not self.raw_nav_sat_fix_frame_id:
            self.raw_nav_sat_fix_frame_id = self.child_frame_id

        self._ensure_can_interface_is_up()

        self.publisher = self.create_publisher(Odometry, self.topic_name, 10)
        self.raw_nav_sat_fix_publisher = None
        if self.publish_raw_nav_sat_fix:
            self.raw_nav_sat_fix_publisher = self.create_publisher(
                NavSatFix, self.raw_nav_sat_fix_topic, 10
            )
        self.state = NavigationState()
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._configure_origin()
        self._published_count = 0
        self._published_raw_nav_sat_fix_count = 0
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self._log_file = None
        self._log_file_path = None

        if self.log_enabled:
            self._log_file_path = resolve_log_output_path(
                self.log_path_text, self.log_name, self.log_name_format
            )
            self._log_file = open_log_file(self._log_file_path, self.log_format)
            self.get_logger().info(
                "Log enabled (%s): %s" % (self.log_format, self._log_file_path)
            )
        else:
            self.get_logger().info("Log disabled by parameter 'log_enabled'.")

        self._socket = self._open_can_socket()
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

        self.get_logger().info(
            "Listening on %s and publishing odometry to %s"
            % (self.can_interface, self.topic_name)
        )
        if self.publish_raw_nav_sat_fix:
            self.get_logger().info(
                "Publishing raw NavSatFix to %s (frame_id=%s)"
                % (self.raw_nav_sat_fix_topic, self.raw_nav_sat_fix_frame_id)
            )
        else:
            self.get_logger().info("Raw NavSatFix publishing is disabled.")

    def _configure_origin(self) -> None:
        fixed_origin = self._load_origin_from_reference_parameters()
        if fixed_origin is not None:
            self._set_navigation_origin(fixed_origin)
            self.get_logger().info(
                "Using persisted INS reference origin: lat=%.10f lon=%.10f alt=%.3f"
                % (
                    fixed_origin[0],
                    fixed_origin[1],
                    fixed_origin[2],
                )
            )
            return

        fixed_origin = self._load_origin_from_map_projector_info()
        if fixed_origin is None:
            self.get_logger().info("Using first valid INS fix as ENU origin.")
            return

        self._set_navigation_origin(fixed_origin)
        self.get_logger().info(
            "Using fixed ENU origin from %s: lat=%.10f lon=%.10f alt=%.3f"
            % (
                self.map_projector_info_path,
                fixed_origin[0],
                fixed_origin[1],
                fixed_origin[2],
            )
        )

    def _load_origin_from_reference_parameters(self) -> Optional[Tuple[float, float, float]]:
        if not self.reference_origin_active:
            return None

        latitude = self.reference_origin_latitude_deg
        longitude = self.reference_origin_longitude_deg
        altitude = self.reference_origin_altitude_m
        if not (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and math.isfinite(altitude)
        ):
            self.get_logger().warning(
                "reference_origin_* parameters are active but invalid; falling back to other origin sources."
            )
            return None

        return (latitude, longitude, altitude)

    def _set_navigation_origin(self, origin: Tuple[float, float, float]) -> None:
        self.state.origin_geodetic = origin
        self.state.origin_ecef = geodetic_to_ecef(*origin)

    def _clear_navigation_origin(self) -> None:
        self.state.origin_geodetic = None
        self.state.origin_ecef = None

    def _on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        active = self.reference_origin_active
        latitude = self.reference_origin_latitude_deg
        longitude = self.reference_origin_longitude_deg
        altitude = self.reference_origin_altitude_m
        touched_reference_origin = False

        for parameter in parameters:
            if parameter.name == "reference_origin_active":
                active = bool(parameter.value)
                touched_reference_origin = True
            elif parameter.name == "reference_origin_latitude_deg":
                latitude = float(parameter.value)
                touched_reference_origin = True
            elif parameter.name == "reference_origin_longitude_deg":
                longitude = float(parameter.value)
                touched_reference_origin = True
            elif parameter.name == "reference_origin_altitude_m":
                altitude = float(parameter.value)
                touched_reference_origin = True

        if not touched_reference_origin:
            return SetParametersResult(successful=True)

        if active and not (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and math.isfinite(altitude)
        ):
            return SetParametersResult(
                successful=False,
                reason="reference origin parameters must be finite when reference_origin_active is true",
            )

        self.reference_origin_active = active
        self.reference_origin_latitude_deg = latitude
        self.reference_origin_longitude_deg = longitude
        self.reference_origin_altitude_m = altitude

        if self.reference_origin_active:
            self._set_navigation_origin(
                (
                    self.reference_origin_latitude_deg,
                    self.reference_origin_longitude_deg,
                    self.reference_origin_altitude_m,
                )
            )
            self.get_logger().info(
                "Updated INS reference origin: lat=%.10f lon=%.10f alt=%.3f"
                % (
                    self.reference_origin_latitude_deg,
                    self.reference_origin_longitude_deg,
                    self.reference_origin_altitude_m,
                )
            )
        else:
            self._clear_navigation_origin()
            self.state.reset_origin_if_needed()
            self.get_logger().info(
                "INS reference origin deactivated; using first valid INS fix as ENU origin."
            )

        return SetParametersResult(successful=True)

    def _load_origin_from_map_projector_info(self) -> Optional[Tuple[float, float, float]]:
        if not self.map_projector_info_path:
            return None

        map_projector_info_path = Path(self.map_projector_info_path).expanduser()
        if not map_projector_info_path.is_file():
            raise ValueError(
                "Parameter 'map_projector_info_path' does not point to an existing file: %s"
                % map_projector_info_path
            )

        try:
            with map_projector_info_path.open("r", encoding="utf-8") as input_file:
                map_projector_info = yaml.safe_load(input_file) or {}
        except Exception as exc:
            raise ValueError(
                "Failed to load map projector info from %s: %s"
                % (map_projector_info_path, exc)
            ) from exc

        projector_type = str(map_projector_info.get("projector_type", "")).strip().lower()
        if projector_type and projector_type != "local":
            self.get_logger().warning(
                "map_projector_info_path points to projector_type '%s'. "
                "INS ENU conversion only supports 'Local'; falling back to first fix origin."
                % map_projector_info.get("projector_type")
            )
            return None

        map_origin = map_projector_info.get("map_origin")
        if not isinstance(map_origin, dict):
            raise ValueError(
                "map_projector_info_path must contain a 'map_origin' mapping: %s"
                % map_projector_info_path
            )

        try:
            latitude = float(map_origin["latitude"])
            longitude = float(map_origin["longitude"])
            altitude = float(map_origin["altitude"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                "map_projector_info_path has an invalid map_origin in %s: %s"
                % (map_projector_info_path, exc)
            ) from exc

        return (latitude, longitude, altitude)

    def _ensure_can_interface_is_up(self) -> None:
        flags = get_interface_flags(self.can_interface)
        if flags is None:
            self.get_logger().error(
                "CAN interface '%s' was not found. Bring the CAN interface up before launching this node."
                % self.can_interface
            )
            raise CanInterfaceNotReadyError(self.can_interface)

        if not (flags & IFF_UP):
            self.get_logger().error(
                "CAN interface '%s' is down. Bring the CAN interface up before launching this node."
                % self.can_interface
            )
            raise CanInterfaceNotReadyError(self.can_interface)

    def _open_can_socket(self) -> socket.socket:
        if not hasattr(socket, "AF_CAN") or not hasattr(socket, "CAN_RAW"):
            raise RuntimeError("This Python environment does not support SocketCAN.")

        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.settimeout(self.socket_timeout_sec)
        can_socket.bind((self.can_interface,))
        return can_socket

    def _receive_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                assert self._socket is not None
                raw_frame = self._socket.recv(CAN_READ_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"CAN receive failed: {exc}")
                return

            frame = decode_can_frame(raw_frame)
            if frame is None:
                continue

            self._write_can_line(frame)

            try:
                should_publish = update_navigation_state(self.state, frame)
                if frame.can_id in POSITION_FRAME_IDS or frame.can_id in ALTITUDE_FRAME_IDS:
                    self._publish_raw_nav_sat_fix()
                if should_publish:
                    self._publish_odometry(frame)
            except Exception as exc:
                self.get_logger().error(f"Failed to decode CAN frame 0x{frame.can_id:X}: {exc}")

    def _publish_raw_nav_sat_fix(self) -> None:
        if self.raw_nav_sat_fix_publisher is None:
            return
        if self.state.latitude_deg is None or self.state.longitude_deg is None:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.raw_nav_sat_fix_frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS
        if self.state.nav_flag is None or self.state.nav_flag > 0:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.latitude = self.state.latitude_deg
        msg.longitude = self.state.longitude_deg
        msg.altitude = float("nan") if self.state.altitude_m is None else self.state.altitude_m
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.raw_nav_sat_fix_publisher.publish(msg)

        self._published_raw_nav_sat_fix_count += 1
        if (
            self._published_raw_nav_sat_fix_count == 1
            or self._published_raw_nav_sat_fix_count % 50 == 0
        ):
            self.get_logger().info(
                "Published %d raw NavSatFix messages. Latest lat/lon=(%.7f, %.7f), alt=%s"
                % (
                    self._published_raw_nav_sat_fix_count,
                    self.state.latitude_deg,
                    self.state.longitude_deg,
                    "nan" if self.state.altitude_m is None else f"{self.state.altitude_m:.3f}",
                )
            )

    def _publish_odometry(self, frame: CanFrame) -> None:
        assert self.state.has_odometry()
        assert self.state.origin_geodetic is not None
        assert self.state.origin_ecef is not None
        assert self.state.latitude_deg is not None
        assert self.state.longitude_deg is not None
        assert self.state.altitude_m is not None
        assert self.state.pitch_deg is not None
        assert self.state.heading_deg is not None
        assert self.state.vel_enu is not None

        x, y, z = geodetic_to_enu(
            self.state.latitude_deg,
            self.state.longitude_deg,
            self.state.altitude_m,
            self.state.origin_geodetic,
            self.state.origin_ecef,
        )

        roll = math.radians(self.state.roll_deg)
        pitch = math.radians(self.state.pitch_deg)
        yaw = math.radians(90.0 - self.state.heading_deg)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        linear_body = rotate_world_to_body(self.state.vel_enu, roll, pitch, yaw)
        angular_body = self.state.angular_velocity_body
        stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.twist.twist.linear.x = linear_body[0]
        msg.twist.twist.linear.y = linear_body[1]
        msg.twist.twist.linear.z = linear_body[2]
        msg.twist.twist.angular.x = angular_body[0]
        msg.twist.twist.angular.y = angular_body[1]
        msg.twist.twist.angular.z = angular_body[2]

        self.publisher.publish(msg)
        self._published_count += 1
        self._write_odometry_line(msg, frame)

        if self._published_count == 1 or self._published_count % 50 == 0:
            yaw_deg = normalize_angle_deg(math.degrees(yaw))
            self.get_logger().info(
                "Published %d odom messages. Latest pos=(%.3f, %.3f, %.3f), "
                "ypr(deg)=(%.2f, %.2f, %.2f)"
                % (
                    self._published_count,
                    x,
                    y,
                    z,
                    yaw_deg,
                    self.state.pitch_deg,
                    self.state.roll_deg,
                )
            )

    def _write_odometry_line(self, msg: Odometry, frame: CanFrame) -> None:
        if self._log_file is None or self.log_format != LOG_FORMAT_ODOM_CSV:
            return

        self._log_file.write(
            f"{frame.stamp.isoformat(timespec='milliseconds')},"
            f"{msg.header.stamp.sec},"
            f"{msg.header.stamp.nanosec},"
            f"{msg.header.frame_id},"
            f"{msg.child_frame_id},"
            f"{msg.pose.pose.position.x:.6f},"
            f"{msg.pose.pose.position.y:.6f},"
            f"{msg.pose.pose.position.z:.6f},"
            f"{msg.pose.pose.orientation.x:.9f},"
            f"{msg.pose.pose.orientation.y:.9f},"
            f"{msg.pose.pose.orientation.z:.9f},"
            f"{msg.pose.pose.orientation.w:.9f},"
            f"{msg.twist.twist.linear.x:.6f},"
            f"{msg.twist.twist.linear.y:.6f},"
            f"{msg.twist.twist.linear.z:.6f},"
            f"{msg.twist.twist.angular.x:.9f},"
            f"{msg.twist.twist.angular.y:.9f},"
            f"{msg.twist.twist.angular.z:.9f}\n"
        )

    def _write_can_line(self, frame: CanFrame) -> None:
        if self._log_file is None or self.log_format != LOG_FORMAT_CAN:
            return

        self._log_file.write(format_can_line(frame))

    def destroy_node(self) -> bool:
        self._stop_event.set()

        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        if hasattr(self, "_receiver_thread") and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)

        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

        return super().destroy_node()


def get_interface_flags(interface_name: str) -> Optional[int]:
    if_name = interface_name.encode("utf-8")[:15]
    request = struct.pack("16sH", if_name, 0)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ioctl_socket:
            response = fcntl.ioctl(ioctl_socket.fileno(), SIOCGIFFLAGS, request)
    except OSError:
        return None

    return struct.unpack("16sH", response[:18])[1]


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Ez21InsDriverNode] = None
    try:
        node = Ez21InsDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except CanInterfaceNotReadyError:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
