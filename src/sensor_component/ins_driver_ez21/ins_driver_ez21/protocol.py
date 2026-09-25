import math
import struct
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional
from typing import TextIO
from typing import Tuple


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

ATTITUDE_FRAME_ID = 0x10B
POSITION_FRAME_IDS = frozenset({0x20B, 0x21B})
ALTITUDE_FRAME_IDS = frozenset({0x30B, 0x31B})
VELOCITY_FRAME_IDS = frozenset({0x40B, 0x41B})
GYRO_FRAME_ID = 0x60B
GYRO_ACCEL_FRAME_ID = 0x70B

LOG_FORMAT_CAN = "can"
LOG_FORMAT_ODOM_CSV = "odom_csv"
LOG_FORMAT_ALIASES = {
    "can": LOG_FORMAT_CAN,
    "can_signal": LOG_FORMAT_CAN,
    "raw_can": LOG_FORMAT_CAN,
    "odom": LOG_FORMAT_ODOM_CSV,
    "csv": LOG_FORMAT_ODOM_CSV,
    "odom_csv": LOG_FORMAT_ODOM_CSV,
}

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_READ_SIZE = 72
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF


@dataclass(frozen=True)
class CanFrame:
    stamp: datetime
    can_id: int
    data: bytes


@dataclass
class NavigationState:
    heading_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: float = 0.0
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    altitude_m: Optional[float] = None
    nav_flag: Optional[int] = None
    vel_enu: Optional[Tuple[float, float, float]] = None
    angular_velocity_body: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_geodetic: Optional[Tuple[float, float, float]] = None
    origin_ecef: Optional[Tuple[float, float, float]] = None

    def has_pose(self) -> bool:
        return (
            self.heading_deg is not None
            and self.pitch_deg is not None
            and self.latitude_deg is not None
            and self.longitude_deg is not None
            and self.altitude_m is not None
        )

    def has_odometry(self) -> bool:
        return self.has_pose() and self.vel_enu is not None

    def reset_origin_if_needed(self) -> None:
        if self.origin_geodetic is not None:
            return
        if self.latitude_deg is None or self.longitude_deg is None or self.altitude_m is None:
            return

        self.origin_geodetic = (self.latitude_deg, self.longitude_deg, self.altitude_m)
        self.origin_ecef = geodetic_to_ecef(*self.origin_geodetic)


def decode_can_frame(raw_frame: bytes) -> Optional[CanFrame]:
    if len(raw_frame) < CAN_FRAME_SIZE:
        return None

    raw_can_id, can_dlc, payload = struct.unpack(CAN_FRAME_FORMAT, raw_frame[:CAN_FRAME_SIZE])
    if raw_can_id & CAN_ERR_FLAG or raw_can_id & CAN_RTR_FLAG or can_dlc > 8:
        return None

    if raw_can_id & CAN_EFF_FLAG:
        can_id = raw_can_id & CAN_EFF_MASK
    else:
        can_id = raw_can_id & CAN_SFF_MASK

    return CanFrame(stamp=datetime.now(), can_id=can_id, data=payload[:can_dlc])


def update_navigation_state(state: NavigationState, frame: CanFrame) -> bool:
    data = frame.data
    can_id = frame.can_id

    if can_id == ATTITUDE_FRAME_ID and len(data) >= 4:
        state.heading_deg = decode_u16(data, 0) * 1e-2
        state.pitch_deg = decode_s16(data, 2) * 1e-2
        if len(data) >= 6:
            state.roll_deg = decode_s16(data, 4) * 1e-2
        return False

    if can_id in POSITION_FRAME_IDS and len(data) >= 8:
        state.latitude_deg = decode_s32(data, 0) * 1e-7
        state.longitude_deg = decode_s32(data, 4) * 1e-7
        state.reset_origin_if_needed()
        return False

    if can_id in ALTITUDE_FRAME_IDS and len(data) >= 5:
        state.altitude_m = decode_s32(data, 0) * 1e-3
        state.nav_flag = data[4]
        state.reset_origin_if_needed()
        return False

    if can_id in VELOCITY_FRAME_IDS and len(data) >= 6:
        east = decode_s16(data, 0) * 1e-2
        north = decode_s16(data, 2) * 1e-2
        up = decode_s16(data, 4) * 1e-2
        state.vel_enu = (east, north, up)
        return state.has_odometry()

    if can_id == GYRO_FRAME_ID and len(data) >= 8:
        wx = math.radians(decode_s32(data, 0) * 1e-5)
        wy = math.radians(decode_s32(data, 4) * 1e-5)
        _, _, wz = state.angular_velocity_body
        state.angular_velocity_body = (wx, wy, wz)
        return False

    if can_id == GYRO_ACCEL_FRAME_ID and len(data) >= 4:
        wx, wy, _ = state.angular_velocity_body
        wz = math.radians(decode_s32(data, 0) * 1e-5)
        state.angular_velocity_body = (wx, wy, wz)

    return False


def resolve_log_output_path(log_path_text: str, log_name: str, log_name_format: str) -> Path:
    base_path = Path(log_path_text).expanduser()
    if base_path.suffix.lower() in {".txt", ".csv"}:
        return base_path

    if log_name.strip() and log_name.strip().lower() != "auto":
        file_name = log_name.strip()
    else:
        file_name = datetime.now().strftime(log_name_format)
    return base_path / file_name


def open_log_file(path: Path, log_format: str) -> TextIO:
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new_file = not path.exists() or path.stat().st_size == 0
    handle = path.open("a", encoding="utf-8", buffering=1)
    if is_new_file and log_format == LOG_FORMAT_ODOM_CSV:
        handle.write(
            "receive_time,stamp_sec,stamp_nanosec,frame_id,child_frame_id,"
            "position_x,position_y,position_z,"
            "orientation_x,orientation_y,orientation_z,orientation_w,"
            "linear_x,linear_y,linear_z,"
            "angular_x,angular_y,angular_z\n"
        )
    return handle


def normalize_log_format(value: str) -> str:
    normalized = value.strip().lower()
    log_format = LOG_FORMAT_ALIASES.get(normalized)
    if log_format is None:
        valid_formats = ", ".join(sorted(set(LOG_FORMAT_ALIASES.values())))
        raise ValueError(f"Parameter 'log_format' must be one of: {valid_formats}.")
    return log_format


def format_can_line(frame: CanFrame) -> str:
    can_id_width = 3 if frame.can_id <= CAN_SFF_MASK else 8
    return f"{frame.can_id:0{can_id_width}X}#{frame.data.hex().upper()}\n"


def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def decode_u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=False)


def decode_s16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=True)


def decode_s32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="little", signed=True)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (radius + alt_m) * cos_lat * cos_lon
    y = (radius + alt_m) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return (x, y, z)


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_geodetic: Tuple[float, float, float],
    origin_ecef: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    dx = x - origin_ecef[0]
    dy = y - origin_ecef[1]
    dz = z - origin_ecef[2]

    lat0 = math.radians(origin_geodetic[0])
    lon0 = math.radians(origin_geodetic[1])
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return (east, north, up)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def rotate_world_to_body(
    vector_enu: Tuple[float, float, float], roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float]:
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)
    sy = math.sin(yaw)
    cy = math.cos(yaw)

    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )

    vx = (
        rotation[0][0] * vector_enu[0]
        + rotation[1][0] * vector_enu[1]
        + rotation[2][0] * vector_enu[2]
    )
    vy = (
        rotation[0][1] * vector_enu[0]
        + rotation[1][1] * vector_enu[1]
        + rotation[2][1] * vector_enu[2]
    )
    vz = (
        rotation[0][2] * vector_enu[0]
        + rotation[1][2] * vector_enu[1]
        + rotation[2][2] * vector_enu[2]
    )
    return (vx, vy, vz)
