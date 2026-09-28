import struct

import pytest

from ins_driver_ez21.protocol import CAN_FRAME_FORMAT
from ins_driver_ez21.protocol import NavigationState
from ins_driver_ez21.protocol import POSITION_FRAME_IDS
from ins_driver_ez21.protocol import VELOCITY_FRAME_IDS
from ins_driver_ez21.protocol import decode_can_frame
from ins_driver_ez21.protocol import geodetic_to_ecef
from ins_driver_ez21.protocol import geodetic_to_enu
from ins_driver_ez21.protocol import quaternion_from_euler
from ins_driver_ez21.protocol import update_navigation_state


def make_raw_frame(can_id: int, payload: bytes) -> bytes:
    return struct.pack(
        CAN_FRAME_FORMAT,
        can_id,
        len(payload),
        payload.ljust(8, b"\x00"),
    )


def test_decode_can_frame_decodes_standard_frame():
    raw_frame = make_raw_frame(0x20B, bytes.fromhex("15CD5B07C7CFD204"))

    frame = decode_can_frame(raw_frame)

    assert frame is not None
    assert frame.can_id == 0x20B
    assert frame.data == bytes.fromhex("15CD5B07C7CFD204")


def test_update_navigation_state_builds_origin_and_odometry_readiness():
    state = NavigationState()

    attitude = bytes.fromhex("10270A0000000000")
    position = struct.pack("<ii", int(30.1234567 * 1e7), int(114.7654321 * 1e7))
    altitude = struct.pack("<iB", int(123.456 * 1000), 1)
    velocity = struct.pack("<hhh", 120, -30, 5)

    assert not update_navigation_state(state, decode_can_frame(make_raw_frame(0x10B, attitude)))
    assert not update_navigation_state(
        state, decode_can_frame(make_raw_frame(next(iter(POSITION_FRAME_IDS)), position))
    )
    assert not update_navigation_state(state, decode_can_frame(make_raw_frame(0x30B, altitude)))
    assert update_navigation_state(
        state, decode_can_frame(make_raw_frame(next(iter(VELOCITY_FRAME_IDS)), velocity))
    )

    assert state.origin_geodetic == pytest.approx((30.1234567, 114.7654321, 123.456), abs=1e-6)
    assert state.vel_enu == pytest.approx((1.2, -0.3, 0.05), abs=1e-6)


def test_geodetic_to_enu_returns_zero_at_origin():
    origin = (30.1234567, 114.7654321, 123.456)

    origin_ecef = geodetic_to_ecef(*origin)
    east, north, up = geodetic_to_enu(*origin, origin, origin_ecef)

    assert east == pytest.approx(0.0, abs=1e-6)
    assert north == pytest.approx(0.0, abs=1e-6)
    assert up == pytest.approx(0.0, abs=1e-6)


def test_quaternion_from_euler_identity():
    quaternion = quaternion_from_euler(0.0, 0.0, 0.0)

    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)
