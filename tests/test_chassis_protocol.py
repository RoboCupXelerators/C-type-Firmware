import math
import struct

import pytest

from tools.motor_control import protocol


def test_chassis_message_types_are_named():
    for message_type in (
        protocol.MsgType.CHASSIS_TWIST,
        protocol.MsgType.CHASSIS_ARM,
        protocol.MsgType.CHASSIS_STOP,
        protocol.MsgType.CHASSIS_DISABLE,
        protocol.MsgType.GET_CHASSIS_STATUS,
        protocol.MsgType.CHASSIS_STATUS,
        protocol.MsgType.MOTOR_REFRESH_STATUS,
        protocol.MsgType.CHASSIS_EMERGENCY_STOP,
        protocol.MsgType.CHASSIS_SET_PROFILE,
        protocol.MsgType.GET_CHASSIS_DIAGNOSTICS,
        protocol.MsgType.CHASSIS_DIAGNOSTICS,
    ):
        assert not protocol.MsgType.name(message_type).startswith("UNKNOWN")


def test_chassis_twist_round_trip():
    payload = protocol.encode_chassis_twist(0xFFFFFFF0, 0.03, -0.02, 0.2, 100)
    assert len(payload) == protocol.CHASSIS_TWIST_PAYLOAD_LEN
    result = protocol.decode_chassis_twist(payload)
    assert result["host_monotonic_ms"] == 0xFFFFFFF0
    assert result["vx_mps"] == pytest.approx(0.03)
    assert result["vy_mps"] == pytest.approx(-0.02)
    assert result["wz_rad_s"] == pytest.approx(0.2)
    assert result["valid_for_ms"] == 100


def test_enclosing_frame_provides_sequence_and_crc():
    payload = protocol.encode_chassis_twist(10, 0, 0, 0, 100)
    frame = protocol.build_frame(protocol.MsgType.CHASSIS_TWIST, 77, payload)
    encoded = frame.encode()
    parser = protocol.FrameParser()
    parser.feed(encoded)
    parsed = parser.next_frame()
    assert parsed is not None
    assert parsed.seq == 77
    assert parsed.payload == payload


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_chassis_twist_is_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        protocol.encode_chassis_twist(1, value, 0, 0, 100)


@pytest.mark.parametrize("valid_ms", [0, 1001])
def test_invalid_validity_window_is_rejected(valid_ms):
    with pytest.raises(ValueError, match="valid_for_ms"):
        protocol.encode_chassis_twist(1, 0, 0, 0, valid_ms)


def test_malformed_chassis_payload_is_rejected():
    with pytest.raises(ValueError, match="payload"):
        protocol.decode_chassis_twist(b"short")


def test_invalid_mode_in_wire_payload_is_rejected():
    payload = struct.pack(protocol.CHASSIS_TWIST_FORMAT, 1, 0.0, 0.0, 0.0, 100, 99, 0)
    with pytest.raises(ValueError, match="control mode"):
        protocol.decode_chassis_twist(payload)


def test_chassis_profile_encoding_rejects_unknown_modes():
    assert protocol.encode_chassis_profile(protocol.CHASSIS_PROFILE_NORMAL) == b"\x01"
    assert protocol.encode_chassis_profile(protocol.CHASSIS_PROFILE_COMPETITION_CANDIDATE) == b"\x02"
    with pytest.raises(ValueError, match="profile"):
        protocol.encode_chassis_profile(99)


def test_chassis_diagnostics_preserves_supported_and_unsupported_feedback():
    payload = bytearray(protocol.CHASSIS_DIAGNOSTICS_PAYLOAD_LEN)
    struct.pack_into("<BBBBIHH8f", payload, 0, 1, 1, 1, 0x0F, 5000, 20, 10,
                     16.7, 25.0, 40.0, 66.7, 240.0, 0.2, 0.5, 0.05)
    for index in range(4):
        struct.pack_into("<BBBB5fHBB", payload, 44 + index * 28,
                         index + 1, 1, 2, 0x07,
                         -10.0 - index, -20.0 - index, 5.0 + index,
                         -15.0 - 2 * index, 0.4 + index, 3 + index, 1, 1)
    struct.pack_into("<5I4f4B3I", payload, 156,
                     7, 1000, 1010, 1100, 1200,
                     20.0, -20.0, 18.0, 1.2,
                     4, 1, 1, 1, 10, 90, 100)

    decoded = protocol.decode_chassis_diagnostics(bytes(payload))

    assert decoded["profile"] == protocol.CHASSIS_PROFILE_NORMAL
    assert decoded["limits"]["reverse_brake_rad_s2"] > decoded["limits"]["normal_decel_rad_s2"]
    assert [wheel["wheel"] for wheel in decoded["wheels"]] == ["FL", "FR", "RL", "RR"]
    assert decoded["wheels"][0]["measured_torque_nm"] == pytest.approx(0.4)
    assert decoded["wheels"][0]["measured_current_a"] is None
    assert decoded["wheels"][0]["bus_voltage_v"] is None
    event = decoded["latest_reverse_event"]
    assert event["command_to_brake_latency_ms"] == 10
    assert event["zero_cross_to_reverse_response_ms"] == 100
    assert event["peak_current_a"] is None
