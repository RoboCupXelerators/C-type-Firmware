"""Pi-C minimal communication protocol.

Frame layout (all fields little-endian unless noted):

    Offset  Size  Field
    ------  ----  -----
    0       2     Frame header: 0x55 0xAA
    2       1     Protocol version (currently 1)
    3       1     Message type (see MsgType)
    4       1     Sequence number (wraps 0-255)
    5       1     Payload length (0-250)
    6       N     Payload (N bytes, N = payload length)
    6+N     2     CRC16-CCITT (0x1021, init 0xFFFF) over bytes [0 .. 6+N-1]
    8+N     1     Frame tail: 0x0D

Maximum frame size = 259 bytes.

DM-S2325-1EC CAN protocol is based on the official Damiao V1.1 manual
(2026-05-13) and official Gitee SDK code (gitee.com/kit-miao/damiao).
Source: reference/vendor/damiao-official
"""

from __future__ import annotations

import struct
import math
from dataclasses import dataclass
from typing import Optional

# --- Pi-C protocol constants ---

PROTOCOL_VERSION = 1
FRAME_HEADER = b"\x55\xAA"
FRAME_TAIL = 0x0D
MAX_PAYLOAD = 250

# DM-S2325-1EC official V1.1 defaults (from manual page 15):
#   "Position, velocity, and torque are preset to +/-12.5, +/-200, and +/-20"
# These are MIT mode mapping ranges. For Velocity Mode, v_des is a raw
# float32 in rad/s with no mapping — the driver applies its own limits.
# Our safety clamp uses a conservative value well below the motor's
# mechanical limits (560 rpm output = ~58.6 rad/s).
SAFE_SPEED_MIN = -30.0   # rad/s, conservative safety clamp
SAFE_SPEED_MAX = 30.0    # rad/s, conservative safety clamp
MAX_DURATION_MS = 5000
MIN_DURATION_MS = 100

DEFAULT_MOTOR_IDS = frozenset({1})

# DM-S2325-1EC motor specifications (from V1.1 manual)
DM_S2325_RATED_VOLTAGE_V = 24.0
DM_S2325_RATED_TORQUE_NM = 1.35
DM_S2325_PEAK_TORQUE_NM = 5.0
DM_S2325_RATED_SPEED_RPM = 380.0
DM_S2325_MAX_SPEED_RPM = 560.0
DM_S2325_GEAR_RATIO = 25.0
DM_S2325_POLE_PAIRS = 7
DM_S2325_RATED_CURRENT_A = 3.2
DM_S2325_PEAK_CURRENT_A = 7.5

# MIT mode mapping defaults (from V1.1 manual)
DM_S2325_PMAX_DEFAULT = 12.5    # rad
DM_S2325_VMAX_DEFAULT = 200.0   # rad/s (MIT mapping range, not mechanical limit)
DM_S2325_TMAX_DEFAULT = 10.0    # Nm (actual motor value, read via 0x33)

# --- Damiao CAN protocol constants (from V1.1 manual) ---

# Mode offsets for CAN ID
DM_MODE_OFFSET_MIT       = 0x000
DM_MODE_OFFSET_POS_VEL   = 0x100
DM_MODE_OFFSET_VEL       = 0x200
DM_MODE_OFFSET_POS_FORCE = 0x300

# Parameter read/write CAN ID
DM_PARAM_CAN_ID          = 0x7FF

# Control command bytes (last byte of 0xFF*7 + cmd)
DM_CMD_ENABLE            = 0xFC
DM_CMD_DISABLE           = 0xFD
DM_CMD_SET_ZERO          = 0xFE
DM_CMD_CLEAR_ERROR       = 0xFB

# Control mode codes (register 0x0A)
DM_CTRL_MODE_MIT         = 1
DM_CTRL_MODE_POS_VEL     = 2
DM_CTRL_MODE_VEL         = 3
DM_CTRL_MODE_POS_FORCE   = 4

# CAN baud rate codes (register 0x23)
DM_CAN_BAUD_125K  = 0
DM_CAN_BAUD_200K  = 1
DM_CAN_BAUD_250K  = 2
DM_CAN_BAUD_500K  = 3
DM_CAN_BAUD_1M    = 4
DM_CAN_BAUD_2M    = 5
DM_CAN_BAUD_2M5   = 6
DM_CAN_BAUD_3M2   = 7
DM_CAN_BAUD_4M    = 8
DM_CAN_BAUD_5M    = 9

# Default CAN baud rate: 1 Mbps (code 4), classic CAN
DM_CAN_BAUD_DEFAULT = DM_CAN_BAUD_1M
DM_CAN_BAUD_DEFAULT_HZ = 1000000

# Velocity mode data length: 4 bytes (float32 LE)
DM_VEL_MODE_DATA_LEN = 4
# Enable/disable/clear data length: 8 bytes
DM_CMD_DATA_LEN = 8

# MIT mode Kp/Kd encoding range (official dm_motor_drv.h)
DM_MIT_KP_MIN = 0.0
DM_MIT_KP_MAX = 500.0
DM_MIT_KD_MIN = 0.0
DM_MIT_KD_MAX = 5.0

# --- Error codes from V1.1 manual (feedback D[0] >> 4) ---

class DmError:
    DISABLED             = 0x0
    ENABLED              = 0x1
    SENSOR_ERROR         = 0x5
    PARAM_ERROR          = 0x6
    OVER_VOLTAGE         = 0x8
    UNDER_VOLTAGE        = 0x9
    OVER_CURRENT         = 0xA
    MOS_OVER_TEMP        = 0xB
    COIL_OVER_TEMP       = 0xC
    COMM_LOST            = 0xD
    OVERLOAD             = 0xE

    _names = {
        0x0: "DISABLED", 0x1: "ENABLED", 0x5: "SENSOR_ERROR",
        0x6: "PARAM_ERROR", 0x8: "OVER_VOLTAGE", 0x9: "UNDER_VOLTAGE",
        0xA: "OVER_CURRENT", 0xB: "MOS_OVER_TEMP", 0xC: "COIL_OVER_TEMP",
        0xD: "COMM_LOST", 0xE: "OVERLOAD",
    }

    @classmethod
    def name(cls, val: int) -> str:
        return cls._names.get(val, f"UNKNOWN(0x{val:02X})")


class MsgType:
    PING            = 0x01
    PONG            = 0x02
    GET_STATUS      = 0x03
    STATUS          = 0x04
    MOTOR_ENABLE    = 0x05
    MOTOR_DISABLE   = 0x06
    MOTOR_SET_SPEED = 0x07
    MOTOR_STOP      = 0x08
    HEARTBEAT       = 0x09
    ACK             = 0x0A
    NACK            = 0x0B
    ERROR           = 0x0C
    GET_CAN_STATUS  = 0x0D
    CAN_STATUS      = 0x0E
    GET_BOARD_INFO  = 0x0F
    BOARD_INFO         = 0x10
    DISCOVERY_START    = 0x11
    DISCOVERY_STATUS   = 0x12
    DISCOVERY_RESULT   = 0x13
    DISCOVERY_STOP     = 0x14
    SET_CAN_CHANNEL    = 0x15
    MIT_CTRL           = 0x16
    MIT_STOP           = 0x17
    CHASSIS_TWIST      = 0x18
    CHASSIS_ARM        = 0x19
    CHASSIS_STOP       = 0x1A
    CHASSIS_DISABLE    = 0x1B
    GET_CHASSIS_STATUS = 0x1C
    CHASSIS_STATUS     = 0x1D
    MOTOR_REFRESH_STATUS = 0x1E
    DM_PARAM_READ      = 0x1F
    DM_PARAM_WRITE     = 0x20
    DM_PARAM_SAVE      = 0x21
    DM_PARAM_RESULT    = 0x22
    CHASSIS_REFRESH_DISABLED = 0x23
    CHASSIS_CLEAR_FAULT = 0x24
    CHASSIS_EMERGENCY_STOP = 0x25
    GET_IMU_STATUS     = 0x26
    IMU_STATUS         = 0x27
    IMU_ZERO_YAW       = 0x28
    IMU_SET_REFERENCE  = 0x29
    CHASSIS_SET_PROFILE = 0x2A
    GET_CHASSIS_DIAGNOSTICS = 0x2B
    CHASSIS_DIAGNOSTICS = 0x2C

    _names = {
        0x01: "PING", 0x02: "PONG", 0x03: "GET_STATUS", 0x04: "STATUS",
        0x05: "MOTOR_ENABLE", 0x06: "MOTOR_DISABLE", 0x07: "MOTOR_SET_SPEED",
        0x08: "MOTOR_STOP", 0x09: "HEARTBEAT", 0x0A: "ACK", 0x0B: "NACK",
        0x0C: "ERROR", 0x0D: "GET_CAN_STATUS", 0x0E: "CAN_STATUS",
        0x0F: "GET_BOARD_INFO", 0x10: "BOARD_INFO",
        0x11: "DISCOVERY_START", 0x12: "DISCOVERY_STATUS",
        0x13: "DISCOVERY_RESULT", 0x14: "DISCOVERY_STOP",
        0x15: "SET_CAN_CHANNEL", 0x16: "MIT_CTRL", 0x17: "MIT_STOP",
        0x18: "CHASSIS_TWIST", 0x19: "CHASSIS_ARM", 0x1A: "CHASSIS_STOP",
        0x1B: "CHASSIS_DISABLE", 0x1C: "GET_CHASSIS_STATUS",
        0x1D: "CHASSIS_STATUS", 0x1E: "MOTOR_REFRESH_STATUS",
        0x1F: "DM_PARAM_READ", 0x20: "DM_PARAM_WRITE",
        0x21: "DM_PARAM_SAVE", 0x22: "DM_PARAM_RESULT",
        0x23: "CHASSIS_REFRESH_DISABLED",
        0x24: "CHASSIS_CLEAR_FAULT",
        0x25: "CHASSIS_EMERGENCY_STOP",
        0x26: "GET_IMU_STATUS", 0x27: "IMU_STATUS",
        0x28: "IMU_ZERO_YAW", 0x29: "IMU_SET_REFERENCE",
        0x2A: "CHASSIS_SET_PROFILE", 0x2B: "GET_CHASSIS_DIAGNOSTICS",
        0x2C: "CHASSIS_DIAGNOSTICS",
    }

    @classmethod
    def name(cls, val: int) -> str:
        return cls._names.get(val, f"UNKNOWN(0x{val:02X})")


class NackReason:
    CRC_ERROR        = 0x01
    BAD_VERSION      = 0x02
    UNKNOWN_MSG      = 0x03
    BAD_PAYLOAD      = 0x04
    BAD_MOTOR_ID     = 0x05
    NOT_ENABLED      = 0x06
    OUT_OF_RANGE     = 0x07
    BUSY             = 0x08
    INTERNAL_ERROR   = 0x09
    WATCHDOG_TIMEOUT = 0x0A

    _names = {
        0x01: "CRC_ERROR", 0x02: "BAD_VERSION", 0x03: "UNKNOWN_MSG",
        0x04: "BAD_PAYLOAD", 0x05: "BAD_MOTOR_ID", 0x06: "NOT_ENABLED",
        0x07: "OUT_OF_RANGE", 0x08: "BUSY", 0x09: "INTERNAL_ERROR",
        0x0A: "WATCHDOG_TIMEOUT",
    }

    @classmethod
    def name(cls, val: int) -> str:
        return cls._names.get(val, f"UNKNOWN(0x{val:02X})")


class ErrorSeverity:
    INFO     = 0x00
    WARNING  = 0x01
    CRITICAL = 0x02


# --- CRC16-CCITT (0x1021, init 0xFFFF) ---

def crc16(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# --- Frame dataclass ---

@dataclass
class Frame:
    version: int = PROTOCOL_VERSION
    msg_type: int = 0
    seq: int = 0
    payload: bytes = b""

    def __post_init__(self):
        if self.payload is None:
            self.payload = b""
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f"Payload too large: {len(self.payload)} > {MAX_PAYLOAD}")

    @property
    def msg_name(self) -> str:
        return MsgType.name(self.msg_type)

    def encode(self) -> bytes:
        body = struct.pack(
            "<BBBB",
            self.version, self.msg_type, self.seq & 0xFF, len(self.payload),
        )
        pre_crc = FRAME_HEADER + body + self.payload
        crc = crc16(pre_crc)
        return pre_crc + struct.pack("<H", crc) + bytes([FRAME_TAIL])


# --- Payload encode helpers ---
# STATUS payload format: <BBfffHB> = 17 bytes

STATUS_PAYLOAD_LEN = 41
CAN_STATUS_PAYLOAD_LEN = 29
BOARD_INFO_PAYLOAD_LEN = 20

def encode_ping() -> bytes:
    return b""

def encode_pong(protocol_ver: int = PROTOCOL_VERSION) -> bytes:
    return struct.pack("<B", protocol_ver)

def encode_get_status() -> bytes:
    return b""

def encode_status(motor_id, enabled, online, fault, actual_speed,
                  temperature_c, bus_voltage, error_code=0, *, position=0.0,
                  torque=0.0, raw_can_data=b"\x00" * 8,
                  feedback_sequence=0, feedback_tick_ms=0):
    flags = (1 if enabled else 0) | (2 if online else 0) | (4 if fault else 0)
    if len(raw_can_data) != 8:
        raise ValueError("raw CAN feedback must be exactly 8 bytes")
    return struct.pack(
        "<BBfffHB",
        motor_id & 0xFF, flags, float(actual_speed),
        float(temperature_c), float(bus_voltage),
        error_code & 0xFFFF, PROTOCOL_VERSION,
    ) + struct.pack("<ff", float(position), float(torque)) + bytes(raw_can_data) + struct.pack(
        "<II", int(feedback_sequence) & 0xFFFFFFFF, int(feedback_tick_ms) & 0xFFFFFFFF
    )

def encode_motor_enable(motor_id: int) -> bytes:
    return struct.pack("<B", motor_id & 0xFF)

def encode_motor_disable(motor_id: int) -> bytes:
    return struct.pack("<B", motor_id & 0xFF)

def encode_motor_set_speed(motor_id: int, target_speed: float, duration_ms: int) -> bytes:
    return struct.pack("<BfI", motor_id & 0xFF, float(target_speed), duration_ms & 0xFFFFFFFF)

def encode_motor_stop(motor_id: int) -> bytes:
    return struct.pack("<B", motor_id & 0xFF)

def encode_heartbeat() -> bytes:
    return b""

def encode_ack(seq: int, msg_type: int) -> bytes:
    return struct.pack("<BB", seq & 0xFF, msg_type & 0xFF)

def encode_nack(seq: int, msg_type: int, reason: int) -> bytes:
    return struct.pack("<BBB", seq & 0xFF, msg_type & 0xFF, reason & 0xFF)

def encode_error(severity: int, error_code: int, message: str = "") -> bytes:
    raw = message.encode("utf-8")[:240]
    return struct.pack("<BB", severity & 0xFF, error_code & 0xFF) + raw


# --- Payload decode helpers ---

def decode_pong(payload: bytes) -> dict:
    if len(payload) < 1:
        raise ValueError("PONG payload too short")
    return {"protocol_version": payload[0]}

def decode_status(payload: bytes) -> dict:
    if len(payload) < 17:
        raise ValueError(f"STATUS payload too short: {len(payload)}")
    motor_id, flags, speed, temp, volt, err_code, ver = struct.unpack(
        "<BBfffHB", payload[:17]
    )
    position = struct.unpack("<f", payload[17:21])[0] if len(payload) >= 21 else 0.0
    torque = struct.unpack("<f", payload[21:25])[0] if len(payload) >= 25 else 0.0
    raw_can_data = bytes(payload[25:33]) if len(payload) >= 33 else b""
    feedback_sequence = struct.unpack("<I", payload[33:37])[0] if len(payload) >= 37 else None
    feedback_tick_ms = struct.unpack("<I", payload[37:41])[0] if len(payload) >= 41 else None
    raw_feedback = decode_dm_feedback(raw_can_data, can_id=0) if len(raw_can_data) == 8 and any(raw_can_data) else None
    return {
        "motor_id": motor_id,
        "enabled": bool(flags & 1),
        "online": bool(flags & 2),
        "fault": bool(flags & 4),
        "actual_speed": speed,
        "temperature_c": temp,
        "bus_voltage": volt,
        "error_code": err_code,
        "protocol_version": ver,
        "position": position,
        "torque": torque,
        "raw_can_data": raw_can_data,
        "raw_can_data_hex": raw_can_data.hex().upper(),
        "raw_feedback": raw_feedback,
        "feedback_sequence": feedback_sequence,
        "feedback_tick_ms": feedback_tick_ms,
        "position_raw": raw_feedback["position_raw"] if raw_feedback else None,
        "velocity_raw": raw_feedback["velocity_raw"] if raw_feedback else None,
        "torque_raw": raw_feedback["torque_raw"] if raw_feedback else None,
    }


def encode_can_status(can_init_ok: bool, can_bus_off: bool,
                      tx_count: int, tx_fail_count: int,
                      rx_count: int, rx_error_count: int,
                      can_esr: int, can_bsr: int) -> bytes:
    """Encode CAN diagnostic status (section 39 requirement).

    Layout: <BBIIIIII>
      B  can_init_ok    (0/1)
      B  can_bus_off    (0/1)
      I  tx_count
      I  tx_fail_count
      I  rx_count
      I  rx_error_count
      I  can_esr        (HAL CAN ESR register)
      I  can_bsr        (HAL CAN BSR register)
    Total: 28 bytes
    """
    flags = (1 if can_init_ok else 0) | (2 if can_bus_off else 0)
    return struct.pack(
        "<BIIIIIII",
        flags,
        tx_count & 0xFFFFFFFF, tx_fail_count & 0xFFFFFFFF,
        rx_count & 0xFFFFFFFF, rx_error_count & 0xFFFFFFFF,
        can_esr & 0xFFFFFFFF, can_bsr & 0xFFFFFFFF,
        0,  # reserved
    )


def decode_can_status(payload: bytes) -> dict:
    if len(payload) < CAN_STATUS_PAYLOAD_LEN:
        raise ValueError(f"CAN_STATUS payload too short: {len(payload)}")
    flags, tx_ok, tx_fail, rx_ok, rx_err, esr, bsr, _reserved = struct.unpack(
        "<BIIIIIII", payload[:CAN_STATUS_PAYLOAD_LEN]
    )
    return {
        "can_init_ok": bool(flags & 1),
        "can_bus_off": bool(flags & 2),
        "tx_count": tx_ok,
        "tx_fail_count": tx_fail,
        "rx_count": rx_ok,
        "rx_error_count": rx_err,
        "can_esr": esr,
        "can_bsr": bsr,
    }


def encode_board_info(fw_version: int, can_baud_code: int,
                      can1_used: bool, can2_used: bool,
                      motor_slave_id: int, motor_master_id: int) -> bytes:
    """Encode board info response (section 39 requirement).

    Layout: <BBBBIH...>
      B  fw_version
      B  can_baud_code
      B  can1_used (0/1)
      B  can2_used (0/1)
      I  uptime_ms
      H  motor_slave_id
      H  motor_master_id
      + padding to 20 bytes
    """
    import time as _time
    uptime = int(_time.monotonic() * 1000) & 0xFFFFFFFF
    buf = struct.pack("<BBBBIHH", fw_version & 0xFF, can_baud_code & 0xFF,
                      1 if can1_used else 0, 1 if can2_used else 0,
                      uptime,
                      motor_slave_id & 0xFFFF, motor_master_id & 0xFFFF)
    # Pad to BOARD_INFO_PAYLOAD_LEN
    buf += bytes(BOARD_INFO_PAYLOAD_LEN - len(buf))
    return buf


def decode_board_info(payload: bytes) -> dict:
    if len(payload) < 12:
        raise ValueError(f"BOARD_INFO payload too short: {len(payload)}")
    fw_ver, baud, can1, can2, uptime, sid, mid = struct.unpack(
        "<BBBBIHH", payload[:12]
    )
    return {
        "fw_version": fw_ver,
        "can_baud_code": baud,
        "can1_used": bool(can1),
        "can2_used": bool(can2),
        "uptime_ms": uptime,
        "motor_slave_id": sid,
        "motor_master_id": mid,
    }

def decode_motor_set_speed(payload: bytes) -> dict:
    if len(payload) < 9:
        raise ValueError("MOTOR_SET_SPEED payload too short")
    motor_id, target, duration = struct.unpack("<BfI", payload[:9])
    return {"motor_id": motor_id, "target_speed": target, "duration_ms": duration}

def decode_motor_id(payload: bytes) -> dict:
    if len(payload) < 1:
        raise ValueError("Payload too short for motor ID")
    return {"motor_id": payload[0]}

def decode_ack(payload: bytes) -> dict:
    if len(payload) < 2:
        raise ValueError("ACK payload too short")
    seq, msg_type = struct.unpack("<BB", payload[:2])
    return {"ack_seq": seq, "ack_msg_type": msg_type, "ack_msg_name": MsgType.name(msg_type)}

def decode_nack(payload: bytes) -> dict:
    if len(payload) < 3:
        raise ValueError("NACK payload too short")
    seq, msg_type, reason = struct.unpack("<BBB", payload[:3])
    return {
        "nack_seq": seq, "nack_msg_type": msg_type,
        "nack_reason": reason, "nack_reason_name": NackReason.name(reason),
    }

def decode_error(payload: bytes) -> dict:
    if len(payload) < 2:
        raise ValueError("ERROR payload too short")
    severity, error_code = struct.unpack("<BB", payload[:2])
    message = payload[2:].decode("utf-8", errors="replace") if len(payload) > 2 else ""
    return {"severity": severity, "error_code": error_code, "message": message}


# --- Damiao CAN frame encoding (from V1.1 manual) ---

def encode_dm_vel_command(motor_id: int, velocity_rad_s: float) -> tuple[int, bytes]:
    """Encode a Velocity Mode CAN command frame.

    Returns (can_id, data_bytes) where:
    - can_id = motor_id + 0x200
    - data = 4 bytes float32 little-endian

    Per V1.1 manual page 17: "Control Frame in Velocity Mode"
    Frame ID = 0x200 + ID, data = v_des (float32 LE), 4 bytes only.
    """
    if math.isnan(velocity_rad_s) or math.isinf(velocity_rad_s):
        raise ValueError(f"Velocity must be finite, got {velocity_rad_s}")
    if motor_id < 0 or motor_id > 0x7FF:
        raise ValueError(f"Motor ID out of range: {motor_id}")
    can_id = motor_id + DM_MODE_OFFSET_VEL
    raw = struct.pack('<f', float(velocity_rad_s))
    return can_id, raw

def encode_dm_enable(motor_id: int) -> tuple[int, bytes]:
    """Encode an Enable CAN command frame.

    Per V1.1 manual page 17: data = 0xFF*7 + 0xFC, 8 bytes.
    CAN ID = motor_id (MIT mode) or motor_id + mode_offset.
    For Velocity Mode, use motor_id + 0x200.
    """
    can_id = motor_id + DM_MODE_OFFSET_VEL
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_ENABLE])
    return can_id, data

def encode_dm_disable(motor_id: int) -> tuple[int, bytes]:
    """Encode a Disable CAN command frame.

    Per V1.1 manual page 17: data = 0xFF*7 + 0xFD, 8 bytes.
    """
    can_id = motor_id + DM_MODE_OFFSET_VEL
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_DISABLE])
    return can_id, data

def encode_dm_clear_error(motor_id: int) -> tuple[int, bytes]:
    """Encode a Clear Faults CAN command frame.

    Per V1.1 manual page 17: data = 0xFF*7 + 0xFB, 8 bytes.
    """
    can_id = motor_id + DM_MODE_OFFSET_VEL
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_CLEAR_ERROR])
    return can_id, data

def decode_dm_feedback(data: bytes, can_id: int,
                       pmax: float = DM_S2325_PMAX_DEFAULT,
                       vmax: float = DM_S2325_VMAX_DEFAULT,
                       tmax: float = DM_S2325_TMAX_DEFAULT) -> dict:
    """Decode a Damiao motor feedback frame (8 bytes).

    Per V1.1 manual page 14:
    D[0]: ID (low 4 bits) | ERR (high 4 bits)
    D[1:2]: POS uint16 -> [-PMAX, PMAX] (16-bit)
    D[3:4]: VEL uint12 (top 12 bits) -> [-VMAX, VMAX]
    D[4:5]: T uint12 (bottom nibble of D[4] + D[5]) -> [-TMAX, TMAX]
    D[6]: T_MOS (temperature C)
    D[7]: T_Rotor (temperature C)
    """
    if len(data) < 8:
        raise ValueError(f"Feedback data too short: {len(data)} < 8")
    err_code = (data[0] >> 4) & 0x0F
    motor_id_fb = data[0] & 0x0F
    pos_uint = (data[1] << 8) | data[2]
    vel_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    pos = uint_to_float(pos_uint, -pmax, pmax, 16)
    vel = uint_to_float(vel_uint, -vmax, vmax, 12)
    tau = uint_to_float(tau_uint, -tmax, tmax, 12)
    return {
        "motor_id": motor_id_fb,
        "error_code": err_code,
        "error_name": DmError.name(err_code),
        "position": pos,
        "velocity": vel,
        "torque": tau,
        "position_raw": pos_uint,
        "velocity_raw": vel_uint,
        "torque_raw": tau_uint,
        "temp_mos": data[6],
        "temp_rotor": data[7],
        "can_id": can_id,
    }


# --- Damiao float<->uint conversion (from V1.1 manual page 15) ---

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Map float to unsigned int (Damiao MIT mode encoding)."""
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max
    span = x_max - x_min
    return int((x - x_min) * float((1 << bits) - 1) / span)

def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    """Map unsigned int to float (Damiao feedback decoding)."""
    span = x_max - x_min
    return float(u) * span / float((1 << bits) - 1) + x_min


# --- Streaming frame parser ---

class FrameParser:
    def __init__(self):
        self._buf = bytearray()
        self._discard_count = 0

    def feed(self, data: bytes):
        self._buf.extend(data)

    def next_frame(self) -> Optional[Frame]:
        while self._buf:
            hdr_idx = self._find_header()
            if hdr_idx < 0:
                self._discard_count += len(self._buf)
                self._buf.clear()
                return None
            if hdr_idx > 0:
                self._discard_count += hdr_idx
                del self._buf[:hdr_idx]
            if len(self._buf) < 6:
                return None
            version = self._buf[2]
            msg_type = self._buf[3]
            seq = self._buf[4]
            payload_len = self._buf[5]
            if payload_len > MAX_PAYLOAD:
                self._discard_count += 1
                del self._buf[:1]
                continue
            total = 2 + 4 + payload_len + 2 + 1
            if len(self._buf) < total:
                return None
            payload = bytes(self._buf[6:6 + payload_len])
            crc_recv = struct.unpack("<H", self._buf[6 + payload_len:8 + payload_len])[0]
            tail = self._buf[8 + payload_len]
            pre_crc = bytes(self._buf[:6 + payload_len])
            crc_calc = crc16(pre_crc)
            del self._buf[:total]
            if tail != FRAME_TAIL:
                self._discard_count += total
                continue
            if crc_recv != crc_calc:
                self._discard_count += total
                continue
            return Frame(version=version, msg_type=msg_type, seq=seq, payload=payload)
        return None

    def _find_header(self) -> int:
        for i in range(len(self._buf) - 1):
            if self._buf[i] == 0x55 and self._buf[i + 1] == 0xAA:
                return i
        if self._buf and self._buf[-1] == 0x55:
            return len(self._buf) - 1
        return -1

    @property
    def discard_count(self) -> int:
        return self._discard_count

    def reset(self):
        self._buf.clear()
        self._discard_count = 0


# --- Convenience builders ---

def build_frame(msg_type: int, seq: int, payload: bytes = b"",
                version: int = PROTOCOL_VERSION) -> Frame:
    return Frame(version=version, msg_type=msg_type, seq=seq, payload=payload)

def clamp_speed(value: float) -> float:
    return max(SAFE_SPEED_MIN, min(SAFE_SPEED_MAX, float(value)))

def clamp_duration_ms(value: int) -> int:
    return max(MIN_DURATION_MS, min(MAX_DURATION_MS, int(value)))

def is_valid_motor_id(motor_id: int, allowed: frozenset = DEFAULT_MOTOR_IDS) -> bool:
    return motor_id in allowed


# --- CAN Discovery protocol (section: DM CAN discovery) ---

DISCOVERY_START_PAYLOAD_LEN = 4
DISCOVERY_STATUS_PAYLOAD_LEN = 3
DISCOVERY_RESULT_PAYLOAD_LEN = 28

def encode_discovery_start(id_start: int, id_end: int) -> bytes:
    return struct.pack("<HH", id_start & 0xFFFF, id_end & 0xFFFF)

def decode_discovery_start(payload: bytes) -> dict:
    if len(payload) < DISCOVERY_START_PAYLOAD_LEN:
        raise ValueError("DISCOVERY_START payload too short")
    start, end = struct.unpack("<HH", payload[:DISCOVERY_START_PAYLOAD_LEN])
    return {"id_start": start, "id_end": end}

def encode_discovery_status(state: int, progress: int) -> bytes:
    return struct.pack("<BBB", state & 0xFF, progress & 0xFF, 0)

def decode_discovery_status(payload: bytes) -> dict:
    if len(payload) < 2:
        raise ValueError("DISCOVERY_STATUS payload too short")
    state, progress, _ = struct.unpack("<BBB", payload[:DISCOVERY_STATUS_PAYLOAD_LEN])
    state_names = {0: "IDLE", 1: "SCANNING", 2: "READING_PARAMS", 3: "DONE", 4: "TIMEOUT"}
    return {"state": state, "state_name": state_names.get(state, "UNKNOWN"),
            "progress": progress}

def encode_discovery_result(found: bool, esc_id: int, mst_id: int,
                             ctrl_mode: int, can_baud: int) -> bytes:
    flags = 1 if found else 0
    buf = struct.pack("<BHHBBx", flags, esc_id & 0xFFFF, mst_id & 0xFFFF,
                      ctrl_mode & 0xFF, can_baud & 0xFF)
    # Pad to DISCOVERY_RESULT_PAYLOAD_LEN (16 bytes) to match C-side
    buf += b"\x00" * (DISCOVERY_RESULT_PAYLOAD_LEN - len(buf))
    return buf

def decode_discovery_result(payload: bytes) -> dict:
    if len(payload) < 7:
        raise ValueError("DISCOVERY_RESULT payload too short")
    flags, esc_id, mst_id, ctrl_mode, can_baud = struct.unpack(
        "<BHHBB", payload[:7])
    sw_ver = struct.unpack("<I", payload[7:11])[0] if len(payload) >= 11 else 0
    pmax = struct.unpack("<f", payload[11:15])[0] if len(payload) >= 15 else 0.0
    vmax = struct.unpack("<f", payload[15:19])[0] if len(payload) >= 19 else 0.0
    tmax = struct.unpack("<f", payload[19:23])[0] if len(payload) >= 23 else 0.0
    return {"found": bool(flags & 1), "esc_id": esc_id, "mst_id": mst_id,
            "ctrl_mode": ctrl_mode, "can_baud_code": can_baud,
            "sw_version": sw_ver, "pmax": pmax, "vmax": vmax, "tmax": tmax}

# --- CAN channel selection ---

def encode_set_can_channel(channel: int) -> bytes:
    """0=both, 1=CAN1 only, 2=CAN2 only"""
    return struct.pack('<B', channel & 0xFF)

# --- MIT mode control ---

MIT_CTRL_PAYLOAD_LEN = 21

def encode_mit_ctrl(motor_id: int, p_des: float, v_des: float,
                    kp: float, kd: float, t_ff: float) -> bytes:
    return struct.pack('<Bfffff', motor_id & 0xFF,
                       p_des, v_des, kp, kd, t_ff)

def decode_mit_ctrl(payload: bytes) -> dict:
    if len(payload) < MIT_CTRL_PAYLOAD_LEN:
        raise ValueError('MIT_CTRL payload too short')
    mid, p, v, kp, kd, t = struct.unpack('<Bfffff', payload[:MIT_CTRL_PAYLOAD_LEN])
    return {'motor_id': mid, 'p_des': p, 'v_des': v,
            'kp': kp, 'kd': kd, 't_ff': t}

def encode_mit_stop(motor_id: int) -> bytes:
    return struct.pack('<B', motor_id & 0xFF)


# Chassis protocol v1. The enclosing frame supplies CRC and sequence.
# Payload: host monotonic milliseconds, vx/vy/wz, validity window, mode, flags.
CHASSIS_TWIST_FORMAT = "<IfffHBB"
CHASSIS_TWIST_PAYLOAD_LEN = struct.calcsize(CHASSIS_TWIST_FORMAT)
CHASSIS_CONTROL_MODE_VELOCITY = 1
CHASSIS_FLAG_HEADING_HOLD = 0x01


def encode_chassis_twist(
    host_monotonic_ms: int,
    vx_mps: float,
    vy_mps: float,
    wz_rad_s: float,
    valid_for_ms: int,
    control_mode: int = CHASSIS_CONTROL_MODE_VELOCITY,
    flags: int = 0,
) -> bytes:
    values = (float(vx_mps), float(vy_mps), float(wz_rad_s))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("chassis twist must be finite")
    if not 1 <= int(valid_for_ms) <= 1000:
        raise ValueError("valid_for_ms must be in [1, 1000]")
    if int(control_mode) != CHASSIS_CONTROL_MODE_VELOCITY:
        raise ValueError("unsupported chassis control mode")
    return struct.pack(
        CHASSIS_TWIST_FORMAT,
        int(host_monotonic_ms) & 0xFFFFFFFF,
        *values,
        int(valid_for_ms),
        int(control_mode),
        int(flags) & 0xFF,
    )


def decode_chassis_twist(payload: bytes) -> dict:
    if len(payload) != CHASSIS_TWIST_PAYLOAD_LEN:
        raise ValueError(
            f"CHASSIS_TWIST payload must be {CHASSIS_TWIST_PAYLOAD_LEN} bytes"
        )
    host_ms, vx, vy, wz, valid_ms, mode, flags = struct.unpack(
        CHASSIS_TWIST_FORMAT, payload
    )
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError("chassis twist must be finite")
    if not 1 <= valid_ms <= 1000:
        raise ValueError("invalid chassis command validity window")
    if mode != CHASSIS_CONTROL_MODE_VELOCITY:
        raise ValueError("unsupported chassis control mode")
    return {
        "host_monotonic_ms": host_ms,
        "vx_mps": vx,
        "vy_mps": vy,
        "wz_rad_s": wz,
        "valid_for_ms": valid_ms,
        "control_mode": mode,
        "flags": flags,
    }


DM_REGISTER_MASTER_ID = 7
DM_REGISTER_ESC_ID = 8


def encode_dm_param_read(esc_id: int, register: int) -> bytes:
    if not 0 <= int(esc_id) <= 0x7FF or not 0 <= int(register) <= 0xFF:
        raise ValueError("DM parameter address is out of range")
    return struct.pack("<HB", int(esc_id), int(register))


def encode_dm_param_write(esc_id: int, register: int, value: int) -> bytes:
    return encode_dm_param_read(esc_id, register) + struct.pack("<I", int(value) & 0xFFFFFFFF)


def encode_dm_param_save(esc_id: int) -> bytes:
    if not 0 <= int(esc_id) <= 0x7FF:
        raise ValueError("DM ESC ID is out of range")
    return struct.pack("<H", int(esc_id))


def decode_dm_param_result(payload: bytes) -> dict:
    if len(payload) != 10:
        raise ValueError("DM parameter result payload must be 10 bytes")
    reply_id, response_esc_id, opcode, register, value = struct.unpack("<HHBBI", payload)
    return {
        "reply_can_id": reply_id,
        "response_esc_id": response_esc_id,
        "opcode": opcode,
        "register": register,
        "value": value,
        "raw_payload_hex": payload.hex(),
    }


CHASSIS_WHEEL_NAMES = ("FL", "FR", "RL", "RR")
CHASSIS_STATUS_PAYLOAD_LEN = 248
CHASSIS_DIAGNOSTICS_PAYLOAD_LEN = 208
IMU_STATUS_PAYLOAD_LEN = 156

CHASSIS_PROFILE_SAFE = 0
CHASSIS_PROFILE_NORMAL = 1
CHASSIS_PROFILE_COMPETITION_CANDIDATE = 2


def encode_chassis_profile(profile: int) -> bytes:
    value = int(profile)
    if value not in (CHASSIS_PROFILE_SAFE, CHASSIS_PROFILE_NORMAL,
                     CHASSIS_PROFILE_COMPETITION_CANDIDATE):
        raise ValueError("unsupported chassis control profile")
    return bytes((value,))


def decode_chassis_diagnostics(payload: bytes) -> dict:
    if len(payload) != CHASSIS_DIAGNOSTICS_PAYLOAD_LEN:
        raise ValueError("CHASSIS_DIAGNOSTICS payload must be 208 bytes")
    version, profile, control_mode, reverse_mask, board_tick_ms = struct.unpack_from(
        "<BBBBI", payload, 0
    )
    command_age_ms, control_period_ms = struct.unpack_from("<HH", payload, 8)
    profile_values = struct.unpack_from("<8f", payload, 12)
    if not all(math.isfinite(value) and value > 0.0 for value in profile_values):
        raise ValueError("invalid chassis profile limits")
    names = (
        "normal_accel_rad_s2", "normal_decel_rad_s2", "reverse_brake_rad_s2",
        "normal_jerk_rad_s3", "reverse_jerk_rad_s3", "zero_speed_epsilon_rad_s",
        "reverse_detection_epsilon_rad_s", "velocity_deadband_rad_s",
    )
    limits = dict(zip(names, profile_values))
    wheels = []
    for index, wheel_name in enumerate(CHASSIS_WHEEL_NAMES):
        offset = 44 + index * 28
        motor_id, reverse_state, limiter_state, supported = struct.unpack_from("<BBBB", payload, offset)
        target, requested, measured, error, measured_torque = struct.unpack_from("<5f", payload, offset + 4)
        if not all(math.isfinite(value) for value in (target, requested, measured, error, measured_torque)):
            raise ValueError("non-finite chassis wheel diagnostics")
        feedback_age_ms, fault_code, enabled = struct.unpack_from("<HBB", payload, offset + 24)
        fresh = feedback_age_ms <= 100
        wheels.append({
            "wheel": wheel_name, "motor_id": motor_id, "enabled": bool(enabled),
            "control_mode": "MIT", "target_velocity_rad_s": target,
            "requested_velocity_rad_s": requested,
            "measured_velocity_rad_s": measured if fresh and supported & 0x01 else None,
            "velocity_error_rad_s": error if fresh and supported & 0x01 else None,
            "measured_torque_nm": measured_torque if fresh and supported & 0x02 else None,
            "measured_current_a": None, "bus_voltage_v": None,
            "feedback_age_ms": feedback_age_ms, "fault_code": fault_code,
            "reverse_state": reverse_state, "limiter_state": limiter_state,
            "saturation_flags": 1 if limiter_state == 3 else 0,
        })
    event_values = struct.unpack_from("<5I4f4B3I", payload, 156)
    event_id = event_values[0]
    reverse_event = None
    if event_id:
        def measured_time(value):
            return None if value in (0, 0xFFFFFFFF) else value
        reverse_event = {
            "event_id": event_id,
            "command_received_ms": event_values[1],
            "first_brake_command_ms": event_values[2],
            "zero_cross_ms": measured_time(event_values[3]),
            "reverse_velocity_reached_ms": measured_time(event_values[4]),
            "old_target_velocity_rad_s": event_values[5],
            "new_target_velocity_rad_s": event_values[6],
            "measured_velocity_at_command_rad_s": event_values[7],
            "peak_measured_torque_nm": event_values[8],
            "motor_id": event_values[9], "profile": event_values[10],
            "result": event_values[11], "fault_code": event_values[12],
            "command_to_brake_latency_ms": measured_time(event_values[13]),
            "brake_to_zero_cross_ms": measured_time(event_values[14]),
            "zero_cross_to_reverse_response_ms": measured_time(event_values[15]),
            "peak_current_a": None, "peak_bus_voltage_v": None,
        }
    return {
        "version": version, "profile": profile, "control_mode": control_mode,
        "reverse_mask": reverse_mask, "board_tick_ms": board_tick_ms,
        "command_age_ms": command_age_ms, "control_period_ms": control_period_ms,
        "limits": limits, "torque_limit_nm": None, "current_limit_a": None,
        "wheels": wheels, "latest_reverse_event": reverse_event,
        "raw_payload_hex": payload.hex(),
    }


def encode_imu_set_reference(angle_deg: float) -> bytes:
    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError("IMU yaw reference must be finite")
    return struct.pack("<f", angle)


def decode_imu_status(payload: bytes) -> dict:
    if len(payload) != IMU_STATUS_PAYLOAD_LEN:
        raise ValueError("IMU_STATUS payload must be 156 bytes")
    (sequence, timestamp_us, age_us, flags, sample_count, error_count,
     missed_sample_count, spi_error_count, movement_rejection_count,
     sample_interval_min_us, sample_interval_max_us,
     update_execution_max_us) = struct.unpack_from("<12I", payload, 0)
    state, calibration_state, source, accel_chip_id, gyro_chip_id = struct.unpack_from(
        "<5B", payload, 48
    )
    calibration_sample_count = struct.unpack_from("<H", payload, 54)[0]
    accel_mps2 = struct.unpack_from("<3f", payload, 56)
    gyro_rad_s = struct.unpack_from("<3f", payload, 68)
    mag = struct.unpack_from("<3f", payload, 80)
    quaternion_wxyz = struct.unpack_from("<4f", payload, 92)
    roll_rad, pitch_rad, yaw_rad, yaw_continuous_rad, gyro_z_rad_s, temperature_c = struct.unpack_from(
        "<6f", payload, 108
    )
    gyro_bias_rad_s = struct.unpack_from("<3f", payload, 132)
    calibration_gyro_variance = struct.unpack_from("<3f", payload, 144)
    floats = (*accel_mps2, *gyro_rad_s, *mag, *quaternion_wxyz, roll_rad,
              pitch_rad, yaw_rad, yaw_continuous_rad, gyro_z_rad_s,
              temperature_c, *gyro_bias_rad_s, *calibration_gyro_variance)
    if not all(math.isfinite(value) for value in floats):
        raise ValueError("IMU_STATUS contains NaN or Inf")
    return {
        "sequence": sequence, "cboard_timestamp_us": timestamp_us,
        "age_us": age_us, "flags": flags, "sample_count": sample_count,
        "error_count": error_count, "missed_sample_count": missed_sample_count,
        "spi_error_count": spi_error_count,
        "movement_rejection_count": movement_rejection_count,
        "sample_interval_min_us": sample_interval_min_us,
        "sample_interval_max_us": sample_interval_max_us,
        "update_execution_max_us": update_execution_max_us,
        "state": state, "calibration_state": calibration_state,
        "source_code": source, "accel_chip_id": accel_chip_id,
        "gyro_chip_id": gyro_chip_id,
        "calibration_sample_count": calibration_sample_count,
        "accel_mps2": accel_mps2, "gyro_rad_s": gyro_rad_s, "mag": mag,
        "quaternion_wxyz": quaternion_wxyz, "roll_rad": roll_rad,
        "pitch_rad": pitch_rad, "yaw_rad": yaw_rad,
        "yaw_continuous_rad": yaw_continuous_rad,
        "gyro_z_rad_s": gyro_z_rad_s, "temperature_c": temperature_c,
        "gyro_bias_rad_s": gyro_bias_rad_s,
        "calibration_gyro_variance": calibration_gyro_variance,
        "raw_payload_hex": payload.hex(),
    }


def decode_chassis_status(payload: bytes) -> dict:
    if len(payload) != CHASSIS_STATUS_PAYLOAD_LEN:
        raise ValueError("CHASSIS_STATUS payload must be 248 bytes")
    state, online_mask, enabled_mask, fault_mask, board_tick_ms, can_esr = struct.unpack_from(
        "<BBBBII", payload, 0
    )
    wheels = []
    for index, name in enumerate(CHASSIS_WHEEL_NAMES):
        offset = 12 + index * 52
        esc_id, master_id, flags, error_code, temp_mos, temp_rotor = struct.unpack_from(
            "<HHBBBB", payload, offset
        )
        position, speed, torque = struct.unpack_from("<fff", payload, offset + 8)
        position_raw, speed_raw, torque_raw = struct.unpack_from("<HHH", payload, offset + 20)
        feedback_sequence, feedback_tick_ms = struct.unpack_from("<II", payload, offset + 28)
        position_unwrapped, feedback_age_ms, rx_count, invalid_count, stale_count, last_raw_can_id = struct.unpack_from(
            "<fHIHHH", payload, offset + 36
        )
        wheels.append({
            "wheel": name, "esc_id": esc_id, "master_id": master_id,
            "enabled": bool(flags & 1), "online": bool(flags & 2),
            "fault": bool(flags & 4), "error_code": error_code,
            "temperature_mos_c": temp_mos, "temperature_rotor_c": temp_rotor,
            "position_rad": position, "speed_rad_s": speed, "torque_nm": torque,
            "position_raw": position_raw, "speed_raw": speed_raw,
            "torque_raw": torque_raw, "feedback_sequence": feedback_sequence,
            "feedback_tick_ms": feedback_tick_ms,
            "position_unwrapped_rad": position_unwrapped,
            "feedback_age_ms": feedback_age_ms,
            "rx_count": rx_count,
            "invalid_count": invalid_count,
            "stale_count": stale_count,
            "last_raw_can_id": last_raw_can_id,
        })
    control_sequence, control_tick_count, can_tx_failure_count, max_command_skew_us, scheduler_lateness_max_us, runtime_fault, id_mismatch_count = struct.unpack_from(
        "<IIIIIII", payload, 220
    )
    return {
        "state": state, "online_mask": online_mask, "enabled_mask": enabled_mask,
        "fault_mask": fault_mask, "board_tick_ms": board_tick_ms,
        "can_esr": can_esr, "wheels": wheels,
        "control_sequence": control_sequence,
        "control_tick_count": control_tick_count,
        "can_tx_failure_count": can_tx_failure_count,
        "max_command_skew_us": max_command_skew_us,
        "scheduler_lateness_max_us": scheduler_lateness_max_us,
        "runtime_fault": runtime_fault,
        "id_mismatch_count": id_mismatch_count,
    }
