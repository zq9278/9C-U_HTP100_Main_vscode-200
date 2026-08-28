"""HTP100 pressure tuning protocol shared on USART2.

Tuning frames use 7A A7. The real screen keeps using 5A A5 / 6A A6.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Tuple


HEADER = b"\x7A\xA7"
VERSION = 1
TAIL = b"\x0D\x0A"
MAX_PAYLOAD = 64

CMD_HELLO = 0x01
CMD_GET_ALL = 0x10
CMD_GET_PROFILE = 0x11
CMD_SET_PROFILE = 0x12
CMD_SAVE = 0x13
CMD_DEFAULTS = 0x14
CMD_SET_FIELD = 0x15
CMD_SET_TARGET = 0x20
CMD_PREPARE = 0x21
CMD_START = 0x22
CMD_STOP = 0x23
CMD_TELEMETRY_CONTROL = 0x24
CMD_GET_STATUS = 0x25
CMD_GET_HEAT_PID = 0x26
CMD_SET_HEAT_PID = 0x27
CMD_SET_TEMPERATURE = 0x28
CMD_PREPARE_HEAT = 0x29
CMD_DEBUG_CONTROL = 0x30

RSP_INFO = 0x81
RSP_PROFILE = 0x90
RSP_ACK = 0x91
RSP_HEAT_PID = 0x92
RSP_TELEMETRY = 0xA0
RSP_HOME_EVENT = 0xA1

STATUS_TEXT = {
    0: "成功",
    1: "负载长度错误",
    2: "参数超范围",
    3: "当前状态拒绝",
    4: "保存失败",
    5: "设备不在待机状态",
    6: "充电联锁禁止治疗",
    7: "设备存在故障",
    8: "电机尚未成功回零",
    9: "眼盾缺失、已报废或状态无效",
    10: "ADS1220 零点无效",
}

PROFILE_KEYS = (
    "sensitivity_mv_v",
    "offset_mmhg",
    "fast_speed",
    "approach_speed",
    "retract_speed",
    "approach_threshold",
    "hold_threshold",
    "hold_ms",
    "retract_ms",
    "kp",
    "ki",
)


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return value & 0xFFFF


def build_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    prefix = HEADER + bytes((VERSION, command & 0xFF, sequence & 0xFF, len(payload))) + payload
    return prefix + struct.pack("<H", crc16(prefix)) + TAIL


def build_screen_work_frame(command: int, value: float = 0.0) -> bytes:
    """Build one existing 5A A5 screen work-command frame."""
    prefix = b"\x5A\xA5\x0D" + struct.pack(">H", command) + struct.pack("<f", value)
    return prefix + struct.pack("<H", crc16(prefix)) + b"\xFF\xFF"


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    payload: bytes


class FrameParser:
    """Extract tuning frames while safely ignoring real-screen traffic."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> Tuple[List[Frame], int]:
        self.buffer.extend(data)
        frames: List[Frame] = []
        ignored = 0
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                keep = 1 if self.buffer.endswith(HEADER[:1]) else 0
                ignored += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                ignored += start
                del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            version = self.buffer[2]
            payload_length = self.buffer[5]
            if version != VERSION or payload_length > MAX_PAYLOAD:
                ignored += 1
                del self.buffer[0]
                continue
            total = 10 + payload_length
            if len(self.buffer) < total:
                break
            candidate = bytes(self.buffer[:total])
            crc_at = 6 + payload_length
            expected = struct.unpack_from("<H", candidate, crc_at)[0]
            if candidate[-2:] != TAIL or expected != crc16(candidate[:crc_at]):
                ignored += 1
                del self.buffer[0]
                continue
            frames.append(Frame(candidate[3], candidate[4], candidate[6:crc_at]))
            del self.buffer[:total]
        return frames, ignored


def encode_profile(index: int, values: dict) -> bytes:
    numbers = [float(values[key]) for key in PROFILE_KEYS]
    return struct.pack("<B11f", index, *numbers)


def encode_profile_field(profile_index: int, field_key: str, value: float) -> bytes:
    field_index = PROFILE_KEYS.index(field_key)
    return struct.pack("<BBf", profile_index, field_index, float(value))


def decode_profile(payload: bytes) -> Tuple[int, dict]:
    if len(payload) != 45:
        raise ValueError("invalid profile payload")
    unpacked = struct.unpack("<B11f", payload)
    return unpacked[0], dict(zip(PROFILE_KEYS, unpacked[1:]))


def encode_heat_pid(kp: float, ki: float, kd: float) -> bytes:
    return struct.pack("<fff", float(kp), float(ki), float(kd))


def decode_heat_pid(payload: bytes) -> dict:
    if len(payload) != 12:
        raise ValueError("invalid heat PID payload")
    return dict(zip(("kp", "ki", "kd"), struct.unpack("<fff", payload)))


def decode_telemetry(payload: bytes) -> dict:
    if len(payload) not in (27, 36, 45, 53):
        raise ValueError("invalid telemetry payload")
    values = struct.unpack("<IffiiBBBHBB", payload[:27])
    keys = (
        "timestamp_ms", "target_mmhg", "pressure_mmhg", "raw", "zero_raw",
        "stage", "active", "app_state", "fault", "charging", "zero_valid",
    )
    result = dict(zip(keys, values))
    if len(payload) >= 36:
        extra = struct.unpack("<BBBBBHH", payload[27:36])
        extra_keys = (
            "eye", "home_valid", "stop_reason", "charge_full", "debug_mode",
            "battery_soc", "battery_mv",
        )
        result.update(zip(extra_keys, extra))
    if len(payload) >= 45:
        target_c, measured_c, valid = struct.unpack("<ffB", payload[36:45])
        result.update(
            target_temperature_c=target_c,
            temperature_c=measured_c,
            temperature_valid=valid,
        )
    if len(payload) == 53:
        power_percent, integral_output = struct.unpack("<ff", payload[45:53])
        result.update(
            heat_power_percent=power_percent,
            heat_integral_output=integral_output,
        )
    return result


def decode_home_event(payload: bytes) -> dict:
    if len(payload) != 9:
        raise ValueError("invalid home event payload")
    values = struct.unpack("<IBBHB", payload)
    keys = ("timestamp_ms", "success", "stop_reason", "fault", "zero_valid")
    return dict(zip(keys, values))
