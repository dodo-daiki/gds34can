"""フレームの符号化と応答の復号（通信を伴わない純粋な関数）。

フレームは CAN 2.0 標準フレーム・DLC 8・リトルエンディアン。BYTE0 がコマンドで、応答の BYTE0 は同じコマンドのエコー。
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from .constants import CONTROL_COMMANDS, Cmd, Conf, Fault, Ind, Result
from .errors import ProtocolError

FRAME_LEN = 8
MIT_REPLY_LEN = 6        # MIT プロトコルの応答長
MAX_DURATION_MS = 0xFFFFFF

# 制御応答の換算範囲（プロトコル rev2.2 §3.2.7）
POS_RANGE_RAD = 25.0     # ±12.5 rad
VEL_RANGE_RAD_S = 130.0  # ±65 rad/s


# --- 単位 ---------------------------------------------------------------

def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def seconds_to_duration_ms(seconds: float) -> int:
    """Duration [s] → プロトコルの Duration [ms]（24bit）。0 の扱いは未確認（V3）なので受け付けない。"""
    ms = round(seconds * 1000.0)
    if not 1 <= ms <= MAX_DURATION_MS:
        raise ValueError(f"duration must be 0.001..{MAX_DURATION_MS / 1000:.3f} s (got {seconds} s)")
    return ms


# --- 符号化 ---------------------------------------------------------------

def _frame(*payload: int) -> bytes:
    return bytes(payload).ljust(FRAME_LEN, b"\x00")


def encode_simple(cmd: Cmd) -> bytes:
    """ペイロードのないコマンド（0x82, 0x91, 0x92, 0x97, 0xB1, 0xB2, 0xB3）。"""
    return _frame(cmd)


def encode_indicator(ind: Ind) -> bytes:
    return _frame(Cmd.INDICATOR, ind)


def encode_read_conf(conf: Conf) -> bytes:
    return _frame(Cmd.READ_CONF, conf.spec.conf_type, conf.spec.conf_id)


def pack_conf_value(conf: Conf, value: float) -> bytes:
    if conf.is_float:
        return struct.pack("<f", float(value))
    if int(value) != value:
        raise ValueError(f"{conf.name} is int32 (got {value!r})")
    return struct.pack("<i", int(value))


def unpack_conf_value(conf: Conf, data4: bytes) -> int | float:
    return struct.unpack("<f" if conf.is_float else "<i", bytes(data4))[0]


def encode_write_conf(conf: Conf, value: float) -> bytes:
    """[0x83 | ConfType | ConfID | NULL | DATA0-3]"""
    return bytes([Cmd.WRITE_CONF, conf.spec.conf_type, conf.spec.conf_id, 0]) + pack_conf_value(conf, value)


def encode_control(cmd: Cmd, value: float, duration_ms: int) -> bytes:
    """[CMD | float32 | uint24 Duration]。値はプロトコルの単位（0x93: N·m, 0x94: RPM, 0x95: rad）。"""
    if cmd not in CONTROL_COMMANDS:
        raise ValueError(f"{cmd!r} is not a control command")
    if not math.isfinite(value):
        raise ValueError(f"value must be finite (got {value!r})")
    if not 1 <= duration_ms <= MAX_DURATION_MS:
        raise ValueError(f"duration_ms out of range: {duration_ms}")
    return bytes([cmd]) + struct.pack("<f", value) + int(duration_ms).to_bytes(3, "little")


# --- 応答の照合と結果コード ---------------------------------------------------

def match_key(data: bytes) -> tuple[int, ...]:
    """要求と応答を対応づけるキー。コマンドに加えて副 ID まで含める。

    副 ID の位置は要求と応答で同じ: 0x83/0x84 は BYTE1-2（ConfType, ConfID）、0xA1/0xA2/0xB4 は BYTE1。
    """
    cmd = data[0]
    if cmd in (Cmd.READ_CONF, Cmd.WRITE_CONF):
        return (cmd, data[1], data[2])
    if cmd in (Cmd.INDICATOR, Cmd.READ_PID, Cmd.WRITE_PID):
        return (cmd, data[1])
    return (cmd,)


def _result_index(cmd: int) -> int:
    if cmd in (Cmd.READ_CONF, Cmd.WRITE_CONF):
        return 3
    if cmd in (Cmd.INDICATOR, Cmd.READ_PID, Cmd.WRITE_PID, Cmd.CALIBRATE):
        return 2
    return 1


def parse_result(code: int) -> Result | int:
    try:
        return Result(code)
    except ValueError:
        return code


def result_of(data: bytes) -> Result | int:
    """応答の結果コード（RES）。位置はコマンドによって違う。"""
    return parse_result(data[_result_index(data[0])])


def check_reply_shape(data: bytes) -> None:
    if len(data) != FRAME_LEN:
        raise ProtocolError(f"reply must be {FRAME_LEN} bytes (got {len(data)}: {bytes(data).hex(' ')})")


# --- 応答の復号 ---------------------------------------------------------------

def decode_version(data: bytes) -> int:
    """0xB1: [0xB1 | RES | NULL | NULL | uint32]"""
    return int.from_bytes(bytes(data[4:8]), "little")


def decode_fault(data: bytes) -> Fault:
    """0xB2: [0xB2 | RES | FaultNo]"""
    return Fault(data[2])


def decode_indicator(data: bytes) -> float:
    """0xB4: [0xB4 | IndID | RES | NULL | float32]"""
    return struct.unpack("<f", bytes(data[4:8]))[0]


def decode_conf(data: bytes) -> tuple[Conf, int | float]:
    """0x84: [0x84 | ConfType | ConfID | RES | DATA0-3]"""
    conf = Conf.lookup(data[1], data[2])
    return conf, unpack_conf_value(conf, data[4:8])


@dataclass(frozen=True)
class ControlState:
    """制御指令（0x93/0x94/0x95）の応答。位置・速度・トルクは出力軸の値。"""

    result: Result | int
    temperature_c: int        # モータとドライバのうち高い方
    position_rad: float       # ±12.5 rad で折り返す。多回転の角度は Ind.OUTPUT_ANGLE で読む
    velocity_rad_s: float
    torque_nm: float | None   # Kt・減速比が分からないときは None
    raw_position: int
    raw_velocity: int
    raw_torque: int
    timestamp: float | None = None   # 受信時刻 [s]。Motor 経由で得た場合に入る


def torque_scale(kt: float, gear_ratio: float) -> float:
    """トルク欄の全幅 [N·m]（= 450·Kt·GR）。1LSB はこれを 4095 で割った値。"""
    return 450.0 * kt * gear_ratio


def decode_control(data: bytes, kt: float | None = None, gear_ratio: float | None = None) -> ControlState:
    """[CMD | RES | Temp | Pos0 | Pos1 | ST0 | ST1 | ST2]

    位置 = int16 × 25/65535 − 12.5、速度 = int12 × 130/4095 − 65、
    トルク = int12 × 450·Kt·GR/4095 − 225·Kt·GR（Kt はドライバ内部の値 = モータ軸）。
    """
    pos_int = data[3] | (data[4] << 8)
    vel_int = (data[5] << 4) | (data[6] >> 4)
    tq_int = ((data[6] & 0x0F) << 8) | data[7]
    torque = None
    if kt is not None and gear_ratio is not None:
        full = torque_scale(kt, gear_ratio)
        torque = tq_int * full / 4095.0 - full / 2.0
    return ControlState(
        result=parse_result(data[1]),
        temperature_c=data[2],
        position_rad=pos_int * POS_RANGE_RAD / 65535.0 - POS_RANGE_RAD / 2.0,
        velocity_rad_s=vel_int * VEL_RANGE_RAD_S / 4095.0 - VEL_RANGE_RAD_S / 2.0,
        torque_nm=torque,
        raw_position=pos_int,
        raw_velocity=vel_int,
        raw_torque=tq_int,
    )


def encode_control_reply(cmd: Cmd, result: int, temperature_c: int, position_rad: float,
                         velocity_rad_s: float, torque_nm: float, kt: float, gear_ratio: float) -> bytes:
    """decode_control の逆（擬似ドライバとテスト用）。位置は ±12.5 rad で折り返す。"""
    half = POS_RANGE_RAD / 2.0
    pos = (position_rad + half) % POS_RANGE_RAD - half
    pos_int = min(65535, max(0, round((pos + half) * 65535.0 / POS_RANGE_RAD)))
    vel_int = min(4095, max(0, round((velocity_rad_s + VEL_RANGE_RAD_S / 2.0) * 4095.0 / VEL_RANGE_RAD_S)))
    full = torque_scale(kt, gear_ratio)
    tq_int = min(4095, max(0, round((torque_nm + full / 2.0) * 4095.0 / full)))
    return bytes([cmd, result, max(0, min(255, int(temperature_c))), pos_int & 0xFF, pos_int >> 8,
                  vel_int >> 4, ((vel_int & 0x0F) << 4) | (tq_int >> 8), tq_int & 0xFF])
