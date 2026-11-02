"""codec（符号化・復号）の単体テスト。実機なしで動く。"""
import math
import struct

import pytest

from gds34can import codec
from gds34can.constants import Cmd, Conf, Fault, Ind, Result

KT, GR = 0.08472000062465668, 10   # 実機のドライバ設定値（GIM4310-10、2026-10-24 に読み出し）


def test_frames_are_8_bytes_little_endian():
    assert codec.encode_simple(Cmd.VERSION) == bytes([0xB1, 0, 0, 0, 0, 0, 0, 0])
    assert codec.encode_indicator(Ind.IQ) == bytes([0xB4, 0x09, 0, 0, 0, 0, 0, 0])
    assert codec.encode_read_conf(Conf.TORQUE_CONSTANT) == bytes([0x84, 0x01, 0x03, 0, 0, 0, 0, 0])


def test_write_conf_layout():
    data = codec.encode_write_conf(Conf.MAX_CURRENT, 2)
    assert data[:4] == bytes([0x83, 0x00, 0x01, 0x00])
    assert struct.unpack("<i", data[4:8])[0] == 2
    data = codec.encode_write_conf(Conf.TORQUE_CONSTANT, 0.1)
    assert struct.unpack("<f", data[4:8])[0] == pytest.approx(0.1)
    with pytest.raises(ValueError):
        codec.encode_write_conf(Conf.MAX_CURRENT, 2.5)   # int32 の項目に小数


def test_control_layout():
    # 実機を回したときの bring-up 用スクリプトと同じ並び: [CMD | float32 LE | uint24 LE ms]
    data = codec.encode_control(Cmd.VELOCITY, 10.0, 300)
    assert data == bytes([0x94]) + struct.pack("<f", 10.0) + (300).to_bytes(3, "little")


@pytest.mark.parametrize("bad", [0, -1, 0x1000000])
def test_control_duration_range(bad):
    with pytest.raises(ValueError):
        codec.encode_control(Cmd.VELOCITY, 1.0, bad)


def test_seconds_to_duration_rejects_zero():
    assert codec.seconds_to_duration_ms(0.3) == 300
    with pytest.raises(ValueError):
        codec.seconds_to_duration_ms(0.0)   # Duration = 0 の挙動は未確認（V3）


def test_decode_control_matches_logged_idle_values():
    # 停止中の実機の応答（2026-10-24 のログの 1 行目: speed=-0.016, torque=-0.047）は raw 2047 に当たる
    raw_v = raw_t = 2047
    data = bytes([0x94, 0x00, 30, 0x00, 0x80, raw_v >> 4, ((raw_v & 0xF) << 4) | (raw_t >> 8), raw_t & 0xFF])
    s = codec.decode_control(data, KT, GR)
    assert s.result is Result.SUCCESS and s.temperature_c == 30
    assert s.velocity_rad_s == pytest.approx(-0.016, abs=5e-4)
    assert s.torque_nm == pytest.approx(-0.047, abs=5e-4)
    assert s.position_rad == pytest.approx(32768 * 25 / 65535 - 12.5)


def test_decode_control_without_kt_has_no_torque():
    assert codec.decode_control(bytes(8)).torque_nm is None


def test_control_reply_round_trip_within_one_lsb():
    data = codec.encode_control_reply(Cmd.POSITION, 0, 41, 1.234, -2.5, 0.3, KT, GR)
    s = codec.decode_control(data, KT, GR)
    assert s.position_rad == pytest.approx(1.234, abs=25 / 65535)
    assert s.velocity_rad_s == pytest.approx(-2.5, abs=130 / 4095)
    assert s.torque_nm == pytest.approx(0.3, abs=codec.torque_scale(KT, GR) / 4095)
    assert s.temperature_c == 41


def test_control_reply_position_wraps_like_the_driver():
    # 実機では 11.504 → -12.432 と ±12.5 rad で折り返した（2026-10-24 のログ）
    s = codec.decode_control(codec.encode_control_reply(Cmd.VELOCITY, 0, 30, 13.0, 0, 0, KT, GR), KT, GR)
    assert s.position_rad == pytest.approx(13.0 - 25.0, abs=1e-3)


def test_match_key_includes_sub_ids():
    assert codec.match_key(codec.encode_indicator(Ind.IQ)) != codec.match_key(codec.encode_indicator(Ind.ID))
    req = codec.encode_read_conf(Conf.CAN_ID)
    reply = bytes([0x84, 0x00, 0x12, 0x00, 1, 0, 0, 0])
    assert codec.match_key(req) == codec.match_key(reply)
    assert codec.match_key(codec.encode_read_conf(Conf.HOST_CAN_ID)) != codec.match_key(reply)


def test_result_position_depends_on_command():
    assert codec.result_of(bytes([0x84, 0, 0x12, 0x05, 0, 0, 0, 0])) is Result.UNKNOWN_REGISTER
    assert codec.result_of(bytes([0xB4, 0x09, 0x00, 0, 0, 0, 0, 0])) is Result.SUCCESS
    assert codec.result_of(bytes([0x91, 0x01, 0, 0, 0, 0, 0, 0])) is Result.FAILURE
    assert codec.result_of(bytes([0x91, 0x7E, 0, 0, 0, 0, 0, 0])) == 0x7E   # 未知のコードは int のまま


def test_decode_fault_flags():
    f = codec.decode_fault(bytes([0xB2, 0x00, 0x44, 0, 0, 0, 0, 0]))
    assert Fault.UNDER_VOLTAGE in f and Fault.OVER_CURRENT in f and Fault.OVER_VOLTAGE not in f


def test_decode_conf_and_indicator():
    conf, value = codec.decode_conf(bytes([0x84, 0x01, 0x03, 0x00]) + struct.pack("<f", KT))
    assert conf is Conf.TORQUE_CONSTANT and value == pytest.approx(KT)
    assert codec.decode_indicator(bytes([0xB4, 0x09, 0, 0]) + struct.pack("<f", 0.14)) == pytest.approx(0.14)


def test_units():
    assert codec.rad_s_to_rpm(2 * math.pi) == pytest.approx(60.0)
    assert codec.rpm_to_rad_s(10) == pytest.approx(1.0472, abs=1e-4)


def test_conf_table_is_unique():
    keys = [(c.spec.conf_type, c.spec.conf_id) for c in Conf]
    assert len(keys) == len(set(keys))
