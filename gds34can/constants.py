"""GDS34（SteadyWin GIM プロトコル rev2.2）の定数。

出典: SteadyWin "Motor Driver Protocol Specification" rev2.2 (2023-08-30)
      （EN_GIM_Motor_Driver_Protocol_Specification_rev2.2.pdf）
実機で確かめた点・資料と食い違った点は README「実機で確かめたこと」。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass


class Cmd(enum.IntEnum):
    """フレームの BYTE0（コマンド）。"""

    RESET_CONF = 0x81       # 送信禁止: 全設定のリセット → 相順識別・エンコーダ校正のやり直しが必要
    APPLY_CONF = 0x82
    WRITE_CONF = 0x83
    READ_CONF = 0x84
    START = 0x91
    STOP = 0x92
    TORQUE = 0x93
    VELOCITY = 0x94
    POSITION = 0x95
    PTS = 0x96              # GDS34 は非対応
    ABORT = 0x97
    WRITE_PID = 0xA1
    READ_PID = 0xA2
    VERSION = 0xB1
    FAULT = 0xB2
    CLEAR_FAULT = 0xB3
    INDICATOR = 0xB4
    CALIBRATE = 0xB5        # 送信禁止: モータが正逆に回る
    FIRMWARE_UPDATE = 0xC1  # 送信禁止: 応答せずにファーム更新モードに入る


#: ライブラリから送らないコマンド
BLOCKED_COMMANDS = frozenset({Cmd.RESET_CONF, Cmd.PTS, Cmd.CALIBRATE, Cmd.FIRMWARE_UPDATE})

#: 応答が制御状態 [CMD|RES|Temp|Pos0|Pos1|ST0|ST1|ST2] になるコマンド
CONTROL_COMMANDS = frozenset({Cmd.TORQUE, Cmd.VELOCITY, Cmd.POSITION})


class Result(enum.IntEnum):
    """応答の結果コード（RES）。"""

    SUCCESS = 0x00
    FAILURE = 0x01
    UNKNOWN_COMMAND = 0x02
    UNKNOWN_ID = 0x03
    READ_ONLY = 0x04
    UNKNOWN_REGISTER = 0x05
    STRING_FORMAT = 0x06
    DATA_FORMAT = 0x07
    WRITE_ONLY = 0x0B


class Fault(enum.IntFlag):
    """故障コード（0xB2 の FaultNo）。ビットフラグなので複数が同時に立ちうる。"""

    NONE = 0x00
    FOC_FREQUENCY = 0x01
    OVER_VOLTAGE = 0x02
    UNDER_VOLTAGE = 0x04
    OVER_TEMPERATURE = 0x08
    START_FAILURE = 0x10
    OVER_CURRENT = 0x40
    SOFTWARE = 0x80


class Ind(enum.IntEnum):
    """インジケータ（0xB4 の IndID）。値はすべて float32。"""

    BUS_VOLTAGE = 0x00        # [V]   ファーム 0x21 では更新されない（2.552772 で固定）
    DRIVER_TEMP = 0x01
    MOTOR_TEMP = 0x02
    POWER = 0x03              # [W]   ファーム 0x21 では更新されない（常に 0）
    IA = 0x04                 # [A]
    IB = 0x05
    IC = 0x06
    I_ALPHA = 0x07
    I_BETA = 0x08
    IQ = 0x09                 # [A]
    ID = 0x0A
    IQ_TARGET = 0x0B
    ID_TARGET = 0x0C
    VQ = 0x0D                 # [V]
    VD = 0x0E
    V_ALPHA = 0x0F
    V_BETA = 0x10
    ROTOR_ELEC_ANGLE = 0x11   # [rad]
    ROTOR_MECH_ANGLE = 0x12   # [rad]
    OUTPUT_ANGLE = 0x13       # [rad] 出力軸。多回転（折り返さない）
    OUTPUT_SPEED_RPM = 0x14   # [RPM] 出力軸
    OUTPUT_POWER = 0x15       # [W]


#: 値が更新されないインジケータ（ファーム 0x21 の実機で確認。README「実機で確かめたこと」）
STALE_INDICATORS = frozenset({Ind.BUS_VOLTAGE, Ind.POWER})


class Guard(enum.Enum):
    """設定項目の書き込みに対する保護の種類。"""

    NONE = "none"
    #: モータの物理パラメータ。実物と違う値を書くと正常に回らない（Motor Wizard マニュアル）
    MOTOR_MODEL = "motor_model"
    #: 書いた瞬間に反映されて通信が切れる。Bus.change_id / change_host_id でだけ書く
    COMMUNICATION = "communication"
    READ_ONLY = "read_only"


@dataclass(frozen=True)
class ConfSpec:
    conf_type: int   # 0x00 = int32, 0x01 = float32
    conf_id: int
    unit: str
    guard: Guard
    applies_after_restart: bool = False
    note: str = ""


class Conf(enum.Enum):
    """設定項目（0x83 / 0x84）。プロトコル rev2.2 Table 7。"""

    POLE_PAIRS = ConfSpec(0x00, 0x00, "", Guard.MOTOR_MODEL)
    MAX_CURRENT = ConfSpec(0x00, 0x01, "A", Guard.NONE,
                           note="プロトコル上の名前は Rated Current。Motor Wizard の「最大电流」。出荷時 8")
    MAX_SPEED = ConfSpec(0x00, 0x02, "RPM", Guard.NONE, note="ロータ側の回転数")
    RATED_VOLTAGE = ConfSpec(0x00, 0x06, "V", Guard.NONE)
    PWM_FREQUENCY = ConfSpec(0x00, 0x07, "Hz", Guard.NONE, applies_after_restart=True)
    CURRENT_KP = ConfSpec(0x00, 0x08, "", Guard.NONE)
    CURRENT_KI = ConfSpec(0x00, 0x09, "", Guard.NONE)
    SPEED_KP = ConfSpec(0x00, 0x0C, "", Guard.NONE)
    SPEED_KI = ConfSpec(0x00, 0x0D, "", Guard.NONE)
    POSITION_KP = ConfSpec(0x00, 0x0E, "", Guard.NONE)
    POSITION_KI = ConfSpec(0x00, 0x0F, "", Guard.NONE,
                           note="Motor Wizard マニュアル: ほとんどの場合 0 にすべき（振動で損傷しうる）")
    POSITION_KD = ConfSpec(0x00, 0x10, "", Guard.NONE)
    GEAR_RATIO = ConfSpec(0x00, 0x11, "", Guard.MOTOR_MODEL)
    CAN_ID = ConfSpec(0x00, 0x12, "", Guard.COMMUNICATION)
    HOST_CAN_ID = ConfSpec(0x00, 0x13, "", Guard.COMMUNICATION)
    ZERO_POSITION = ConfSpec(0x00, 0x14, "raw16", Guard.NONE,
                             note="出力軸。rad = 値 × 2π / 65536。単回転か多回転かは資料で食い違う")
    POWER_OFF_POSITION = ConfSpec(0x00, 0x15, "raw16", Guard.READ_ONLY)
    OVER_VOLTAGE = ConfSpec(0x00, 0x16, "V", Guard.NONE)
    UNDER_VOLTAGE = ConfSpec(0x00, 0x17, "V", Guard.NONE)
    CAN_BITRATE = ConfSpec(0x00, 0x18, "x100kbps", Guard.COMMUNICATION,
                           note="5 = 500kbps は実機で確認。ほかの値の対応は未確認")
    FIELD_WEAKENING_KP = ConfSpec(0x00, 0x19, "", Guard.NONE)
    FIELD_WEAKENING_KI = ConfSpec(0x00, 0x1A, "", Guard.NONE)
    PROTOCOL = ConfSpec(0x00, 0x1C, "", Guard.COMMUNICATION, note="0 = GIM（このライブラリ）/ 1 = MIT")
    OVER_TEMPERATURE = ConfSpec(0x00, 0x20, "", Guard.NONE)
    PHASE_RESISTANCE = ConfSpec(0x01, 0x00, "Ohm", Guard.MOTOR_MODEL)
    PHASE_INDUCTANCE = ConfSpec(0x01, 0x01, "H", Guard.MOTOR_MODEL)
    BACK_EMF = ConfSpec(0x01, 0x02, "Vrms/kRPM", Guard.MOTOR_MODEL)
    TORQUE_CONSTANT = ConfSpec(0x01, 0x03, "N.m/A", Guard.MOTOR_MODEL,
                               note="モータ軸の値（GIM4310-10 で 0.08472）。出力軸の仕様値ではない")
    SHUNT_RESISTANCE = ConfSpec(0x01, 0x04, "Ohm", Guard.MOTOR_MODEL)
    AMP_GAIN = ConfSpec(0x01, 0x05, "", Guard.MOTOR_MODEL)

    @property
    def spec(self) -> ConfSpec:
        return self.value

    @property
    def is_float(self) -> bool:
        return self.value.conf_type == 0x01

    @classmethod
    def lookup(cls, conf_type: int, conf_id: int) -> "Conf":
        for c in cls:
            if c.value.conf_type == conf_type and c.value.conf_id == conf_id:
                return c
        raise KeyError(f"unknown conf (type=0x{conf_type:02X}, id=0x{conf_id:02X})")
