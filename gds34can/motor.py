"""ドライバ 1 台の操作。

値はすべて出力軸の SI 単位（rad, rad/s, N·m, s）で受け渡す。プロトコルの RPM・ms への換算は中で行う。

通信を伴うのは read_* / write_* / set_* と start / stop / abort / clear_fault / apply_conf だけ。
cmd_* は要求を作るだけで通信しない（Bus.transact_many にまとめて渡すため）。
"""
from __future__ import annotations

import contextlib
import dataclasses
import math
import warnings
from typing import TYPE_CHECKING, Iterable, Iterator

from . import codec
from .codec import ControlState
from .constants import STALE_INDICATORS, Cmd, Conf, Fault, Guard, Ind, Result
from .errors import CommandRejected, FaultActive, Gds34Error, GuardedSetting, LimitExceeded, VerifyFailed

if TYPE_CHECKING:
    from .bus import Bus, Command, Reply

#: 動作を確認したファームウェアの版（0xB1 の値）
TESTED_FIRMWARE = frozenset({0x21})


class StaleIndicatorWarning(UserWarning):
    """値が更新されないことが分かっているインジケータを読んだ。"""


class Motor:
    """GDS34 ドライバ 1 台。Bus.motor(motor_id) で作る。"""

    def __init__(self, bus: "Bus", motor_id: int, *, host_id: int = 0, default_duration_s: float = 0.3,
                 max_velocity_rad_s: float | None = None, max_torque_nm: float | None = None,
                 read_params: bool = True):
        """
        host_id: ドライバに設定されたホスト ID（応答の送信元）。出荷時は 0
        default_duration_s: 制御指令の Duration を省略したときの値。速度・トルク指令はこの時間で止まる想定
            （README の V3 で未確認）。周期的に送り直す場合は、周期の数倍にする
        max_velocity_rad_s / max_torque_nm: 指令の上限（任意。超えると LimitExceeded）
        read_params: True ならトルクの復号に使う Kt・減速比をここで読む（応答がなければ NoReply）
        """
        if not 0x001 <= motor_id <= 0x7FF or motor_id == host_id:
            raise ValueError(f"invalid motor id 0x{motor_id:03X} (host id 0x{host_id:03X})")
        self.bus = bus
        self.id = motor_id
        self.host_id = host_id
        self.default_duration_s = default_duration_s
        self.max_velocity_rad_s = max_velocity_rad_s
        self.max_torque_nm = max_torque_nm
        self.kt: float | None = None          # ドライバ内部のトルク定数（モータ軸）[N·m/A]
        self.gear_ratio: int | None = None
        if read_params:
            self.refresh_params()

    def __repr__(self) -> str:
        return f"Motor(id=0x{self.id:03X}, host_id=0x{self.host_id:03X}, gear_ratio={self.gear_ratio}, kt={self.kt})"

    def refresh_params(self) -> None:
        """トルクの復号に使う Kt・減速比を読み直す。"""
        self.gear_ratio = int(self.read_conf(Conf.GEAR_RATIO))
        self.kt = float(self.read_conf(Conf.TORQUE_CONSTANT))

    # --- 要求を作る（通信しない） ------------------------------------------------

    def _command(self, data: bytes, label: str, decode) -> "Command":
        from .bus import Command, Request
        return Command(Request(self.id, data, self.host_id, label), decode)

    def _check(self, reply: "Reply", label: str, state=None) -> None:
        res = codec.result_of(reply.data)
        if res != Result.SUCCESS:
            raise CommandRejected(self.id, label, res, state)

    def _simple(self, cmd: Cmd) -> "Command[None]":
        return self._command(codec.encode_simple(cmd), cmd.name, lambda r: self._check(r, cmd.name))

    def cmd_start(self) -> "Command[None]":
        return self._simple(Cmd.START)

    def cmd_stop(self) -> "Command[None]":
        return self._simple(Cmd.STOP)

    def cmd_abort(self) -> "Command[None]":
        return self._simple(Cmd.ABORT)

    def cmd_clear_fault(self) -> "Command[None]":
        return self._simple(Cmd.CLEAR_FAULT)

    def cmd_version(self) -> "Command[int]":
        def decode(r):
            self._check(r, "VERSION")
            return codec.decode_version(r.data)
        return self._command(codec.encode_simple(Cmd.VERSION), "VERSION", decode)

    def cmd_fault(self) -> "Command[Fault]":
        def decode(r):
            self._check(r, "FAULT")
            return codec.decode_fault(r.data)
        return self._command(codec.encode_simple(Cmd.FAULT), "FAULT", decode)

    def cmd_read(self, ind: Ind) -> "Command[float]":
        label = f"INDICATOR {ind.name}"

        def decode(r):
            self._check(r, label)
            return codec.decode_indicator(r.data)
        return self._command(codec.encode_indicator(ind), label, decode)

    def cmd_read_conf(self, conf: Conf) -> "Command[int | float]":
        label = f"READ_CONF {conf.name}"

        def decode(r):
            self._check(r, label)
            return codec.decode_conf(r.data)[1]
        return self._command(codec.encode_read_conf(conf), label, decode)

    def _control(self, cmd: Cmd, value: float, duration_s: float | None, label: str) -> "Command[ControlState]":
        dur_ms = codec.seconds_to_duration_ms(self.default_duration_s if duration_s is None else duration_s)
        data = codec.encode_control(cmd, value, dur_ms)

        def decode(r):
            state = dataclasses.replace(codec.decode_control(r.data, self.kt, self.gear_ratio),
                                        timestamp=r.timestamp)
            self._check(r, label, state)
            return state
        return self._command(data, label, decode)

    def cmd_velocity(self, rad_s: float, duration_s: float | None = None) -> "Command[ControlState]":
        """速度指令（0x94）。rad_s は出力軸 [rad/s]。中で出力軸 RPM に換算する。"""
        if self.max_velocity_rad_s is not None and abs(rad_s) > self.max_velocity_rad_s:
            raise LimitExceeded(f"|{rad_s:.3f}| rad/s > max_velocity_rad_s {self.max_velocity_rad_s}")
        return self._control(Cmd.VELOCITY, codec.rad_s_to_rpm(rad_s), duration_s, "VELOCITY")

    def cmd_torque(self, nm: float, duration_s: float | None = None) -> "Command[ControlState]":
        """トルク指令（0x93）[N·m]。出力軸の値かどうかはプロトコル仕様に明記がない（未確認）。"""
        if self.max_torque_nm is not None and abs(nm) > self.max_torque_nm:
            raise LimitExceeded(f"|{nm:.3f}| N.m > max_torque_nm {self.max_torque_nm}")
        return self._control(Cmd.TORQUE, nm, duration_s, "TORQUE")

    def cmd_position(self, rad: float, duration_s: float | None = None) -> "Command[ControlState]":
        """位置指令（0x95）。rad は出力軸 [rad]。

        未確認の点（README の V4・V5）: Duration の時間をかけて補間するのか、多回転の角度なのか、
        電源を入れ直したあとの角度が正しいか。確かめるまでは、小さい角度・長い Duration で使うこと。
        """
        if not math.isfinite(rad):
            raise ValueError("position must be finite")
        return self._control(Cmd.POSITION, rad, duration_s, "POSITION")

    # --- 状態・設定の読み出し -------------------------------------------------------

    def read_version(self) -> int:
        return self.bus.transact(self.cmd_version())

    def read_fault(self) -> Fault:
        return self.bus.transact(self.cmd_fault())

    def ensure_no_fault(self) -> None:
        """故障があれば FaultActive を送出する。"""
        fault = self.read_fault()
        if fault != Fault.NONE:
            raise FaultActive(self.id, fault)

    def read(self, ind: Ind) -> float:
        """インジケータを 1 つ読む。"""
        if ind in STALE_INDICATORS:
            warnings.warn(f"{ind.name} is not updated by firmware 0x21", StaleIndicatorWarning, stacklevel=2)
        return self.bus.transact(self.cmd_read(ind))

    def read_many(self, inds: Iterable[Ind]) -> dict[Ind, float]:
        """インジケータを複数まとめて読む。1 つでも失敗すれば、その例外を送出する。"""
        inds = list(inds)
        values = self.bus.transact_many([self.cmd_read(i) for i in inds])
        for v in values:
            if isinstance(v, Gds34Error):
                raise v
        return dict(zip(inds, values))

    def read_conf(self, conf: Conf) -> int | float:
        return self.bus.transact(self.cmd_read_conf(conf))

    def read_all_conf(self) -> dict[Conf, int | float | Gds34Error]:
        """全設定項目を読む（実験条件の記録用）。読めなかった項目には例外オブジェクトが入る。"""
        values = self.bus.transact_many([self.cmd_read_conf(c) for c in Conf])
        return dict(zip(Conf, values))

    # --- 設定の書き込み -------------------------------------------------------------

    def write_conf(self, conf: Conf, value: float, *, allow_motor_model: bool = False) -> int | float:
        """設定項目を書き、読み戻して確かめる。書く前の値を返す（記録に残すため）。

        CAN ID・ホスト ID・ボーレート・プロトコルは書けない（Bus.change_id / change_host_id を使う）。
        モータの物理パラメータ（極対数・減速比・Rs・Ls・Kt など）は allow_motor_model=True が要る。
        電源断後も残るか、0x82 が要るかは未確認（README の V2）。
        """
        guard = conf.spec.guard
        if guard is Guard.READ_ONLY:
            raise GuardedSetting(f"{conf.name} is read-only")
        if guard is Guard.COMMUNICATION:
            raise GuardedSetting(f"{conf.name} cuts communication when written; use Bus.change_id / change_host_id")
        if guard is Guard.MOTOR_MODEL and not allow_motor_model:
            raise GuardedSetting(f"{conf.name} is a motor model parameter; pass allow_motor_model=True")
        before = self.read_conf(conf)
        label = f"WRITE_CONF {conf.name}"
        self.bus.transact(self._command(codec.encode_write_conf(conf, value), label,
                                        lambda r: self._check(r, label)))
        after = self.read_conf(conf)
        same = math.isclose(after, value, rel_tol=1e-6, abs_tol=1e-12) if conf.is_float else after == int(value)
        if not same:
            raise VerifyFailed(f"{conf.name}: wrote {value!r} but read back {after!r} (before: {before!r})")
        if conf in (Conf.GEAR_RATIO, Conf.TORQUE_CONSTANT):
            self.refresh_params()
        return before

    def apply_conf(self) -> None:
        """設定反映（0x82）。何が反映されるか（電源断後も残すために要るのか）は未確認（README の V2）。"""
        self.bus.transact(self._simple(Cmd.APPLY_CONF))

    # --- 起動・停止・制御 -------------------------------------------------------------

    def start(self) -> None:
        self.bus.transact(self.cmd_start())

    def stop(self) -> None:
        self.bus.transact(self.cmd_stop())

    def abort(self) -> None:
        """実行中の制御を中止する（0x97）。"""
        self.bus.transact(self.cmd_abort())

    def clear_fault(self) -> None:
        self.bus.transact(self.cmd_clear_fault())

    def safe_stop(self) -> list[Gds34Error]:
        """0x97 → 0x92 を送る。途中で失敗しても両方送り、起きた例外のリストを返す。"""
        errors: list[Gds34Error] = []
        for c in (self.cmd_abort(), self.cmd_stop()):
            try:
                self.bus.transact(c)
            except Gds34Error as e:
                errors.append(e)
        return errors

    @contextlib.contextmanager
    def enabled(self) -> Iterator["Motor"]:
        """起動（0x91）し、抜けるときに必ず 0x97 → 0x92 を送る（例外・Ctrl+C でも）。"""
        self.start()
        try:
            yield self
        finally:
            self.safe_stop()

    def set_velocity(self, rad_s: float, duration_s: float | None = None) -> ControlState:
        return self.bus.transact(self.cmd_velocity(rad_s, duration_s))

    def set_torque(self, nm: float, duration_s: float | None = None) -> ControlState:
        return self.bus.transact(self.cmd_torque(nm, duration_s))

    def set_position(self, rad: float, duration_s: float | None = None) -> ControlState:
        return self.bus.transact(self.cmd_position(rad, duration_s))

    # --- まとめ ---------------------------------------------------------------

    def info(self) -> dict:
        """版・故障・全設定を 1 つの辞書にまとめる（表示・記録用）。"""
        return {"id": self.id, "host_id": self.host_id, "version": self.read_version(),
                "fault": self.read_fault(), "conf": self.read_all_conf()}
