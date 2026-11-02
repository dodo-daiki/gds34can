"""擬似 GDS34（テスト・example 用）。

python-can の virtual インタフェースの上で GDS34 のふりをして応答する。実機なしで、Windows でも動く。

実機で確かめていない挙動は、次の**仮定**で動かしている。実機で確かめたら合わせること（README「未確認の挙動」）:
- 速度・トルク指令は Duration が切れると止まる（V3）
- 位置指令は Duration の時間をかけて、目標（出力軸の多回転の角度）へ直線補間する（V4）
- 起動前・故障中の制御指令には RES = FAILURE を返し、動かない（V10）
- ホスト ID を書き換えると、応答の送信元もその ID に変わる（V1）。host_id_follows_conf=False で変えられる

設定の初期値は実機の読み出し（GIM4310-10・ファーム 0x21、2026-10-24）の値。実機で読んでいない項目
（弱め界磁ゲイン・電流検出抵抗・増幅ゲイン）は UNKNOWN_REGISTER を返す。
電流・電圧のインジケータは簡単なモデルの値で、実機の値ではない（Iq の 0.14 A は 2026-10-24 の無負荷の実測の目安）。
"""
from __future__ import annotations

import math
import struct
import threading
import time

import can

from . import codec
from .bus import Bus
from .constants import Cmd, Conf, Fault, Ind, Result

DEFAULT_CONF: dict[Conf, int | float] = {
    Conf.CAN_ID: 1, Conf.HOST_CAN_ID: 0, Conf.CAN_BITRATE: 5, Conf.PROTOCOL: 0,
    Conf.GEAR_RATIO: 10, Conf.POLE_PAIRS: 14, Conf.MAX_CURRENT: 8, Conf.MAX_SPEED: 3300,
    Conf.RATED_VOLTAGE: 24, Conf.OVER_VOLTAGE: 40, Conf.UNDER_VOLTAGE: 12,
    Conf.ZERO_POSITION: 193, Conf.POWER_OFF_POSITION: 0, Conf.OVER_TEMPERATURE: 80,
    Conf.PWM_FREQUENCY: 30000, Conf.CURRENT_KP: 1000, Conf.CURRENT_KI: 200,
    Conf.SPEED_KP: 2000, Conf.SPEED_KI: 200, Conf.POSITION_KP: 1, Conf.POSITION_KI: 0, Conf.POSITION_KD: 1000,
    Conf.PHASE_RESISTANCE: 0.324, Conf.PHASE_INDUCTANCE: 0.000231, Conf.BACK_EMF: 4.0,
    Conf.TORQUE_CONSTANT: 0.08472,
}

STALE_BUS_VOLTAGE = 2.552772045135498   # ファーム 0x21 の実機が返し続けた値
FRICTION_IQ_A = 0.14                    # 無負荷で回すときの Iq の目安（2026-10-24 の実測。README「実機で確かめたこと」）


class FakeMotor:
    """擬似ドライバ 1 台の状態と応答。"""

    def __init__(self, motor_id: int = 1, *, host_id: int = 0, gear_ratio: int = 10,
                 version: int = 0x21, host_id_follows_conf: bool = True):
        self.conf = dict(DEFAULT_CONF)
        self.conf[Conf.CAN_ID] = motor_id
        self.conf[Conf.HOST_CAN_ID] = host_id
        self.conf[Conf.GEAR_RATIO] = gear_ratio
        self.version = version
        self.host_id_follows_conf = host_id_follows_conf
        self.reply_from = host_id
        self.enabled = False
        self.fault = Fault.NONE
        self.temperature_c = 30
        self.position = 0.0       # 出力軸 [rad]（多回転）
        self.velocity = 0.0       # 出力軸 [rad/s]
        self.torque_cmd = 0.0
        self._mode: Cmd | None = None
        self._deadline = 0.0
        self._pos_from = self._pos_to = 0.0
        self._t0 = self._t1 = 0.0
        self._last = time.perf_counter()
        self.received: list[bytes] = []   # 受け取った要求（テストでの確認用）

    @property
    def id(self) -> int:
        return int(self.conf[Conf.CAN_ID])

    def inject_fault(self, fault: Fault) -> None:
        """故障を起こす。モータは止まり、0xB3 まで制御指令を受け付けない。"""
        self.fault |= fault
        self.enabled = False
        self._halt()

    def _halt(self) -> None:
        self._mode, self.velocity, self.torque_cmd = None, 0.0, 0.0

    # --- 動き -----------------------------------------------------------------

    def _update(self, now: float) -> None:
        prev, self._last = self._last, now
        if self._mode is Cmd.VELOCITY:
            active_until = min(now, self._deadline)
            if active_until > prev:
                self.position += self.velocity * (active_until - prev)
            if now >= self._deadline:
                self._halt()
        elif self._mode is Cmd.POSITION:
            span = max(self._t1 - self._t0, 1e-9)
            frac = min(1.0, max(0.0, (now - self._t0) / span))
            self.position = self._pos_from + (self._pos_to - self._pos_from) * frac
            self.velocity = (self._pos_to - self._pos_from) / span if frac < 1.0 else 0.0
            if frac >= 1.0:
                self._mode = None
        elif self._mode is Cmd.TORQUE and now >= self._deadline:
            self._halt()

    def _iq(self) -> float:
        kt, gr = float(self.conf[Conf.TORQUE_CONSTANT]), float(self.conf[Conf.GEAR_RATIO])
        if self._mode is Cmd.TORQUE:
            return self.torque_cmd / (kt * gr)
        if abs(self.velocity) > 1e-6:
            return math.copysign(FRICTION_IQ_A, self.velocity)
        return 0.0

    def _indicator(self, ind: int) -> float | None:
        gr, pp = float(self.conf[Conf.GEAR_RATIO]), float(self.conf[Conf.POLE_PAIRS])
        iq = self._iq()
        mech = (self.position * gr) % (2 * math.pi)
        elec = (mech * pp) % (2 * math.pi)
        rpm = codec.rad_s_to_rpm(self.velocity)
        values = {
            Ind.BUS_VOLTAGE: STALE_BUS_VOLTAGE, Ind.POWER: 0.0,
            Ind.DRIVER_TEMP: float(self.temperature_c), Ind.MOTOR_TEMP: float(self.temperature_c),
            Ind.IQ: iq, Ind.ID: 0.0, Ind.IQ_TARGET: iq, Ind.ID_TARGET: 0.0,
            Ind.IA: -iq * math.sin(elec), Ind.IB: -iq * math.sin(elec - 2 * math.pi / 3),
            Ind.IC: -iq * math.sin(elec + 2 * math.pi / 3),
            Ind.VQ: 0.011 * rpm + float(self.conf[Conf.PHASE_RESISTANCE]) * iq, Ind.VD: 0.0,
            Ind.ROTOR_ELEC_ANGLE: elec, Ind.ROTOR_MECH_ANGLE: mech,
            Ind.OUTPUT_ANGLE: self.position, Ind.OUTPUT_SPEED_RPM: rpm,
        }
        try:
            return values.get(Ind(ind), 0.0)
        except ValueError:
            return None

    # --- 応答 -----------------------------------------------------------------

    def handle(self, data: bytes, now: float | None = None) -> tuple[int, bytes] | None:
        """要求 1 つを処理し、(応答の送信元 ID, 応答データ) を返す。応答しない場合は None。"""
        now = time.perf_counter() if now is None else now
        self._update(now)
        self.received.append(bytes(data))
        cmd, reply_from = data[0], self.reply_from

        def frame(*b: int) -> bytes:
            return bytes(b).ljust(8, b"\x00")

        if cmd == Cmd.FIRMWARE_UPDATE:
            return None
        if cmd == Cmd.VERSION:
            out = frame(cmd, Result.SUCCESS)[:4] + self.version.to_bytes(4, "little")
        elif cmd == Cmd.FAULT:
            out = frame(cmd, Result.SUCCESS, int(self.fault))
        elif cmd == Cmd.CLEAR_FAULT:
            self.fault = Fault.NONE
            out = frame(cmd, Result.SUCCESS)
        elif cmd == Cmd.START:
            ok = self.fault == Fault.NONE
            self.enabled = self.enabled or ok
            out = frame(cmd, Result.SUCCESS if ok else Result.FAILURE)
        elif cmd == Cmd.STOP:
            self.enabled = False
            self._halt()
            out = frame(cmd, Result.SUCCESS)
        elif cmd == Cmd.ABORT:
            self._halt()
            out = frame(cmd, Result.SUCCESS)
        elif cmd == Cmd.APPLY_CONF:
            out = frame(cmd, Result.SUCCESS)
        elif cmd == Cmd.INDICATOR:
            value = self._indicator(data[1])
            out = (frame(cmd, data[1], Result.UNKNOWN_REGISTER) if value is None
                   else frame(cmd, data[1], Result.SUCCESS, 0)[:4] + struct.pack("<f", value))
        elif cmd in (Cmd.READ_CONF, Cmd.WRITE_CONF):
            out, reply_from = self._conf(data, reply_from)
        elif cmd in (Cmd.TORQUE, Cmd.VELOCITY, Cmd.POSITION):
            out = self._control(Cmd(cmd), data, now)
        else:
            out = frame(cmd, Result.UNKNOWN_COMMAND)
        return reply_from, out

    def _conf(self, data: bytes, reply_from: int) -> tuple[bytes, int]:
        cmd = data[0]
        head = bytes(data[:3])
        try:
            conf = Conf.lookup(data[1], data[2])
        except KeyError:
            conf = None
        if conf is None or conf not in self.conf:
            return head + bytes([Result.UNKNOWN_REGISTER]) + bytes(4), reply_from
        if cmd == Cmd.READ_CONF:
            return head + bytes([Result.SUCCESS]) + codec.pack_conf_value(conf, self.conf[conf]), reply_from
        if conf is Conf.POWER_OFF_POSITION:
            return head + bytes([Result.READ_ONLY]) + bytes(4), reply_from
        self.conf[conf] = codec.unpack_conf_value(conf, data[4:8])
        if conf is Conf.HOST_CAN_ID and self.host_id_follows_conf:
            self.reply_from = int(self.conf[conf])   # この応答は古い ID から、次から新しい ID から
        return head + bytes([Result.SUCCESS]) + bytes(4), reply_from

    def _control(self, cmd: Cmd, data: bytes, now: float) -> bytes:
        value = struct.unpack("<f", bytes(data[1:5]))[0]
        duration_s = int.from_bytes(bytes(data[5:8]), "little") / 1000.0
        ok = self.enabled and self.fault == Fault.NONE
        if ok:
            if cmd is Cmd.VELOCITY:
                self._mode, self.velocity, self._deadline = cmd, codec.rpm_to_rad_s(value), now + duration_s
            elif cmd is Cmd.TORQUE:
                self._mode, self.torque_cmd, self._deadline = cmd, value, now + duration_s
            else:
                self._mode, self._pos_from, self._pos_to = cmd, self.position, value
                self._t0, self._t1 = now, now + duration_s
        kt, gr = float(self.conf[Conf.TORQUE_CONSTANT]), float(self.conf[Conf.GEAR_RATIO])
        return codec.encode_control_reply(cmd, Result.SUCCESS if ok else Result.FAILURE, self.temperature_c,
                                          self.position, self.velocity, self._iq() * kt * gr, kt, gr)


class FakeNetwork:
    """擬似ドライバをつないだ仮想の CAN バス。

    >>> with FakeNetwork(motors=[FakeMotor(1)]) as net, net.open_bus() as bus:
    ...     print(bus.scan())
    """

    def __init__(self, motors: list[FakeMotor] | None = None, *, channel: str = "gds34can-fake",
                 latency_s: float = 0.0003):
        self.motors = list(motors) if motors else [FakeMotor(1)]
        self.channel = channel
        self.latency_s = latency_s
        self._bus = can.Bus(interface="virtual", channel=channel)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gds34can-fake", daemon=True)

    def start(self) -> "FakeNetwork":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._bus.shutdown()

    def __enter__(self) -> "FakeNetwork":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def open_bus(self, **kwargs) -> Bus:
        """この仮想バスにつながる Bus を作る。"""
        return Bus(self.channel, interface="virtual", **kwargs)

    def _run(self) -> None:
        while not self._stop.is_set():
            msg = self._bus.recv(0.01)
            if msg is None or msg.is_extended_id or len(msg.data) != 8:
                continue
            for m in self.motors:
                if m.id != msg.arbitration_id:
                    continue
                out = m.handle(bytes(msg.data))
                if out is None:
                    continue
                if self.latency_s:
                    time.sleep(self.latency_s)
                self._bus.send(can.Message(arbitration_id=out[0], data=out[1], is_extended_id=False))
