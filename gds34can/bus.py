"""1 本の CAN バスの送受信。

GDS34 は出荷時、全軸がホスト ID（0x000）から応答し、応答のデータにも軸の情報がない。
そのため、送受信の排他はバス単位で行う: 1 本のバスにつき、未応答の要求は常に 1 つまで。
"""
from __future__ import annotations

import json
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Generic, Iterable, Sequence, TypeVar

import can

from . import codec
from .constants import BLOCKED_COMMANDS, Cmd, Conf
from .errors import BlockedCommand, BusError, GuardedSetting, Gds34Error, NoReply, VerifyFailed, WrongProtocol

if TYPE_CHECKING:
    from .motor import Motor

T = TypeVar("T")


@dataclass(frozen=True)
class Request:
    motor_id: int
    data: bytes
    #: 応答を受け付ける送信元 ID（＝ドライバに設定されたホスト ID）。None なら問わない
    reply_from: int | None
    label: str


@dataclass(frozen=True)
class Reply:
    motor_id: int
    arbitration_id: int
    data: bytes
    #: 受信時刻 [s]（python-can の Message.timestamp。SocketCAN ではエポック秒）
    timestamp: float


@dataclass(frozen=True)
class Command(Generic[T]):
    """送る要求と、応答を結果に変える関数の組。Motor.cmd_* が作る（作るだけでは通信しない）。"""

    request: Request
    decode: Callable[[Reply], T]


@dataclass(frozen=True)
class Found:
    motor_id: int
    host_id: int   # 応答の送信元 ID
    version: int


@dataclass(frozen=True)
class Event:
    """記録用のイベント。kind: timeout / late_reply / error_frame / bus_error"""

    time: float
    kind: str
    motor_id: int | None
    detail: str


FrameHook = Callable[[str, can.Message], None]   # direction ("tx" / "rx"), message
EventHook = Callable[[Event], None]


class Bus:
    """1 本の CAN バス。複数のスレッドから呼んでよい（中で排他する）。

    >>> with Bus("hatcan0") as bus:
    ...     m = bus.motor(1)
    """

    def __init__(self, channel: str = "hatcan0", *, interface: str = "socketcan",
                 timeout_s: float = 0.05, late_reply_guard_s: float | None = None,
                 on_frame: FrameHook | None = None, on_event: EventHook | None = None,
                 can_bus: can.BusABC | None = None):
        """
        channel / interface: python-can に渡す。ビットレートは OS 側で設定する（README「導入」）
        timeout_s: 応答の待ち時間の既定値
        late_reply_guard_s: タイムアウトのあと、次の送信までに待つ時間（遅れて届いた応答を捨てるため）。
            None なら timeout_s と同じ
        on_frame / on_event: 生フレームとイベントを受け取るコールバック（記録用）
        can_bus: 作成済みの python-can のバスを使う場合（テスト用）。close() では閉じない
        """
        self.channel = channel
        self.interface = interface
        self.timeout_s = timeout_s
        self.late_reply_guard_s = timeout_s if late_reply_guard_s is None else late_reply_guard_s
        self.on_frame = on_frame
        self.on_event = on_event
        self._can = can_bus if can_bus is not None else can.Bus(interface=interface, channel=channel)
        self._owns_can = can_bus is None
        self._lock = threading.RLock()
        self._quiet_until = 0.0

    # --- 開閉 ---------------------------------------------------------------

    def close(self) -> None:
        if self._owns_can:
            self._can.shutdown()

    def __enter__(self) -> "Bus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- モータ -------------------------------------------------------------

    def motor(self, motor_id: int, **kwargs) -> "Motor":
        """このバス上のドライバ 1 台。キーワード引数は Motor にそのまま渡す。"""
        from .motor import Motor
        return Motor(self, motor_id, **kwargs)

    # --- 送受信 -------------------------------------------------------------

    def transact(self, command: Command[T], timeout_s: float | None = None) -> T:
        """1 つの要求を送り、応答を待って、結果を返す。"""
        return command.decode(self._exchange(command.request, timeout_s))

    def transact_many(self, commands: Sequence[Command], timeout_s: float | None = None) -> list:
        """複数の要求を順に送受信し、結果のリストを返す。

        失敗した要素は、例外を送出せずに、その位置に例外オブジェクトを入れて返す
        （1 軸の失敗で残りの軸の指令を止めないため）。
        今は 1 つずつ順に送る。応答の送信元をホスト ID で区別できると確認できたら（README の V1）、
        まとめて送る実装に変えられる。呼び出し側は変えなくてよい。
        """
        results: list = []
        with self._lock:
            for c in commands:
                try:
                    results.append(self.transact(c, timeout_s))
                except Gds34Error as e:
                    results.append(e)
        return results

    def _exchange(self, req: Request, timeout_s: float | None) -> Reply:
        if req.data[0] in BLOCKED_COMMANDS:
            raise BlockedCommand(f"command 0x{req.data[0]:02X} ({Cmd(req.data[0]).name}) is blocked")
        timeout = self.timeout_s if timeout_s is None else timeout_s
        key = codec.match_key(req.data)
        with self._lock:
            wait = self._quiet_until - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            self._drain()
            msg = can.Message(arbitration_id=req.motor_id, data=req.data, is_extended_id=False)
            try:
                self._can.send(msg)
            except can.CanError as e:
                state = self.can_state()
                self._emit("bus_error", req.motor_id, f"send failed: {e}")
                raise BusError(f"failed to send {req.label} to id=0x{req.motor_id:03X}: {e}", state) from e
            self._hook_frame("tx", msg)
            deadline = time.perf_counter() + timeout
            while (left := deadline - time.perf_counter()) > 0:
                rx = self._can.recv(left)
                if rx is None:
                    break
                self._hook_frame("rx", rx)
                if rx.is_error_frame:
                    self._emit("error_frame", None, f"arb_id=0x{rx.arbitration_id:08X} data={bytes(rx.data).hex(' ')}")
                    continue
                if rx.is_extended_id or rx.is_remote_frame or rx.arbitration_id == req.motor_id:
                    continue   # 同じドライバ宛ての、ほかのホストからの要求は応答ではない
                if req.reply_from is not None and rx.arbitration_id != req.reply_from:
                    continue
                if len(rx.data) == codec.MIT_REPLY_LEN:
                    raise WrongProtocol(
                        f"got a 6-byte reply from 0x{rx.arbitration_id:03X} ({bytes(rx.data).hex(' ')}). "
                        "The driver seems to be in MIT protocol mode (ConfID 0x1C = 1).")
                if len(rx.data) != codec.FRAME_LEN or codec.match_key(rx.data) != key:
                    continue
                return Reply(req.motor_id, rx.arbitration_id, bytes(rx.data), rx.timestamp)
            self._quiet_until = time.perf_counter() + self.late_reply_guard_s
            self._emit("timeout", req.motor_id, req.label)
            raise NoReply(req.motor_id, req.label, timeout)

    def _drain(self) -> None:
        """送信前に、受信済みの古いフレームを捨てる（遅れて届いた応答を、次の要求の応答にしない）。"""
        while (rx := self._can.recv(0)) is not None:
            self._hook_frame("rx", rx)
            if not rx.is_error_frame and len(rx.data) == codec.FRAME_LEN:
                self._emit("late_reply", None,
                           f"discarded arb_id=0x{rx.arbitration_id:03X} data={bytes(rx.data).hex(' ')}")

    # --- 探索 ---------------------------------------------------------------

    def ping(self, motor_id: int, timeout_s: float | None = None) -> Found | None:
        """バージョン取得（0xB1）を送り、応答があれば Found を返す。送信元の ID は問わない。"""
        req = Request(motor_id, codec.encode_simple(Cmd.VERSION), None, "VERSION")
        try:
            r = self._exchange(req, timeout_s)
        except NoReply:
            return None
        return Found(motor_id, r.arbitration_id, codec.decode_version(r.data))

    def scan(self, ids: Iterable[int] = range(0x01, 0x80), timeout_s: float = 0.02) -> list[Found]:
        """ID の範囲に 0xB1 を送り、応答した ID を返す。

        全軸の応答が同じ送信元 ID から来るため、遅れて届いた応答を別の ID のものと取り違える余地がある。
        見つかった ID には、もう一度通常のタイムアウトで送り直して確かめる。
        """
        with self._lock:
            guard, self.late_reply_guard_s = self.late_reply_guard_s, 0.0
            try:
                candidates = [i for i in ids if self.ping(i, timeout_s) is not None]
            finally:
                self.late_reply_guard_s = guard
            return [f for i in candidates if (f := self.ping(i)) is not None]

    # --- 通信設定の書き換え ----------------------------------------------

    def _require_single(self, motor_id: int, scan_ids: Iterable[int]) -> Found:
        found = self.scan(scan_ids)
        if len(found) != 1 or found[0].motor_id != motor_id:
            raise GuardedSetting(
                f"bus must have exactly one responding driver (id=0x{motor_id:03X}) before changing "
                f"communication settings; found {[hex(f.motor_id) for f in found]}")
        return found[0]

    def _write_comm(self, motor_id: int, conf: Conf, value: int) -> None:
        req = Request(motor_id, codec.encode_write_conf(conf, value), None, f"WRITE_CONF {conf.name}")
        try:
            self._exchange(req, None)   # 書いた瞬間に通信が切れるので、応答がなくても続ける
        except NoReply:
            pass

    def change_id(self, motor_id: int, new_id: int, *, scan_ids: Iterable[int] = range(0x01, 0x80)) -> Found:
        """ドライバの CAN ID（ConfID 0x12）を書き換える。

        バス上で応答するドライバが motor_id の 1 台だけのときに限り行う。書いたあと、新しい ID が応答し、
        古い ID が応答しないことを確かめる。電源断後も残るかは未確認（README の V2）。
        """
        scan_ids = list(scan_ids)
        with self._lock:
            found = self._require_single(motor_id, scan_ids)
            if not 0x001 <= new_id <= 0x7FF or new_id in (motor_id, found.host_id):
                raise ValueError(f"invalid new id 0x{new_id:03X} (host id is 0x{found.host_id:03X})")
            self._write_comm(motor_id, Conf.CAN_ID, new_id)
            after = self.ping(new_id)
            if after is None or self.ping(motor_id) is not None:
                raise VerifyFailed(f"after writing CAN ID 0x{new_id:03X}: new id "
                                   f"{'responds' if after else 'does not respond'}; rescan the bus")
            return after

    def change_host_id(self, motor_id: int, new_host_id: int, *,
                       scan_ids: Iterable[int] = range(0x01, 0x80)) -> Found:
        """ドライバのホスト CAN ID（ConfID 0x13）を書き換え、そのあとの応答の送信元 ID を返す。

        応答の送信元がホスト ID に従うかは未確認（README の V1）。返り値の host_id が new_host_id と
        同じなら従う。以後この Motor は bus.motor(motor_id, host_id=返り値の host_id) で作り直すこと。
        """
        with self._lock:
            self._require_single(motor_id, scan_ids)
            if not 0x000 <= new_host_id <= 0x7FF or new_host_id == motor_id:
                raise ValueError(f"invalid host id 0x{new_host_id:03X}")
            self._write_comm(motor_id, Conf.HOST_CAN_ID, new_host_id)
            after = self.ping(motor_id)
            if after is None:
                raise VerifyFailed(f"id=0x{motor_id:03X} stopped responding after writing host id")
            return after

    # --- 状態 ---------------------------------------------------------------

    def can_state(self) -> str | None:
        """SocketCAN の状態（ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF など）。Linux の socketcan 以外では None。"""
        if self.interface != "socketcan":
            return None
        try:
            out = subprocess.run(["ip", "-details", "-json", "link", "show", self.channel],
                                 capture_output=True, text=True, timeout=1.0, check=True).stdout
            return json.loads(out)[0]["linkinfo"]["info_data"]["state"]
        except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError):
            return None

    # --- フック -------------------------------------------------------------

    def _hook_frame(self, direction: str, msg: can.Message) -> None:
        if self.on_frame is not None:
            self.on_frame(direction, msg)

    def _emit(self, kind: str, motor_id: int | None, detail: str) -> None:
        if self.on_event is not None:
            self.on_event(Event(time.time(), kind, motor_id, detail))
