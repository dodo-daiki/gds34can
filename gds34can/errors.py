"""gds34can の例外。すべて Gds34Error を継承する。"""
from __future__ import annotations

from typing import Any


class Gds34Error(Exception):
    """gds34can の全例外の基底。"""


class NoReply(Gds34Error):
    """タイムアウトまでに応答がなかった。"""

    def __init__(self, motor_id: int, label: str, timeout_s: float):
        super().__init__(f"no reply from id=0x{motor_id:03X} to {label} within {timeout_s * 1000:.0f} ms")
        self.motor_id, self.label, self.timeout_s = motor_id, label, timeout_s


class CommandRejected(Gds34Error):
    """ドライバが結果コード SUCCESS 以外を返した。制御指令の場合は state に応答の状態が入る。"""

    def __init__(self, motor_id: int, label: str, result: Any, state: Any = None):
        name = getattr(result, "name", f"0x{int(result):02X}")
        super().__init__(f"id=0x{motor_id:03X} rejected {label}: {name}")
        self.motor_id, self.label, self.result, self.state = motor_id, label, result, state


class FaultActive(Gds34Error):
    """ドライバが故障状態にある（0xB3 で解除するまで全コマンドが拒否される）。"""

    def __init__(self, motor_id: int, fault: Any):
        super().__init__(f"id=0x{motor_id:03X} is in fault: {fault!r}")
        self.motor_id, self.fault = motor_id, fault


class WrongProtocol(Gds34Error):
    """MIT プロトコルの応答（DLC = 6）を受け取った。ドライバが ConfID 0x1C = 1 になっている。"""


class BusError(Gds34Error):
    """CAN バスの異常（送信できない、ERROR-PASSIVE、BUS-OFF など）。state は SocketCAN の状態。"""

    def __init__(self, message: str, state: str | None = None):
        super().__init__(message if state is None else f"{message} (can state: {state})")
        self.state = state


class BlockedCommand(Gds34Error):
    """送信禁止のコマンド（0x81 / 0x96 / 0xB5 / 0xC1）を送ろうとした。"""


class GuardedSetting(Gds34Error):
    """保護された設定項目を、必要な手順・許可なしに書こうとした。"""


class LimitExceeded(Gds34Error):
    """Motor に設定した上限を超える指令を送ろうとした。"""


class VerifyFailed(Gds34Error):
    """書き込み後の読み戻しや、ID 変更後の確認が一致しなかった。"""


class ProtocolError(Gds34Error):
    """応答の形がプロトコル仕様と合わない。"""
