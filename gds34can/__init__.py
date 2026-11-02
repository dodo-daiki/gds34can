"""gds34can — SteadyWin GDS34 ドライバを CAN（SocketCAN）で扱うライブラリ。

プロトコル: SteadyWin GIM プロトコル rev2.2（CAN 2.0 標準フレーム）。

公開 API はこのモジュールが export するものだけ。

    from gds34can import Bus, Ind, Conf

    with Bus("hatcan0") as bus:
        m = bus.motor(1)
        with m.enabled():
            state = m.set_velocity(rad_s=1.0, duration_s=0.3)
"""
from .bus import Bus, Command, Event, Found, Reply, Request
from .codec import ControlState, rad_s_to_rpm, rpm_to_rad_s
from .constants import BLOCKED_COMMANDS, STALE_INDICATORS, Cmd, Conf, ConfSpec, Fault, Guard, Ind, Result
from .errors import (BlockedCommand, BusError, CommandRejected, FaultActive, Gds34Error, GuardedSetting,
                     LimitExceeded, NoReply, ProtocolError, VerifyFailed, WrongProtocol)
from .motor import TESTED_FIRMWARE, Motor, StaleIndicatorWarning

__version__ = "0.1.0"

__all__ = [
    "Bus", "Command", "Event", "Found", "Reply", "Request",
    "Motor", "ControlState", "TESTED_FIRMWARE", "StaleIndicatorWarning",
    "Cmd", "Conf", "ConfSpec", "Fault", "Guard", "Ind", "Result", "BLOCKED_COMMANDS", "STALE_INDICATORS",
    "rad_s_to_rpm", "rpm_to_rad_s",
    "Gds34Error", "NoReply", "CommandRejected", "FaultActive", "WrongProtocol", "BusError",
    "BlockedCommand", "GuardedSetting", "LimitExceeded", "VerifyFailed", "ProtocolError",
    "__version__",
]
