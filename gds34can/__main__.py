"""ベンチ用の CLI（読み出しと停止だけ。動かす操作は examples/ にある）。

    python -m gds34can scan
    python -m gds34can info --id 1
    python -m gds34can fault --id 1 [--clear]
    python -m gds34can stop --id 1
    python -m gds34can --fake scan        # 実機なし（擬似ドライバ）
"""
from __future__ import annotations

import argparse
import contextlib
import sys

from . import Bus, Conf, Fault, Gds34Error, Ind, TESTED_FIRMWARE
from .fake import FakeNetwork


def _int(x: str) -> int:
    return int(x, 0)


def fault_text(f: Fault) -> str:
    return "NONE" if f == Fault.NONE else " | ".join(m.name for m in Fault if m and m in f)


def cmd_scan(bus: Bus, args) -> int:
    found = bus.scan(range(args.first, args.last + 1))
    for f in found:
        print(f"id=0x{f.motor_id:03X}  reply_from=0x{f.host_id:03X}  version=0x{f.version:08X}")
    if not found:
        print("応答なし。ビットレート（出荷時 500kbps）・配線・終端・ドライバの電源を確認する")
    return 0 if found else 1


def cmd_info(bus: Bus, args) -> int:
    m = bus.motor(args.id, host_id=args.host_id)
    info = m.info()
    ver = info["version"]
    print(f"id           = 0x{m.id:03X}")
    print(f"version      = 0x{ver:08X}" + ("" if ver in TESTED_FIRMWARE else "  (未確認の版)"))
    print(f"fault        = {fault_text(info['fault'])}")
    for ind in (Ind.DRIVER_TEMP, Ind.MOTOR_TEMP, Ind.OUTPUT_ANGLE):
        print(f"{ind.name:<28} = {m.read(ind):.4f}")
    for conf, value in info["conf"].items():
        unit = f" [{conf.spec.unit}]" if conf.spec.unit else ""
        text = f"<{type(value).__name__}>" if isinstance(value, Gds34Error) else f"{value}"
        print(f"{conf.name + unit:<28} = {text}")
    return 0


def cmd_fault(bus: Bus, args) -> int:
    m = bus.motor(args.id, host_id=args.host_id, read_params=False)
    f = m.read_fault()
    print(f"fault = {fault_text(f)}")
    if args.clear and f != Fault.NONE:
        m.clear_fault()
        print(f"cleared -> {fault_text(m.read_fault())}")
    return 0


def cmd_stop(bus: Bus, args) -> int:
    m = bus.motor(args.id, host_id=args.host_id, read_params=False)
    errors = m.safe_stop()
    print("stopped" if not errors else f"stop sent with errors: {errors}")
    return 0 if not errors else 1


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="python -m gds34can", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iface", default="hatcan0", help="CAN インタフェース名（既定 hatcan0）")
    p.add_argument("--fake", action="store_true", help="擬似ドライバ（ID 1）を相手にする")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan", help="応答する ID を探す")
    s.add_argument("--first", type=_int, default=0x01)
    s.add_argument("--last", type=_int, default=0x7F)
    for name, helptext in (("info", "版・故障・全設定を表示する"), ("fault", "故障を読む（--clear で解除）"),
                           ("stop", "0x97 → 0x92 を送る")):
        sp = sub.add_parser(name, help=helptext)
        sp.add_argument("--id", type=_int, required=True)
        sp.add_argument("--host-id", type=_int, default=0)
        if name == "fault":
            sp.add_argument("--clear", action="store_true")
    args = p.parse_args(argv)

    with contextlib.ExitStack() as stack:
        if args.fake:
            net = stack.enter_context(FakeNetwork())
            bus = stack.enter_context(net.open_bus())
        else:
            bus = stack.enter_context(Bus(args.iface))
        handler = {"scan": cmd_scan, "info": cmd_info, "fault": cmd_fault, "stop": cmd_stop}[args.cmd]
        try:
            return handler(bus, args)
        except Gds34Error as e:
            print(f"error: {e}", file=sys.stderr)
            return 1


if __name__ == "__main__":
    sys.exit(main())
