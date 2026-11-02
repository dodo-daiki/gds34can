"""examples 共通: 引数と、実機 / 擬似ドライバの切り替え。"""
from __future__ import annotations

import argparse
import contextlib
import pathlib
import sys

try:
    import gds34can  # noqa: F401  pip install -e 済みならそれを使う
except ImportError:   # リポジトリから直接実行したとき
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gds34can import Bus  # noqa: E402
from gds34can.fake import FakeMotor, FakeNetwork  # noqa: E402


def parser(doc: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iface", default="hatcan0", help="CAN インタフェース名（既定 hatcan0）")
    p.add_argument("--fake", action="store_true", help="実機の代わりに擬似ドライバを使う（Windows でも動く）")
    return p


@contextlib.contextmanager
def open_bus(args, fake_ids=(1,), **bus_kwargs):
    """--fake なら擬似ドライバ（fake_ids の台数）を立ててつなぐ。そうでなければ実機のバスを開く。"""
    if args.fake:
        with FakeNetwork(motors=[FakeMotor(i) for i in fake_ids]) as net, net.open_bus(**bus_kwargs) as bus:
            yield bus
    else:
        with Bus(args.iface, **bus_kwargs) as bus:
            yield bus
