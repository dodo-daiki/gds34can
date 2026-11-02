"""06: 送受信の生フレームとイベントを記録する（フックの使い方。読み出しだけ）。

    python examples/06_frame_log.py --log frames.log
    python examples/06_frame_log.py --fake

on_frame には送受信した全フレームが、on_event にはタイムアウトなどのイベントが渡される。
記録の形式は candump -td に近い。存在しない ID（0x7E）にも送り、タイムアウトのイベントが出ることを見せる。
"""
import time

from _common import open_bus, parser

from gds34can import Ind, NoReply


def main():
    p = parser(__doc__)
    p.add_argument("--id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--log", default=None, help="生フレームの記録先（省略時は画面）")
    args = p.parse_args()
    out = open(args.log, "w", encoding="utf-8") if args.log else None
    t0 = time.time()

    def on_frame(direction, msg):
        line = (f"({time.time() - t0:010.6f}) {direction} {msg.arbitration_id:03X} "
                f"[{msg.dlc}] {bytes(msg.data).hex(' ').upper()}")
        print(line, file=out) if out else print(line)

    def on_event(ev):
        print(f"EVENT {ev.kind:<12} id={ev.motor_id} {ev.detail}")

    with open_bus(args, on_frame=on_frame, on_event=on_event) as bus:
        m = bus.motor(args.id)
        print("version", hex(m.read_version()), "fault", repr(m.read_fault()))
        print("iq", m.read(Ind.IQ))
        try:
            bus.motor(0x7E, read_params=False).read_version()
        except NoReply as e:
            print("expected:", e)
    if out:
        out.close()
        print(f"frames: {args.log}")


if __name__ == "__main__":
    main()
