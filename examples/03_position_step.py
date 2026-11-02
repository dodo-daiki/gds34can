"""03: 位置指令（0x95）を 1 回だけ送り、出力軸の角度の変化を記録する（README の V3・V4 の確認用）。

    python examples/03_position_step.py --delta 0.2 --duration 1.0 --confirm
    python examples/03_position_step.py --fake --confirm

位置指令には未確認の点が多い（Duration の時間をかけて補間するのか、多回転の角度なのか、Duration が切れたあと
保持するのか）。この example はそれを確かめるためのもので、次の安全策を入れている:
- --confirm がなければ何もしない
- 目標 = 今の多回転の角度（Ind.OUTPUT_ANGLE）+ delta。|delta| は 0.5 rad まで
- 今の多回転の角度が ±2π を超えていたら中止する（位置指令の基準が分からないまま大きく動くのを防ぐ）
"""
import math
import time

from _common import open_bus, parser

from gds34can import Ind


def main():
    p = parser(__doc__)
    p.add_argument("--id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--delta", type=float, default=0.2, help="今の角度からの移動量 [rad]（出力軸）")
    p.add_argument("--duration", type=float, default=1.0, help="指令の Duration [s]")
    p.add_argument("--observe", type=float, default=2.0, help="指令のあと記録する時間 [s]")
    p.add_argument("--confirm", action="store_true", help="モータを動かしてよい")
    args = p.parse_args()
    if abs(args.delta) > 0.5:
        p.error("|--delta| は 0.5 rad まで")
    if not args.confirm:
        print("--confirm を付けると実行する（出力軸が動く）")
        return

    with open_bus(args) as bus:
        m = bus.motor(args.id)
        m.ensure_no_fault()
        start = m.read(Ind.OUTPUT_ANGLE)
        if abs(start) > 2 * math.pi:
            print(f"多回転の角度が {start:.3f} rad。位置指令の基準が確かめられていないので中止する"
                  "（電源の入れ直しか零点の設定のあとで試す）")
            return
        target = start + args.delta
        print(f"start={start:+.4f} rad  target={target:+.4f} rad  duration={args.duration}s")
        with m.enabled():
            s = m.set_position(target, duration_s=args.duration)
            print(f"reply: result={s.result!r} pos={s.position_rad:+.4f}")
            t0 = time.perf_counter()
            while (t := time.perf_counter() - t0) < args.observe:
                angle = m.read(Ind.OUTPUT_ANGLE)
                iq = m.read(Ind.IQ)
                print(f"t={t:5.2f}s  angle={angle:+.4f} rad  (target-angle={target - angle:+.4f})  iq={iq:+.3f} A")
                time.sleep(0.05)


if __name__ == "__main__":
    main()
