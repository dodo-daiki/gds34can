"""05: 1 本のバスの複数台に、一定周期（既定 100 Hz）で指令を送る周期ループの例。

    python examples/05_cycle_multi_motor.py --ids 1 --seconds 3            # 実機（今は 1 台）
    python examples/05_cycle_multi_motor.py --fake --ids 1 2 3 4            # 擬似ドライバ 4 台

各周期で、全台への速度指令（ゆっくりした正弦波）を Bus.transact_many にまとめて送る。
終わったあと、1 周期の送受信にかかった時間と、周期のずれ（ジッタ）を表示する。
バスが複数ある場合は、バスごとに Bus を作り、別々のスレッドでこのループを回せる（排他はバス単位）。
"""
import math
import statistics
import time
from contextlib import ExitStack

from _common import open_bus, parser

from gds34can import Gds34Error


def main():
    p = parser(__doc__)
    p.add_argument("--ids", type=lambda x: int(x, 0), nargs="+", default=[1])
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--amp", type=float, default=0.5, help="速度の振幅 [rad/s]")
    p.add_argument("--freq", type=float, default=0.5, help="速度の正弦波の周波数 [Hz]")
    args = p.parse_args()
    period = 1.0 / args.hz

    with open_bus(args, fake_ids=args.ids) as bus, ExitStack() as stack:
        motors = [bus.motor(i, max_velocity_rad_s=1.0, default_duration_s=3 * period) for i in args.ids]
        for m in motors:
            m.ensure_no_fault()
            stack.enter_context(m.enabled())        # 抜けるとき、逆順に全台を止める
        busy, starts, errors = [], [], 0
        t0 = time.perf_counter()
        next_tick = t0
        while (now := time.perf_counter()) - t0 < args.seconds:
            starts.append(now)
            v = args.amp * math.sin(2 * math.pi * args.freq * (now - t0))
            results = bus.transact_many([m.cmd_velocity(v) for m in motors])
            errors += sum(isinstance(r, Gds34Error) for r in results)
            busy.append(time.perf_counter() - now)
            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    gaps = [b - a for a, b in zip(starts, starts[1:])]
    print(f"motors={len(motors)} cycles={len(starts)} errors={errors}")
    print(f"送受信時間/周期: median {statistics.median(busy) * 1000:.2f} ms, max {max(busy) * 1000:.2f} ms "
          f"（1 台あたり {statistics.median(busy) / len(motors) * 1000:.2f} ms）")
    print(f"周期: median {statistics.median(gaps) * 1000:.2f} ms, "
          f"p95 {sorted(gaps)[int(len(gaps) * 0.95)] * 1000:.2f} ms, max {max(gaps) * 1000:.2f} ms "
          f"（目標 {period * 1000:.1f} ms）")


if __name__ == "__main__":
    main()
