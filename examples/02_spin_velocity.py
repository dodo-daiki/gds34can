"""02: 一定の速度で回す（速度指令 0x94 を周期的に送り直す）。

    python examples/02_spin_velocity.py --rad-s 1.047 --seconds 5     # 出力軸 10 RPM で 5 秒
    python examples/02_spin_velocity.py --fake

- 速度は出力軸の rad/s で指定する（中で RPM に換算される）
- Duration は周期の 3 倍。送り直しが止まれば、その時間でモータが止まる想定（README の V3 で未確認）
- with m.enabled() を抜けるとき（Ctrl+C でも）0x97 → 0x92 が送られる
"""
import math
import time

from _common import open_bus, parser


def main():
    p = parser(__doc__)
    p.add_argument("--id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--rad-s", type=float, default=2 * math.pi * 10 / 60, help="出力軸の速度（既定 10 RPM）")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--period", type=float, default=0.1, help="送り直しの周期 [s]")
    p.add_argument("--max-rad-s", type=float, default=math.pi, help="安全のための上限（既定 30 RPM）")
    args = p.parse_args()

    with open_bus(args) as bus:
        m = bus.motor(args.id, max_velocity_rad_s=args.max_rad_s, default_duration_s=3 * args.period)
        m.ensure_no_fault()
        with m.enabled():
            t0 = time.perf_counter()
            next_print = 0.0
            while (t := time.perf_counter() - t0) < args.seconds:
                s = m.set_velocity(args.rad_s)
                if t >= next_print:
                    print(f"t={t:5.2f}s  pos={s.position_rad:+8.3f} rad  vel={s.velocity_rad_s:+6.3f} rad/s  "
                          f"torque={s.torque_nm:+.3f} N.m  temp={s.temperature_c} C")
                    next_print += 0.5
                time.sleep(args.period)
        print("stopped")


if __name__ == "__main__":
    main()
