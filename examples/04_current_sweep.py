"""04: 速度を段階的に変えて回し、電流（Iq）などを CSV に記録する（電流の把握用）。

    python examples/04_current_sweep.py --rpm 5 10 -10 --seconds 5 --csv sweep.csv --vbus 13.8
    python examples/04_current_sweep.py --fake --seconds 2

1 周期ぶんの要求（速度指令 + インジケータ 3 つ）を Bus.transact_many にまとめて渡す例でもある。
電源電圧はドライバの値が使えない（ファーム 0x21）ので --vbus で手入力し、CSV のメタデータ行に残す。
"""
import csv
import math
import statistics
import time

from _common import open_bus, parser

from gds34can import Gds34Error, Ind, rpm_to_rad_s


def main():
    p = parser(__doc__)
    p.add_argument("--id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--rpm", type=float, nargs="+", default=[5, 10, 20, -5, -10, -20], help="出力軸の RPM の列")
    p.add_argument("--seconds", type=float, default=5.0, help="1 条件あたりの時間")
    p.add_argument("--settle", type=float, default=1.0, help="統計から除く起動直後の時間")
    p.add_argument("--period", type=float, default=0.05)
    p.add_argument("--vbus", type=float, default=None, help="電源電圧 [V]（記録用）")
    p.add_argument("--csv", default=None)
    args = p.parse_args()
    if any(abs(r) > 30 for r in args.rpm):
        p.error("この example では |rpm| <= 30")
    if args.settle >= args.seconds:
        p.error("--settle は --seconds より短くする（統計に使うサンプルがなくなる）")

    rows, summary = [], []
    with open_bus(args) as bus:
        m = bus.motor(args.id, max_velocity_rad_s=math.pi, default_duration_s=3 * args.period)
        m.ensure_no_fault()
        for rpm in args.rpm:
            with m.enabled():
                t0 = time.perf_counter()
                while (t := time.perf_counter() - t0) < args.seconds:
                    state, iq, vq, rpm_out = bus.transact_many([
                        m.cmd_velocity(rpm_to_rad_s(rpm)), m.cmd_read(Ind.IQ), m.cmd_read(Ind.VQ),
                        m.cmd_read(Ind.OUTPUT_SPEED_RPM)])
                    if not any(isinstance(x, Gds34Error) for x in (state, iq, vq, rpm_out)):
                        rows.append({"rpm_cmd": rpm, "t_s": round(t, 4), "pos_rad": state.position_rad,
                                     "vel_rad_s": state.velocity_rad_s, "torque_nm": state.torque_nm,
                                     "temp_c": state.temperature_c, "iq_a": iq, "vq_v": vq, "out_rpm": rpm_out})
                    time.sleep(args.period)
            steady = [r for r in rows if r["rpm_cmd"] == rpm and r["t_s"] > args.settle]
            iqs = [r["iq_a"] for r in steady]
            if not steady:
                print(f"rpm={rpm}: 統計に使えるサンプルがない（送受信がすべて失敗した）")
                continue
            summary.append((rpm, len(steady), statistics.fmean(r["out_rpm"] for r in steady),
                            statistics.fmean(iqs), math.sqrt(statistics.fmean(x * x for x in iqs)),
                            min(iqs), max(iqs), statistics.fmean(r["vq_v"] for r in steady)))
            time.sleep(0.5)

    print(f"{'rpm':>6} {'n':>4} {'out_rpm':>8} {'Iq_mean':>8} {'Iq_rms':>8} {'Iq_min':>8} {'Iq_max':>8} {'Vq_mean':>8}")
    for rpm, n, out, mean, rms, lo, hi, vq in summary:
        print(f"{rpm:6.1f} {n:4d} {out:8.3f} {mean:8.4f} {rms:8.4f} {lo:8.4f} {hi:8.4f} {vq:8.4f}")
    if args.csv and rows:
        with open(args.csv, "w", newline="", encoding="utf-8") as fp:
            fp.write(f"# motor_id={args.id} vbus_v={args.vbus} fake={args.fake} period_s={args.period}\n")
            w = csv.DictWriter(fp, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"csv: {args.csv} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
