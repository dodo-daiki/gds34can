"""01: バス上のドライバを探し、状態と主な設定を読む（読み出しだけ。モータは動かない）。

    python examples/01_scan_and_info.py              # 実機（hatcan0）
    python examples/01_scan_and_info.py --fake       # 擬似ドライバ
"""
from _common import open_bus, parser

from gds34can import Conf, Fault, Ind, TESTED_FIRMWARE

KEY_CONF = [Conf.GEAR_RATIO, Conf.MAX_CURRENT, Conf.TORQUE_CONSTANT, Conf.PWM_FREQUENCY,
            Conf.UNDER_VOLTAGE, Conf.OVER_VOLTAGE, Conf.OVER_TEMPERATURE, Conf.PROTOCOL]


def main():
    args = parser(__doc__).parse_args()
    with open_bus(args) as bus:
        found = bus.scan()
        if not found:
            print("応答なし")
            return
        for f in found:
            m = bus.motor(f.motor_id, host_id=f.host_id)
            fault = m.read_fault()
            note = "" if f.version in TESTED_FIRMWARE else "（動作未確認の版）"
            print(f"== id=0x{m.id:03X}  version=0x{f.version:X}{note}  "
                  f"fault={'NONE' if fault == Fault.NONE else repr(fault)}")
            for conf in KEY_CONF:
                print(f"  {conf.name:<18} = {m.read_conf(conf)} {conf.spec.unit}")
            values = m.read_many([Ind.OUTPUT_ANGLE, Ind.DRIVER_TEMP, Ind.MOTOR_TEMP])
            for ind, v in values.items():
                print(f"  {ind.name:<18} = {v:.4f}")


if __name__ == "__main__":
    main()
