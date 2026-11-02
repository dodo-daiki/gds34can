"""07: ドライバの CAN ID / ホスト ID を書き換える（組み立て時の ID 割り当て。README の V1・V2 の確認にも使う）。

    python examples/07_change_id.py --id 1 --new-id 4                    # 計画を表示するだけ
    python examples/07_change_id.py --id 1 --new-id 4 --execute          # 実行
    python examples/07_change_id.py --id 1 --new-host-id 0x101 --execute # ホスト ID（V1 の確認）
    python examples/07_change_id.py --fake --id 1 --new-id 4 --execute

- バス上で応答するドライバが 1 台だけのときしか実行しない（ID の衝突を防ぐ）
- 書いた値は即時に反映され、古い ID では通信できなくなる。新しい値を控えておくこと
- 電源を切っても残るかは未確認（V2）。電源を入れ直してから 01_scan_and_info.py で確かめる
"""
from _common import open_bus, parser

from gds34can import Conf


def main():
    p = parser(__doc__)
    p.add_argument("--id", type=lambda x: int(x, 0), required=True, help="今の CAN ID")
    p.add_argument("--new-id", type=lambda x: int(x, 0))
    p.add_argument("--new-host-id", type=lambda x: int(x, 0))
    p.add_argument("--execute", action="store_true")
    args = p.parse_args()
    if args.new_id is None and args.new_host_id is None:
        p.error("--new-id か --new-host-id を指定する")

    with open_bus(args) as bus:
        found = bus.scan()
        print("responding:", [f"0x{f.motor_id:03X} (reply_from=0x{f.host_id:03X})" for f in found])
        if not args.execute:
            print("計画:", f"CAN ID 0x{args.id:03X} -> 0x{args.new_id:03X}" if args.new_id is not None else "",
                  f"host ID -> 0x{args.new_host_id:03X}" if args.new_host_id is not None else "")
            print("--execute を付けると書き込む")
            return
        motor_id = args.id
        if args.new_id is not None:
            after = bus.change_id(motor_id, args.new_id)
            print(f"CAN ID changed: 0x{motor_id:03X} -> 0x{after.motor_id:03X}")
            motor_id = after.motor_id
        if args.new_host_id is not None:
            after = bus.change_host_id(motor_id, args.new_host_id)
            stored = bus.motor(motor_id, host_id=after.host_id, read_params=False).read_conf(Conf.HOST_CAN_ID)
            print(f"HOST_CAN_ID (read back) = 0x{stored:03X}, replies now come from 0x{after.host_id:03X}")
            if after.host_id == args.new_host_id:
                print("→ V1: 応答の送信元はホスト ID に従う（軸ごとに応答を区別できる）")
            else:
                print("→ V1: 応答の送信元はホスト ID に従わない")


if __name__ == "__main__":
    main()
