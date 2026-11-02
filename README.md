# gds34can

SteadyWin の **GDS34** モータドライバ（GIM4310 などに付属）を、**CAN**（Linux の SocketCAN）で扱う Python ライブラリ。
プロトコルは SteadyWin GIM プロトコル rev2.2（CAN 2.0 標準フレーム）。

```python
from gds34can import Bus, Ind

with Bus("can0") as bus:
    m = bus.motor(1)                          # トルクの復号に使う Kt・減速比はここで自動で読む
    with m.enabled():                         # 抜けるとき必ず停止する（例外・Ctrl+C でも）
        s = m.set_velocity(rad_s=1.05)        # 出力軸 rad/s。中でプロトコルの RPM に換算する
        print(s.position_rad, s.velocity_rad_s, s.torque_nm, m.read(Ind.IQ))
```

| | |
|---|---|
| 版 | 0.1.0 |
| 状態 | 擬似ドライバとテスト（30 件）で動作を確認済み。**このライブラリ自体は、まだ実機で動かしていない**（2026-11-02 時点）。ライブラリが前提にしているプロトコルの挙動は、別の bring-up 用スクリプトで実機で確かめてある（[実機で確かめたこと](#8-実機で確かめたこと)） |
| 動作環境 | Python 3.10 以上、python-can 4.0 以上（4.5.0 で確認）。実機は Linux の SocketCAN。擬似ドライバは Windows / macOS でも動く |
| 依存 | python-can だけ |

---

## 目次

1. [概要](#1-概要)
2. [導入](#2-導入)
3. [使い方](#3-使い方)
4. [基本の考え方](#4-基本の考え方)
5. [API リファレンス](#5-api-リファレンス)
6. [安全のしくみ](#6-安全のしくみ)
7. [プロトコル仕様（このライブラリが実装している範囲）](#7-プロトコル仕様このライブラリが実装している範囲)
8. [実機で確かめたこと](#8-実機で確かめたこと)
9. [未確認の挙動](#9-未確認の挙動)
10. [擬似ドライバ（実機なしでの開発・テスト）](#10-擬似ドライバ実機なしでの開発テスト)
11. [CLI](#11-cli)
12. [examples](#12-examples)
13. [テスト](#13-テスト)
14. [構成](#14-構成)
15. [未実装・今後](#15-未実装今後)
16. [出典](#16-出典)

---

## 1. 概要

### できること

- ドライバの探索（`scan`）、版・故障・インジケータ（電流・電圧・角度など 22 項目）・設定項目（30 項目）の読み出し
- 速度・位置・トルク指令。値は**出力軸の SI 単位**（rad・rad/s・N·m・s）で渡し、応答も SI 単位の `ControlState` で受け取る
- 設定項目の書き込み（読み戻して確かめる）、CAN ID・ホスト ID の書き換え（安全な手順つき）
- 1 周期ぶんの複数の要求をまとめて送受信する `transact_many`（周期ループ用）
- 送受信した生フレームとイベント（タイムアウトなど）を受け取るフック（記録用）
- 実機なしで動く擬似ドライバ（python-can の `virtual` インタフェース上で動く）

### やらないこと

- ロボットの構成（関節名・可動範囲・複数バスの周期ループ・軌道生成）は扱わない。このライブラリの上に別のパッケージとして作る想定
- CAN インタフェースの設定（ビットレートなど）は OS 側で行う（[2.2](#22-can-インタフェースの準備linux)）
- MIT プロトコル（ConfID `0x1C` = 1）には対応しない。MIT モードのドライバを見分けて例外にするだけ

### 対象

| 項目 | 内容 |
|---|---|
| ドライバ | SteadyWin GDS34（内部型番 SDC101）。CAN のみのドライバ（RS485 版は GDZ34） |
| モータ | GIM4310-10 で確認。GDS34 を使うほかのモータ（GIM4310-36 など）も同じプロトコル |
| ファームウェア | `0x21` で確認（`TESTED_FIRMWARE`）。ほかの版では警告を表示する（CLI・example） |
| プロトコル | SteadyWin GIM プロトコル rev2.2。CAN 2.0（CAN FD ではない）、標準フレーム 11bit、DLC 8、リトルエンディアン |
| CAN インタフェース | SocketCAN で使えるもの。開発では Raspberry Pi 5 + Waveshare 2-CH CAN HAT+（MCP2515）を使った |

### 名前について

ドライバの型番（GDS34）と通信方式（CAN）を並べた。「GIM」は名前に入れていない。
SteadyWin の**新世代の GIM ドライバ（GDK 系など）は 29bit 拡張フレームのレジスタ方式で、GDS34 とはまったく別のプロトコル**であり、
メーカーのダウンロードページでも同じ「GIM」の名前で並んでいて取り違えやすいため。**新世代の GIM ドライバには、このライブラリは使えない。**

---

## 2. 導入

### 2.1 インストール

```sh
git clone https://github.com/dodo-daiki/gds34can.git
cd gds34can
pip install -e .            # テストも動かすなら pip install -e ".[test]"
```

リポジトリは非公開なので、clone には GitHub の認証（`gh auth login` など）が要る。

Raspberry Pi OS では、apt の `python3-can` をそのまま使えるように `--system-site-packages` 付きの venv に入れるとよい。

```sh
python3 -m venv --system-site-packages ~/venvs/gds34can
~/venvs/gds34can/bin/pip install -e ~/gds34can
```

インストールしなくても、`examples/` はリポジトリから直接実行できる（`examples/_common.py` がパスを通す）。

### 2.2 CAN インタフェースの準備（Linux）

GDS34 の出荷時のビットレートは **500kbps**。ドライバと違うビットレートで送ると ACK が返らず、送信バッファがあふれて
`BusError`（ERROR-PASSIVE）になる。

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100   # classic CAN（GDS34 は CAN FD 非対応）
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -details -statistics link show can0                          # state ERROR-ACTIVE になっていること
```

- インタフェース名は環境に合わせる。CLI と examples の既定値 `hatcan0` は、開発環境で udev ルールによって付けた名前
- バスの両端には 120Ω の終端が要る。ドライバが終端抵抗を内蔵しているかは資料に記載がない

### 2.3 まず読み出しだけで確かめる

```sh
python -m gds34can --iface can0 scan            # 応答するドライバの ID を探す（モータは動かない）
python -m gds34can --iface can0 info --id 1     # 版・故障・全設定を表示する
```

実機がなければ `--fake` で擬似ドライバを相手にできる（`python -m gds34can --fake scan`）。

---

## 3. 使い方

### 3.1 状態と設定を読む

```python
from gds34can import Bus, Conf, Fault, Ind

with Bus("can0") as bus:
    print(bus.scan())                         # [Found(motor_id=1, host_id=0, version=33)]
    m = bus.motor(1)
    print(m.read_version())                   # 33 (= 0x21)
    print(m.read_fault() == Fault.NONE)       # True なら故障なし
    print(m.read_conf(Conf.MAX_CURRENT))      # 8
    print(m.read_many([Ind.IQ, Ind.VQ, Ind.OUTPUT_ANGLE]))   # Ind → 値 の辞書
    print(m.info())                           # 版・故障・全設定をまとめた辞書
```

### 3.2 一定速度で回す

速度・トルク指令には **Duration**（その指令を実行する時間）がある。周期的に送り直し、Duration は周期の数倍にしておく。
送り直しが止まれば、Duration が切れた時点でモータが止まる想定（[V3](#9-未確認の挙動) で未確認）。

```python
import time
from gds34can import Bus

with Bus("can0") as bus:
    m = bus.motor(1, max_velocity_rad_s=3.14, default_duration_s=0.3)
    m.ensure_no_fault()                       # 故障があれば FaultActive
    with m.enabled():                         # 0x91 で起動。抜けるとき 0x97 → 0x92
        for _ in range(50):                   # 0.1 秒ごとに 5 秒間
            s = m.set_velocity(1.047)         # 出力軸 1.047 rad/s（= 10 RPM）
            print(f"{s.position_rad:+.3f} rad  {s.velocity_rad_s:+.3f} rad/s  {s.torque_nm:+.3f} N.m")
            time.sleep(0.1)
```

### 3.3 周期ループ（複数台・複数の要求をまとめる）

`cmd_*` は要求を作るだけで通信しない。1 周期ぶんをリストにして `bus.transact_many` に渡すと、順に送受信して結果をリストで返す。
**失敗した要素は例外を送出せず、その位置に例外オブジェクトが入る**（1 台の失敗で残りの台への指令を止めないため）。

```python
from gds34can import Bus, Gds34Error, Ind

with Bus("can0") as bus:
    m1, m2 = bus.motor(1), bus.motor(2)
    with m1.enabled(), m2.enabled():
        results = bus.transact_many([
            m1.cmd_velocity(0.5, duration_s=0.03),
            m2.cmd_velocity(-0.5, duration_s=0.03),
            m1.cmd_read(Ind.IQ),
        ])
        for r in results:
            if isinstance(r, Gds34Error):
                print("failed:", r)
```

### 3.4 失敗を扱う

```python
from gds34can import Bus, CommandRejected, FaultActive, NoReply

with Bus("can0") as bus:
    m = bus.motor(1)
    try:
        m.ensure_no_fault()
        with m.enabled():
            m.set_velocity(1.0)
    except FaultActive as e:
        print("fault:", e.fault)              # Fault のビットフラグ
        m.clear_fault()                       # 0xB3。原因を取り除いてから
    except CommandRejected as e:
        print(e.result, e.state)              # 結果コードと、制御指令なら応答の状態
    except NoReply as e:
        print("no reply:", e.motor_id, e.label)
```

### 3.5 記録（フック）

```python
def on_frame(direction, msg):                 # direction は "tx" / "rx"、msg は can.Message
    print(direction, hex(msg.arbitration_id), bytes(msg.data).hex(" "))

def on_event(ev):                             # timeout / late_reply / error_frame / bus_error
    print(ev.kind, ev.motor_id, ev.detail)

with Bus("can0", on_frame=on_frame, on_event=on_event) as bus:
    ...
```

フックは送受信の排他の中から呼ばれる。重い処理をしない、フックの中から `Bus` のメソッドを呼ばない。

---

## 4. 基本の考え方

### 4.1 単位

ライブラリの入出力はすべて**出力軸**の値で、単位は SI。プロトコルの単位への換算はライブラリの中で行う。

| 量 | ライブラリ | プロトコル | 換算 |
|---|---|---|---|
| 速度指令（`0x94`） | rad/s | 出力軸 RPM | `rpm = rad_s × 60 / 2π`（実機で確認） |
| 位置指令（`0x95`） | rad | rad | そのまま |
| トルク指令（`0x93`） | N·m | N·m | そのまま（出力軸の値かは未確認） |
| Duration | s | ms（24bit 整数） | `round(s × 1000)`。0.001〜16777.215 s |
| 制御応答の位置・速度・トルク | rad・rad/s・N·m | 16bit・12bit・12bit の整数 | [7.4](#74-制御応答の復号) |
| インジケータ | プロトコルのまま | float32 | 単位は `Ind` の表（[5.6](#56-列挙型)）。`OUTPUT_SPEED_RPM` だけは RPM のまま |
| 設定項目 | プロトコルのまま | int32 / float32 | 単位は `Conf` の表（[5.6](#56-列挙型)） |

### 4.2 通信するメソッド・しないメソッド

- **通信する**: `read_*` / `write_*` / `set_*` と `start` / `stop` / `abort` / `safe_stop` / `clear_fault` / `apply_conf` / `info` / `refresh_params`、
  `Bus.transact` / `transact_many` / `scan` / `ping` / `change_id` / `change_host_id`
- **通信しない**: `cmd_*`。`Command`（要求と、応答を結果に変える関数の組）を作るだけ。`bus.transact(cmd)` で 1 つ、`bus.transact_many([...])` でまとめて送る

`m.set_velocity(v)` は `bus.transact(m.cmd_velocity(v))` と同じ。

### 4.3 送受信はバス単位で 1 つずつ

**出荷時の GDS34 は、ドライバ自身の ID ではなく、ホスト ID と同じ `0x000` から応答する。応答のデータにも送信元の台の情報がない**（1 台で確認）。
出荷時は全台のホスト ID が 0 なので、同じバスに 2 台つないで続けて要求を送ると、どちらの応答か区別できない。そこで `Bus` は、**1 本のバスにつき未応答の要求を常に 1 つまで**に制限している。

- `Bus` は複数のスレッドから呼んでよい。中で排他する（`threading.RLock`）
- `transact_many` は、まとめて渡した要求を排他したまま順に送受信する。途中にほかのスレッドの要求は入らない
- バスが複数あるなら、バスごとに `Bus` を作る。排他はバスごとなので、別々のスレッドで並行して回せる
- **バス上のホスト（要求を送る側）は、このライブラリだけ**であることを前提にしている。送信の直前に、受信済みのフレームを捨てるため

### 4.4 応答の照合

応答は次の条件をすべて満たすフレームだけを受け付ける。満たさないフレームは読み飛ばす。

1. 標準フレームのデータフレーム（拡張フレーム・リモートフレーム・エラーフレームではない）
2. ID が要求先のドライバの ID と違う（同じドライバ宛ての、ほかのホストからの要求は応答ではない）
3. 送信元 ID がドライバのホスト ID と一致する（`Motor` 経由の要求。`ping` / `scan` は送信元を問わない）
4. DLC が 8（DLC 6 なら MIT プロトコルの応答とみなし `WrongProtocol`）
5. 照合キーが要求と一致する: コマンド（BYTE0）に加えて副 ID まで比べる

| コマンド | 照合キー |
|---|---|
| `0x83` 設定変更 / `0x84` 設定取得 | コマンド・ConfType・ConfID |
| `0xB4` インジケータ / `0xA1`・`0xA2` PID | コマンド・IndID（ParaID） |
| そのほか | コマンド |

### 4.5 タイムアウトと遅れて届いた応答

- 応答の待ち時間は `Bus(timeout_s=0.05)`（既定 50 ms）。要求ごとに `transact(cmd, timeout_s=...)` で変えられる
- タイムアウトしたら `NoReply` を送出し、そのあと `late_reply_guard_s`（既定は `timeout_s` と同じ）だけ次の送信を待つ
- 送信の直前に、受信済みのフレームをすべて捨てる。遅れて届いた応答を、次の要求の応答と取り違えないため（`late_reply` イベントが出る）

### 4.6 止め方

| 方法 | 送るもの | 使いどころ |
|---|---|---|
| `with m.enabled():` を抜ける | `0x97`（中止）→ `0x92`（停止） | 通常。例外・Ctrl+C でも必ず送る |
| `m.safe_stop()` | `0x97` → `0x92` | 途中で失敗しても両方送り、起きた例外のリストを返す |
| `m.abort()` | `0x97` | 実行中の制御だけ中止する |
| `m.stop()` | `0x92` | 停止 |
| `python -m gds34can stop --id 1` | `0x97` → `0x92` | 別の端末から止める（動かしているプロセスが指令を送り続けていると、また動きうる） |
| Duration が切れる | — | 送り直しが止まった場合の最後の砦（想定。V3 で未確認） |

`0x92` で停止したあと、出力軸が手で回るか（逆駆動できるか）は未確認（V11）。

### 4.7 故障

ドライバは故障（過電流・過電圧など）を検出するとモータを止め、**`0xB3`（`clear_fault()`）で解除するまで全コマンドを拒否する**（プロトコル仕様）。
プロトコル仕様は、ホストが `0xB2`（`read_fault()`）で定期的に故障を確認することを求めている。故障中の制御指令に何が返るかは未確認（V10）。

---

## 5. API リファレンス

公開 API は `gds34can` から import できるものだけ（`gds34can.__all__`）。`gds34can.codec`（符号化・復号の関数）と
`gds34can.fake`（擬似ドライバ）も import できるが、`codec` は内部向けで、版が上がると変わることがある。

### 5.1 `Bus`

1 本の CAN バス。`with` で使うと抜けるときに閉じる。

```python
Bus(channel="hatcan0", *, interface="socketcan", timeout_s=0.05, late_reply_guard_s=None,
    on_frame=None, on_event=None, can_bus=None)
```

| 引数 | 既定 | 内容 |
|---|---|---|
| `channel` | `"hatcan0"` | python-can に渡すチャネル名（SocketCAN ならインタフェース名） |
| `interface` | `"socketcan"` | python-can のインタフェース。擬似ドライバは `"virtual"` |
| `timeout_s` | `0.05` | 応答の待ち時間の既定値 [s] |
| `late_reply_guard_s` | `None` | タイムアウトのあと次の送信までに待つ時間 [s]。`None` なら `timeout_s` と同じ |
| `on_frame` | `None` | `on_frame(direction, msg)`。送受信した全フレーム。`direction` は `"tx"` / `"rx"` |
| `on_event` | `None` | `on_event(event: Event)`。タイムアウトなどのイベント |
| `can_bus` | `None` | 作成済みの `can.BusABC` を使う場合（テスト用）。`close()` では閉じない |

| メソッド | 戻り値 | 内容 |
|---|---|---|
| `motor(motor_id, **kwargs)` | `Motor` | このバス上のドライバ 1 台。`kwargs` は [`Motor`](#52-motor) の引数 |
| `scan(ids=range(0x01, 0x80), timeout_s=0.02)` | `list[Found]` | 各 ID に `0xB1` を送り、応答した ID を返す。見つかった ID には通常のタイムアウトでもう一度送って確かめる（取り違え防止）。応答がなければ 1 ID あたり `timeout_s` かかる（既定の範囲で約 2.5 秒） |
| `ping(motor_id, timeout_s=None)` | `Found \| None` | `0xB1` を 1 回送る。応答がなければ `None` |
| `transact(command, timeout_s=None)` | 要求に応じた値 | 1 つ送受信して、`command.decode` の結果を返す。失敗は例外 |
| `transact_many(commands, timeout_s=None)` | `list` | 順に送受信し、結果のリストを返す。失敗した要素には例外オブジェクトが入る（送出しない）。全体を排他したまま行う |
| `change_id(motor_id, new_id, *, scan_ids=range(0x01, 0x80))` | `Found` | ドライバの CAN ID を書き換える（[6](#6-安全のしくみ)）。書いたあと、新しい ID が応答し古い ID が応答しないことを確かめる |
| `change_host_id(motor_id, new_host_id, *, scan_ids=range(0x01, 0x80))` | `Found` | ホスト CAN ID を書き換え、書いたあとの応答の送信元を `Found.host_id` で返す。以後は `bus.motor(motor_id, host_id=返り値.host_id)` で作り直す |
| `can_state()` | `str \| None` | SocketCAN の状態（`ERROR-ACTIVE` / `ERROR-PASSIVE` / `BUS-OFF` など）。`ip -details -json link show` で読む。socketcan 以外では `None` |
| `close()` | — | バスを閉じる |

`change_id` / `change_host_id` は、**バス上で応答するドライバが `motor_id` の 1 台だけ**でなければ `GuardedSetting` を送出する。
`new_id` は `0x001`〜`0x7FF` でホスト ID・今の ID と違う値、`new_host_id` は `0x000`〜`0x7FF` でドライバの ID と違う値でなければ `ValueError`。

### 5.2 `Motor`

ドライバ 1 台。`bus.motor(motor_id, ...)` で作る。

```python
Motor(bus, motor_id, *, host_id=0, default_duration_s=0.3, max_velocity_rad_s=None, max_torque_nm=None,
      read_params=True)
```

| 引数 | 既定 | 内容 |
|---|---|---|
| `motor_id` | — | ドライバの CAN ID（`0x001`〜`0x7FF`、`host_id` と違う値） |
| `host_id` | `0` | ドライバに設定されたホスト ID（応答の送信元）。出荷時は 0 |
| `default_duration_s` | `0.3` | 制御指令の Duration を省略したときの値 [s]。周期的に送り直すなら周期の数倍にする |
| `max_velocity_rad_s` | `None` | 速度指令の上限（絶対値）。超えると `LimitExceeded`。`None` なら制限しない |
| `max_torque_nm` | `None` | トルク指令の上限（絶対値）。同上 |
| `read_params` | `True` | `True` なら作るときに減速比と Kt を読む（トルクの復号に使う）。応答がなければ `NoReply` |

| 属性 | 内容 |
|---|---|
| `bus`, `id`, `host_id` | 引数のとおり |
| `default_duration_s`, `max_velocity_rad_s`, `max_torque_nm` | 引数のとおり。あとから変えてよい |
| `gear_ratio` | 減速比（`Conf.GEAR_RATIO`）。`read_params=False` なら `None` |
| `kt` | ドライバ内部のトルク定数（`Conf.TORQUE_CONSTANT`、**モータ軸**）[N·m/A]。同上 |

**要求を作るだけ（通信しない）** — すべて `Command` を返す。

| メソッド | 送るコマンド | `transact` の結果 |
|---|---|---|
| `cmd_velocity(rad_s, duration_s=None)` | `0x94` | `ControlState` |
| `cmd_position(rad, duration_s=None)` | `0x95` | `ControlState` |
| `cmd_torque(nm, duration_s=None)` | `0x93` | `ControlState` |
| `cmd_read(ind)` | `0xB4` | `float` |
| `cmd_read_conf(conf)` | `0x84` | `int` / `float` |
| `cmd_version()` | `0xB1` | `int` |
| `cmd_fault()` | `0xB2` | `Fault` |
| `cmd_start()` / `cmd_stop()` / `cmd_abort()` / `cmd_clear_fault()` | `0x91` / `0x92` / `0x97` / `0xB3` | `None` |

`duration_s=None` なら `default_duration_s` を使う。上限を超える値は、要求を作る時点で `LimitExceeded` になる。

**状態・設定の読み出し**

| メソッド | 戻り値 | 内容 |
|---|---|---|
| `read_version()` | `int` | ファームウェアの版（`0x21` など） |
| `read_fault()` | `Fault` | 故障のビットフラグ。正常なら `Fault.NONE` |
| `ensure_no_fault()` | — | 故障があれば `FaultActive` |
| `read(ind)` | `float` | インジケータを 1 つ読む。`BUS_VOLTAGE` / `POWER` は `StaleIndicatorWarning` を出す |
| `read_many(inds)` | `dict[Ind, float]` | まとめて読む。1 つでも失敗すればその例外を送出する |
| `read_conf(conf)` | `int` / `float` | 設定項目を 1 つ読む |
| `read_all_conf()` | `dict[Conf, int \| float \| Gds34Error]` | 全設定項目を読む。読めなかった項目には例外オブジェクトが入る（実験条件の記録用） |
| `refresh_params()` | — | 減速比と Kt を読み直す |
| `info()` | `dict` | `{"id", "host_id", "version", "fault", "conf"}`（表示・記録用） |

**設定の書き込み**

| メソッド | 内容 |
|---|---|
| `write_conf(conf, value, *, allow_motor_model=False)` | 書いて読み戻し、一致を確かめる。**書く前の値を返す**（記録に残すため）。一致しなければ `VerifyFailed`。減速比・Kt を書いたら `refresh_params()` も行う。int32 の項目に整数でない値を渡すと `ValueError` |
| `apply_conf()` | `0x82`（設定反映）を送る。何のために要るのか（電源断後も残すためか）は未確認（V2） |

`write_conf` で書けない項目: 読み取り専用（`POWER_OFF_POSITION`）、通信設定（`CAN_ID` / `HOST_CAN_ID` / `CAN_BITRATE` / `PROTOCOL`。
ID は `Bus.change_id` / `change_host_id` を使う）。モータの物理パラメータ（`Guard.MOTOR_MODEL`）は `allow_motor_model=True` が要る。
いずれも `GuardedSetting` を送出する。

**起動・停止・制御**

| メソッド | 戻り値 | 内容 |
|---|---|---|
| `start()` | — | `0x91` 起動 |
| `stop()` | — | `0x92` 停止 |
| `abort()` | — | `0x97` 実行中の制御を中止 |
| `clear_fault()` | — | `0xB3` 故障解除 |
| `safe_stop()` | `list[Gds34Error]` | `0x97` → `0x92`。途中で失敗しても両方送る |
| `enabled()` | コンテキストマネージャ | 入るとき `start()`、抜けるとき `safe_stop()` |
| `set_velocity(rad_s, duration_s=None)` | `ControlState` | 速度指令（出力軸 rad/s） |
| `set_position(rad, duration_s=None)` | `ControlState` | 位置指令（出力軸 rad）。**補間の有無・多回転か・電源を入れ直したあとの基準が未確認**（V4・V5）。確かめるまでは小さい角度・長い Duration で使う |
| `set_torque(nm, duration_s=None)` | `ControlState` | トルク指令 [N·m]。出力軸の値かは未確認 |

制御指令の結果コードが SUCCESS 以外なら `CommandRejected` を送出し、`e.state` に応答の `ControlState` が入る。

### 5.3 `ControlState`

制御指令（`0x93` / `0x94` / `0x95`）の応答。frozen dataclass。

| フィールド | 型 | 内容 |
|---|---|---|
| `result` | `Result \| int` | 結果コード |
| `temperature_c` | `int` | 温度 [℃]（モータとドライバのうち高い方） |
| `position_rad` | `float` | 出力軸の位置。**±12.5 rad で折り返す**。多回転の角度は `read(Ind.OUTPUT_ANGLE)` で読む |
| `velocity_rad_s` | `float` | 出力軸の速度（±65 rad/s） |
| `torque_nm` | `float \| None` | トルク。Kt・減速比が分からない（`read_params=False`）ときは `None` |
| `raw_position` / `raw_velocity` / `raw_torque` | `int` | 換算前の整数（16bit / 12bit / 12bit） |
| `timestamp` | `float \| None` | 受信時刻 [s]（python-can の `Message.timestamp`。SocketCAN ではエポック秒） |

分解能は [7.4](#74-制御応答の復号)。トルク欄は 12bit で粗い（GIM4310-10 で 1LSB ≈ 0.093 N·m）。細かいトルクが要るなら `Ind.IQ` を読む。

### 5.4 データクラス

| クラス | フィールド | 内容 |
|---|---|---|
| `Found` | `motor_id`, `host_id`, `version` | `scan` / `ping` の結果。`host_id` は応答の送信元 ID |
| `Event` | `time`, `kind`, `motor_id`, `detail` | `on_event` に渡す。`time` はエポック秒。`kind` は下表 |
| `Request` | `motor_id`, `data`, `reply_from`, `label` | 送る要求。`reply_from` は応答を受け付ける送信元 ID（`None` なら問わない） |
| `Reply` | `motor_id`, `arbitration_id`, `data`, `timestamp` | 受け取った応答 |
| `Command` | `request`, `decode` | 要求と、`Reply` を結果に変える関数の組。`cmd_*` が作る |

| `Event.kind` | 出るとき |
|---|---|
| `timeout` | 応答がタイムアウトした（`NoReply` の直前） |
| `late_reply` | 送信の直前に、受信済みの古いフレームを捨てた |
| `error_frame` | 受信待ちの間に CAN のエラーフレームを受けた |
| `bus_error` | 送信に失敗した（`BusError` の直前） |

### 5.5 関数・定数

| 名前 | 内容 |
|---|---|
| `rad_s_to_rpm(rad_s)` / `rpm_to_rad_s(rpm)` | 単位換算 |
| `BLOCKED_COMMANDS` | 送信禁止のコマンド（`0x81` / `0x96` / `0xB5` / `0xC1`） |
| `STALE_INDICATORS` | ファーム `0x21` で値が更新されないインジケータ（`BUS_VOLTAGE` / `POWER`） |
| `TESTED_FIRMWARE` | 動作を確認したファームウェアの版（`{0x21}`） |
| `StaleIndicatorWarning` | `STALE_INDICATORS` を読んだときの警告（`UserWarning` の派生） |
| `__version__` | `"0.1.0"` |

### 5.6 列挙型

#### `Cmd`（コマンド。フレームの BYTE0）

| 名前 | 値 | 内容 | ライブラリでの扱い |
|---|---|---|---|
| `RESET_CONF` | `0x81` | 全設定を初期値に戻す | **送信禁止**（相順識別・エンコーダ校正のやり直しが必要になる） |
| `APPLY_CONF` | `0x82` | 設定反映 | `apply_conf()` |
| `WRITE_CONF` | `0x83` | 設定変更 | `write_conf()`、`change_id()`、`change_host_id()` |
| `READ_CONF` | `0x84` | 設定取得 | `read_conf()` |
| `START` | `0x91` | 起動 | `start()` |
| `STOP` | `0x92` | 停止 | `stop()` |
| `TORQUE` | `0x93` | トルク制御 | `set_torque()` |
| `VELOCITY` | `0x94` | 速度制御 | `set_velocity()` |
| `POSITION` | `0x95` | 位置制御 | `set_position()` |
| `PTS` | `0x96` | PTS 制御 | **送信禁止**（GDS34 は非対応） |
| `ABORT` | `0x97` | 実行中の制御を中止 | `abort()` |
| `WRITE_PID` / `READ_PID` | `0xA1` / `0xA2` | 実行時 PID の変更・取得 | 未実装 |
| `VERSION` | `0xB1` | 版の取得 | `read_version()`、`ping()`、`scan()` |
| `FAULT` | `0xB2` | 故障の取得 | `read_fault()` |
| `CLEAR_FAULT` | `0xB3` | 故障の解除 | `clear_fault()` |
| `INDICATOR` | `0xB4` | インジケータの取得 | `read()` |
| `CALIBRATE` | `0xB5` | キャリブレーション | **送信禁止**（モータが正逆に回る） |
| `FIRMWARE_UPDATE` | `0xC1` | ファーム更新モードに入る | **送信禁止**（応答せずに更新モードに入る） |

#### `Result`（結果コード）

`SUCCESS` 0x00 / `FAILURE` 0x01 / `UNKNOWN_COMMAND` 0x02 / `UNKNOWN_ID` 0x03 / `READ_ONLY` 0x04 / `UNKNOWN_REGISTER` 0x05 /
`STRING_FORMAT` 0x06 / `DATA_FORMAT` 0x07 / `WRITE_ONLY` 0x0B。表にない値は `int` のまま返す。

#### `Fault`（故障。`IntFlag` なので複数が同時に立ちうる）

`NONE` 0x00 / `FOC_FREQUENCY` 0x01（FOC 周波数過大）/ `OVER_VOLTAGE` 0x02 / `UNDER_VOLTAGE` 0x04 / `OVER_TEMPERATURE` 0x08 /
`START_FAILURE` 0x10 / `OVER_CURRENT` 0x40 / `SOFTWARE` 0x80（ソフトウェア例外）

#### `Ind`（インジケータ。値はすべて float32）

| 名前 | IndID | 内容 | 名前 | IndID | 内容 |
|---|---|---|---|---|---|
| `BUS_VOLTAGE` | 0x00 | バス電圧 [V]（**ファーム 0x21 では更新されない**） | `ID_TARGET` | 0x0C | 目標 Id [A] |
| `DRIVER_TEMP` | 0x01 | ドライバ基板の温度 | `VQ` / `VD` | 0x0D / 0x0E | Vq / Vd [V] |
| `MOTOR_TEMP` | 0x02 | モータの温度 | `V_ALPHA` / `V_BETA` | 0x0F / 0x10 | Vα / Vβ [V] |
| `POWER` | 0x03 | 電力 [W]（**ファーム 0x21 では更新されない**） | `ROTOR_ELEC_ANGLE` | 0x11 | ロータの電気角 [rad] |
| `IA` / `IB` / `IC` | 0x04〜0x06 | 相電流 [A] | `ROTOR_MECH_ANGLE` | 0x12 | ロータの機械角 [rad] |
| `I_ALPHA` / `I_BETA` | 0x07 / 0x08 | Iα / Iβ [A] | `OUTPUT_ANGLE` | 0x13 | **出力軸の角度 [rad]（多回転。折り返さない）** |
| `IQ` / `ID` | 0x09 / 0x0A | Iq / Id [A] | `OUTPUT_SPEED_RPM` | 0x14 | 出力軸の速度 [RPM] |
| `IQ_TARGET` | 0x0B | 目標 Iq [A] | `OUTPUT_POWER` | 0x15 | 出力の電力 [W] |

インジケータは 1 要求で 1 項目しか読めない。複数を読むと時刻がずれる（1 項目あたり約 0.7 ms。推定。[8](#8-実機で確かめたこと)）。
Iq・Vq などは FOC が動いている間（起動中）だけ更新される。

#### `Conf`（設定項目）

値は `ConfSpec(conf_type, conf_id, unit, guard, applies_after_restart, note)`。`conf.spec` で取り出せる。
`conf.is_float` は float32 の項目なら `True`。`Conf.lookup(conf_type, conf_id)` で逆引きできる。
「実機の値」は GIM4310-10（ファーム 0x21）を出荷時の状態で読み出した値（2026-10-24）。

| 名前 | Type / ID | 単位 | 保護 | 実機の値 | 備考 |
|---|---|---|---|---|---|
| `POLE_PAIRS` | 0x00 / 0x00 | — | MOTOR_MODEL | 14 | 極対数 |
| `MAX_CURRENT` | 0x00 / 0x01 | A | — | 8 | プロトコル上の名前は Rated Current。設定アプリの「最大电流」。電流を本当に制限するかは未確認（V6） |
| `MAX_SPEED` | 0x00 / 0x02 | RPM | — | 3300 | ロータ側の回転数 |
| `RATED_VOLTAGE` | 0x00 / 0x06 | V | — | 24 | |
| `PWM_FREQUENCY` | 0x00 / 0x07 | Hz | — | 30000 | **再起動後に反映** |
| `CURRENT_KP` / `CURRENT_KI` | 0x00 / 0x08, 0x09 | — | — | 1000 / 200 | 電流ループの既定ゲイン |
| `SPEED_KP` / `SPEED_KI` | 0x00 / 0x0C, 0x0D | — | — | 2000 / 200 | 速度ループの既定ゲイン |
| `POSITION_KP` / `POSITION_KI` / `POSITION_KD` | 0x00 / 0x0E〜0x10 | — | — | 1 / 0 / 1000 | 位置ループの既定ゲイン。設定アプリのマニュアルでは、Ki は「ほとんどの場合 0 にすべき（振動で損傷しうる）」 |
| `GEAR_RATIO` | 0x00 / 0x11 | — | MOTOR_MODEL | 10 | 減速比 |
| `CAN_ID` | 0x00 / 0x12 | — | COMMUNICATION | 1 | 書いた瞬間に反映される |
| `HOST_CAN_ID` | 0x00 / 0x13 | — | COMMUNICATION | 0 | 同上 |
| `ZERO_POSITION` | 0x00 / 0x14 | raw16 | — | 193 | 出力軸の零点。rad = 値 × 2π / 65536。単回転か多回転かは資料どうしで食い違う |
| `POWER_OFF_POSITION` | 0x00 / 0x15 | raw16 | READ_ONLY | 0 | 電源断時の位置 |
| `OVER_VOLTAGE` / `UNDER_VOLTAGE` | 0x00 / 0x16, 0x17 | V | — | 40 / 12 | 過電圧・低電圧のしきい値 |
| `CAN_BITRATE` | 0x00 / 0x18 | ×100kbps | COMMUNICATION | 5 | 5 = 500kbps は実機で確認。ほかの値との対応は未確認 |
| `FIELD_WEAKENING_KP` / `_KI` | 0x00 / 0x19, 0x1A | — | — | 未読 | 弱め界磁の既定ゲイン |
| `PROTOCOL` | 0x00 / 0x1C | — | COMMUNICATION | 0 | 0 = GIM（このライブラリ）/ 1 = MIT |
| `OVER_TEMPERATURE` | 0x00 / 0x20 | — | — | 80 | 過温度のしきい値 |
| `PHASE_RESISTANCE` | 0x01 / 0x00 | Ω | MOTOR_MODEL | 0.324 | 相抵抗 |
| `PHASE_INDUCTANCE` | 0x01 / 0x01 | H | MOTOR_MODEL | 0.000231 | 相インダクタンス |
| `BACK_EMF` | 0x01 / 0x02 | Vrms/kRPM | MOTOR_MODEL | 4.0 | 逆起電力定数 |
| `TORQUE_CONSTANT` | 0x01 / 0x03 | N·m/A | MOTOR_MODEL | 0.08472 | **モータ軸の値**（カタログのトルク定数 0.96 N·m/A とは別物） |
| `SHUNT_RESISTANCE` | 0x01 / 0x04 | Ω | MOTOR_MODEL | 未読 | 電流検出抵抗 |
| `AMP_GAIN` | 0x01 / 0x05 | — | MOTOR_MODEL | 未読 | 増幅ゲイン |

#### `Guard`（設定項目の保護の種類）

| 値 | 内容 | `write_conf` |
|---|---|---|
| `NONE` | 保護なし | 書ける |
| `MOTOR_MODEL` | モータの物理パラメータ。実物と違う値を書くと正常に回らない | `allow_motor_model=True` のときだけ書ける |
| `COMMUNICATION` | 書いた瞬間に反映されて通信が切れる | 書けない（`Bus.change_id` / `change_host_id`） |
| `READ_ONLY` | 読み取り専用 | 書けない |

### 5.7 例外

すべて `Gds34Error` の派生。

| 例外 | 送出するとき | 属性 |
|---|---|---|
| `NoReply` | タイムアウトまでに応答がなかった | `motor_id`, `label`, `timeout_s` |
| `CommandRejected` | 結果コードが SUCCESS 以外だった | `motor_id`, `label`, `result`, `state`（制御指令なら `ControlState`、それ以外は `None`） |
| `FaultActive` | `ensure_no_fault()` で故障があった | `motor_id`, `fault` |
| `WrongProtocol` | DLC 6 の応答を受けた（ドライバが MIT プロトコルになっている） | — |
| `BusError` | 送信に失敗した（ACK が返らない・ERROR-PASSIVE・BUS-OFF など） | `state`（SocketCAN の状態。取れなければ `None`） |
| `BlockedCommand` | 送信禁止のコマンドを送ろうとした | — |
| `GuardedSetting` | 保護された設定項目を、必要な手順・許可なしに書こうとした | — |
| `LimitExceeded` | `max_velocity_rad_s` / `max_torque_nm` を超える指令を作ろうとした | — |
| `VerifyFailed` | 書き込み後の読み戻しや、ID 変更後の確認が一致しなかった | — |
| `ProtocolError` | 応答の形がプロトコル仕様と合わない | — |

値の範囲の誤り（Duration が範囲外、`nan` の指令、不正な ID など）は `ValueError`。

---

## 6. 安全のしくみ

| しくみ | 内容 |
|---|---|
| 送信禁止 | `0x81`（全リセット）・`0x96`（非対応）・`0xB5`（キャリブレーション）・`0xC1`（ファーム更新）は、どの経路からも送れない（`BlockedCommand`） |
| 通信設定の保護 | CAN ID・ホスト ID・ボーレート・プロトコルは `write_conf` では書けない。ID は `Bus.change_id` / `change_host_id` で、**バス上で応答するドライバが 1 台だけ**のときに限り書き、書いたあと確かめる（ID の衝突を防ぐ） |
| モータの物理パラメータの保護 | 極対数・減速比・Rs・Ls・Kt などは `allow_motor_model=True` がないと書けない |
| 書き込みの確認 | `write_conf` は読み戻して一致を確かめ、書く前の値を返す |
| 必ず止める | `with motor.enabled():` を抜けるとき（例外・Ctrl+C でも）`0x97` → `0x92` を送る |
| Duration | 速度・トルク指令の Duration は省略時 0.3 秒。送り直しが止まれば止まる想定（**未確認: V3**）。Duration 0 は扱いが未確認なので受け付けない |
| 指令の上限 | `max_velocity_rad_s` / `max_torque_nm` を超える指令は送らない（任意） |
| 取り違え防止 | 応答はコマンドと副 ID まで照合する。タイムアウトのあとは少し待ってから次を送り、遅れて届いた応答を捨てる |
| 使えない値の警告 | バス電圧と電力のインジケータはファーム 0x21 では更新されない。読むと `StaleIndicatorWarning` |
| examples の安全策 | 動かす example は速度の上限つき。位置指令（03）と ID の書き換え（07）は、確認のフラグがなければ何もしない |

**このライブラリの安全策はソフトウェアだけ**。非常停止（電源の遮断）は別に用意すること。

---

## 7. プロトコル仕様（このライブラリが実装している範囲）

SteadyWin GIM プロトコル rev2.2 の要点と、このライブラリでの実装。符号化・復号は `gds34can/codec.py`。

### 7.1 物理層

- CAN 2.0（CAN FD ではない）、標準フレーム 11bit、データフレーム、DLC 8、バイト順はリトルエンディアン
- ビットレートは 1Mbps 以下で、ConfID `0x18` で変えられる。出荷時は 500kbps（実機で確認。資料には記載がない）
- 要求はドライバの CAN ID（ConfID `0x12`）へ送る。応答の送信元 ID は、出荷時の状態では `0x000`（ホスト ID（ConfID `0x13`）の値と同じ）だった（実機で確認）。
  ホスト ID を変えたときに送信元も変わるかは未確認（V1）
- 応答の BYTE0 は要求のコマンドのエコー。結果コード（RES）の位置はコマンドによって違う（下表）

### 7.2 フレームの構成

| コマンド | 要求（BYTE0〜7） | 応答（BYTE0〜7） | RES の位置 |
|---|---|---|---|
| `0x82` `0x91` `0x92` `0x97` `0xB3` | `CMD` | `CMD RES` | BYTE1 |
| `0x83` 設定変更 | `0x83 ConfType ConfID 00 DATA0..3` | `0x83 ConfType ConfID RES` | BYTE3 |
| `0x84` 設定取得 | `0x84 ConfType ConfID` | `0x84 ConfType ConfID RES DATA0..3` | BYTE3 |
| `0x93` `0x94` `0x95` 制御 | `CMD float32(値) uint24(Duration ms)` | `CMD RES Temp Pos0 Pos1 ST0 ST1 ST2` | BYTE1 |
| `0xB1` 版 | `0xB1` | `0xB1 RES 00 00 uint32(版)` | BYTE1 |
| `0xB2` 故障 | `0xB2` | `0xB2 RES FaultNo` | BYTE1 |
| `0xB4` インジケータ | `0xB4 IndID` | `0xB4 IndID RES 00 float32(値)` | BYTE2 |
| `0xA1` / `0xA2` PID（未実装） | `CMD ParaID ...` | `CMD ParaID RES ...` | BYTE2 |

- 要求の残りのバイトは 0 で埋めて、常に DLC 8 で送る
- ConfType `0x00` は int32、`0x01` は float32（どちらもリトルエンディアン）。プロトコル仕様書の int32 の復元式は `DATA2<<8` と誤記されている（正しくは `DATA1<<8`）

### 7.3 制御指令

`[CMD | float32 値 | uint24 Duration]`。値の単位は `0x93` が N·m、`0x94` が**出力軸 RPM**（実機で確認）、`0x95` が rad。
Duration は「その指令を実行する時間 [ms]」で、1〜16777215（`0xFFFFFF`）。0 の扱いは仕様書に記載がないので、ライブラリは 0 を送らない。

### 7.4 制御応答の復号

`[CMD | RES | Temp | Pos0 | Pos1 | ST0 | ST1 | ST2]`

| 量 | ビット構成 | 換算式 | 範囲 | 1LSB |
|---|---|---|---|---|
| 温度 | Temp（uint8） | そのまま [℃]（モータとドライバのうち高い方） | — | 1 ℃ |
| 位置 | `Pos0 \| Pos1<<8`（16bit） | `int × 25/65535 − 12.5` [rad] | ±12.5 rad（約 ±2 回転）で折り返す | 3.8×10⁻⁴ rad |
| 速度 | `ST0<<4 \| ST1>>4`（12bit） | `int × 130/4095 − 65` [rad/s] | ±65 rad/s | 0.0317 rad/s |
| トルク | `(ST1 & 0x0F)<<8 \| ST2`（12bit） | `int × 450·Kt·GR/4095 − 225·Kt·GR` [N·m] | ±225·Kt·GR | 450·Kt·GR/4095 |

Kt はドライバ内部のトルク定数（`Conf.TORQUE_CONSTANT`、モータ軸）、GR は減速比。GIM4310-10 の実機の値（Kt = 0.08472、GR = 10）では、
トルクの範囲は ±190.6 N·m、1LSB は 0.093 N·m（式からの計算値）。定格トルク（2.05 N·m）に比べて粗い。

### 7.5 MIT モードの見分け方

ドライバが MIT プロトコル（ConfID `0x1C` = 1）になっていると、GIM のコマンドが一切通らない。MIT の応答は DLC 6 なので、
ライブラリは DLC 6 の応答を受けると `WrongProtocol` を送出する。`scan` に何も応答しないのに `candump` で DLC 6 のフレームが見えたら、MIT モードを疑う。
プロトコルの設定は、メーカーの設定アプリ（Motor Wizard）でも変えられる（資料による。試していない）。

---

## 8. 実機で確かめたこと

2026-10-24、GIM4310-10（GDS34 付き）1 台、ファーム `0x21`、500kbps、Raspberry Pi 5 + Waveshare 2-CH CAN HAT+、無負荷。
このライブラリより前の bring-up 用スクリプトで確かめた。**このライブラリ自体で同じ結果になるかは、まだ確かめていない。**

| 項目 | 結果 |
|---|---|
| 出荷時の通信設定 | CAN ID 1、ホスト ID 0、500kbps（ConfID `0x18` = 5）、プロトコル GIM（`0x1C` = 0）。1Mbps では ACK が返らず ERROR-PASSIVE になった |
| 応答の送信元 ID | `0x000`（ホスト ID の値と同じ。ドライバの ID 1 ではない）→ [4.3](#43-送受信はバス単位で-1-つずつ) の排他の根拠 |
| `0x94` の単位 | 出力軸 RPM。10 RPM（1.047 rad/s）の指令に対し、30 秒間の位置の傾きが 1.049 rad/s |
| `Conf.TORQUE_CONSTANT` | 0.08472 N·m/A。カタログのトルク定数 0.96 N·m/A（減速後の値とみられる）を減速比 10 で割った値に近く、モータ軸の値と判断した |
| 制御応答の位置欄 | ±12.5 rad で折り返す（11.504 → −12.432 rad） |
| バス電圧・電力のインジケータ | 回転中も 2.552772 V・0 W のまま変わらない（Iq・Vq は回転中に更新される）→ 更新されないとみなし、警告を出す |
| 送受信 1 回の時間 | 約 0.7 ms（推定。Python + python-can、Pi 5。ループ周期の中央値から sleep の時間を引いて逆算した） |
| 速度の追従（無負荷、電源 13.8V） | ±5〜±30 RPM の指令に対し、位置の傾きは指令の ±0.3% 以内 |
| 無負荷の Iq（電源 13.8V） | 平均 0.12〜0.17 A（速度によらずほぼ一定）。1 回の計測の中で変動し、ピークは +5 RPM で 0.55 A、+30 RPM で 0.30 A |
| Vq | 速度にほぼ比例（+10 → +30 RPM で 0.183 → 0.404 V） |
| Rs / Ls の設定値 | 0.324 Ω / 0.231 mH。カタログ（相間 1.89 Ω / 0.75 mH）と合わないが、出荷時の設定のまま回転した |

---

## 9. 未確認の挙動

実機で確かめてから頼ること。擬似ドライバ（[10](#10-擬似ドライバ実機なしでの開発テスト)）は、確かめるまで右の列の**仮定**で動いている。
確かめたら、このライブラリと擬似ドライバをそれに合わせる。

| | 未確認のこと | 擬似ドライバの仮定 | 確かめ方 |
|---|---|---|---|
| V1 | ホスト ID を変えると、応答の送信元も変わるか（変われば、台ごとに応答を区別でき、`transact_many` でまとめて送れる） | 変わる | `examples/07_change_id.py --id 1 --new-host-id 0x101 --execute` |
| V2 | `0x83` で書いた設定が電源を切っても残るか。`0x82`（設定反映）が要るか | — | 害のない項目（過温度しきい値など）を書き、電源を入れ直して `01_scan_and_info.py` で読む |
| V3 | Duration が切れたあとの挙動（トルク・速度・位置のそれぞれ）。Duration = 0 の扱い | 速度・トルクは止まる | 低速で指令を 1 回だけ送り、送るのをやめて位置と Iq を記録する |
| V4 | 位置指令は Duration の時間をかけて目標へ補間するのか、すぐに目標へ向かうのか | 直線補間する | `examples/03_position_step.py --delta 0.2 --duration 1.0 --confirm` |
| V5 | 電源を入れ直したあとの出力軸の位置（電源が切れている間に手で回した場合も） | — | 電源断中に出力軸を既知の角度だけ回し、入れ直して `Ind.OUTPUT_ANGLE` を外部の基準と比べる |
| V6 | `MAX_CURRENT`（`0x01`）が本当に電流を制限するか | — | 低い値にして、負荷をかけたときの Iq を見る |
| V7 | `0x91` だけを送ったとき、モータは保持するのか、空転するのか | — | 起動だけして、出力軸を手で軽く回し Iq を記録する |
| V8 | 応答の遅れの分布（最大値）。`timeout_s` の既定値（50 ms）の妥当性 | 0.3 ms で応答 | 1000 回送って往復時間の分布を記録する |
| V9 | 電源断から復帰したあと、ドライバはどの状態から始まるか（故障が残るか、`0x91` が要るか） | — | 停止中に電源を切って入れ直し、`0xB2` と `0xB1` を読む |
| V10 | 故障中に制御指令を送ったとき、制御応答の RES は何になるか | FAILURE（動かない） | 故障が起きたときに記録する |
| V11 | `0x92` で停止したあと、出力軸は手で回るか（逆駆動できるか） | — | 停止中に出力軸を手で回し、`Ind.OUTPUT_ANGLE` の変化を記録する |

ほかに、トルク指令（`0x93`）の値が出力軸の N·m か、`ZERO_POSITION` が単回転か多回転か、`CAN_BITRATE` の 5 以外の値の意味も未確認。

---

## 10. 擬似ドライバ（実機なしでの開発・テスト）

`gds34can.fake` は、python-can の `virtual` インタフェースの上で GDS34 のふりをして応答する。実機なしで、Windows でも動く。
テスト・examples・上位のソフトウェアの開発に使う。

```python
from gds34can import Fault
from gds34can.fake import FakeMotor, FakeNetwork

with FakeNetwork([FakeMotor(1), FakeMotor(2, gear_ratio=36)]) as net, net.open_bus() as bus:
    print(bus.scan(range(1, 8)))              # [Found(motor_id=1, ...), Found(motor_id=2, ...)]
    m = bus.motor(1)
    with m.enabled():
        m.set_velocity(1.0)
    net.motors[0].inject_fault(Fault.OVER_CURRENT)   # 故障を起こす
```

| クラス | 引数 | 内容 |
|---|---|---|
| `FakeNetwork` | `motors=None`（既定は `[FakeMotor(1)]`）, `channel="gds34can-fake"`, `latency_s=0.0003` | 擬似ドライバをつないだ仮想バス。応答はスレッドで返す。`with` で開始・停止。`open_bus(**kwargs)` でつながる `Bus` を作る |
| `FakeMotor` | `motor_id=1`, `host_id=0`, `gear_ratio=10`, `version=0x21`, `host_id_follows_conf=True` | 擬似ドライバ 1 台。`inject_fault(fault)` で故障を起こす |

`FakeMotor` の属性: `conf`（設定項目の辞書）、`enabled`、`fault`、`position`（出力軸 rad、多回転）、`velocity`、
`received`（受け取った要求のリスト。テストでの確認用）。

- 同じプロセスで複数の `FakeNetwork` を同時に使うときは、`channel` をそれぞれ別の名前にする（virtual は同じ名前のチャネルどうしがつながる）
- 設定の初期値は、実機で読み出した値（[5.6 の `Conf` の表](#conf設定項目)）。実機で読んでいない項目（弱め界磁ゲイン・電流検出抵抗・増幅ゲイン）は `UNKNOWN_REGISTER` を返す
- 実機で確かめていない挙動は [9](#9-未確認の挙動) の仮定で動く。`host_id_follows_conf=False` にすると V1 の逆（応答の送信元が変わらない）になる
- 電流・電圧のインジケータは簡単なモデルの値で、実機の値ではない。回転中の Iq は ±0.14 A（実機の無負荷の平均の目安）、
  Vq は 0.011 V/RPM × 速度 + Rs × Iq、バス電圧は実機と同じ 2.552772 V で固定
- `0xC1` には応答しない。PID（`0xA1` / `0xA2`）などの未対応のコマンドには `UNKNOWN_COMMAND` を返す

---

## 11. CLI

読み出しと停止だけ（モータを動かす操作は examples にある）。

```sh
python -m gds34can [--iface IFACE] [--fake] scan [--first 0x01] [--last 0x7F]
python -m gds34can [--iface IFACE] [--fake] info  --id ID [--host-id 0]    # 版・故障・温度・角度・全設定
python -m gds34can [--iface IFACE] [--fake] fault --id ID [--host-id 0] [--clear]
python -m gds34can [--iface IFACE] [--fake] stop  --id ID [--host-id 0]    # 0x97 → 0x92
```

- `--iface` の既定は `hatcan0`。`--fake` で擬似ドライバ（ID 1）を相手にする
- 終了コード: 成功 0。`scan` で何も見つからない、`stop` で失敗した、`Gds34Error` が起きた場合は 1

---

## 12. examples

すべて `--fake` で擬似ドライバを相手にできる。実機では `--iface`（既定 `hatcan0`）でインタフェースを指定する。
引数の一覧は `python examples/0X_....py --help`。

| ファイル | 内容 | モータ | 主な引数 |
|---|---|---|---|
| `01_scan_and_info.py` | ドライバを探し、版・故障・主な設定・角度・温度を読む | 動かない | — |
| `02_spin_velocity.py` | 一定速度で回す（速度指令を周期的に送り直す。Duration は周期の 3 倍） | 動く | `--id 1 --rad-s 1.047 --seconds 5 --period 0.1 --max-rad-s 3.14` |
| `03_position_step.py` | 位置指令を 1 回送り、角度の変化を記録する（V3・V4 の確認用）。**`--confirm` がなければ何もしない**。移動量は 0.5 rad まで、今の角度が ±2π を超えていたら中止 | 動く | `--delta 0.2 --duration 1.0 --observe 2.0 --confirm` |
| `04_current_sweep.py` | 速度を段階的に変えて回し、Iq・Vq・速度を CSV に記録する。`transact_many` の例。\|rpm\| は 30 まで | 動く | `--rpm 5 10 -10 --seconds 5 --vbus 13.8 --csv sweep.csv` |
| `05_cycle_multi_motor.py` | 複数台に 100 Hz で速度指令（正弦波）を送る周期ループ。送受信時間と周期のずれを表示する | 動く | `--ids 1 2 --hz 100 --seconds 3 --amp 0.5` |
| `06_frame_log.py` | 生フレームとイベントを記録するフックの使い方（存在しない ID にも送り、タイムアウトのイベントを見せる） | 動かない | `--log frames.log` |
| `07_change_id.py` | CAN ID・ホスト ID の書き換え（V1・V2 の確認にも使う）。**`--execute` がなければ計画を表示するだけ** | 動かない | `--id 1 --new-id 4 --execute` / `--id 1 --new-host-id 0x101 --execute` |

擬似ドライバでの参考値（Windows、`05_cycle_multi_motor.py --fake --ids 1 2 3 4`）: 周期の中央値 10.04 ms（目標 10 ms）、
送受信時間は 1 台あたり 0.63〜0.65 ms（2 回の実行）。**擬似ドライバの値で、実機の値ではない**。

---

## 13. テスト

```sh
pip install -e ".[test]"
python -m pytest
```

実機は要らない（Windows でも動く）。30 件。

| ファイル | 内容 |
|---|---|
| `tests/test_codec.py`（17 件） | フレームの並び・Duration の範囲・制御応答の復号（実機のログの値と照合）・位置の折り返し・照合キー・結果コードの位置・故障フラグ・単位換算・設定項目の表の重複 |
| `tests/test_bus.py`（13 件） | 擬似ドライバとの結合: 探索・速度指令と `with` での停止（例外でも）・上限と送信禁止・タイムアウトからの復帰・`transact_many` の失敗の扱い・故障中の拒否と解除・`write_conf` の保護と読み戻し・ID の書き換え・2 スレッドから同時に使っても応答を取り違えないこと |

---

## 14. 構成

```
gds34can/
  __init__.py    公開 API
  constants.py   コマンド・結果コード・故障・インジケータ・設定項目の表（Conf は単位と保護の種類つき）
  codec.py       フレームの符号化・復号（通信しない純粋な関数）
  bus.py         Bus: 1 本の CAN バスの送受信（排他・タイムアウト・照合・探索・ID の書き換え）
  motor.py       Motor: ドライバ 1 台の操作（SI 単位）
  errors.py      例外
  fake.py        擬似ドライバ
  __main__.py    CLI
examples/        使用例（_common.py は共通の引数と、実機 / 擬似ドライバの切り替え）
tests/           テスト
```

---

## 15. 未実装・今後

| 項目 | 理由・条件 |
|---|---|
| 実行時 PID の読み書き（`0xA1` / `0xA2`） | ParaID の一覧を資料から整理できていない |
| ボーレートの書き換え | OS 側のインタフェースも設定し直す必要がある。当面 500kbps のまま使う |
| `transact_many` で要求をまとめて送る | 今は 1 つずつ順に送る。V1 で応答の送信元を台ごとに区別できると分かれば、まとめて送れる（呼び出し側は変えなくてよい） |
| 実機での動作確認 | 読み出し（`scan` / `info`）→ 2026-10-24 の電流の計測を `04_current_sweep.py` でやり直す → V1〜V11 の順に行う |

### 変更履歴

- **0.1.0**（2026-11-02）: 最初の版。擬似ドライバとテストで確認。実機では未確認

### ライセンス

未設定。

---

## 16. 出典

- SteadyWin "Motor Driver Protocol Specification" rev2.2（2023-08-30。EN_GIM_Motor_Driver_Protocol_Specification_rev2.2.pdf）— プロトコルの主資料
- SteadyWin MIT プロトコル手册 rev1.4 — MIT モードの見分け方
- SteadyWin "GIM Motor Driver App User Manual (Windows)" rev1.6 — 設定項目の意味・注意（Motor Wizard）
- GDS34 ドライバ図面（SDC101）
- 資料の入手元: https://www.steadywin.cn/en/h-col-124.html
