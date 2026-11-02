"""Bus / Motor の結合テスト（擬似ドライバ + python-can virtual。実機なしで動く）。"""
import itertools
import threading
import time

import pytest

from gds34can import (BlockedCommand, Command, CommandRejected, Conf, Fault, FaultActive, GuardedSetting, Ind,
                      LimitExceeded, NoReply, Request, StaleIndicatorWarning)
from gds34can.fake import FakeMotor, FakeNetwork

_channels = itertools.count()


@pytest.fixture
def net_factory():
    nets = []

    def make(*motors):
        net = FakeNetwork(motors=list(motors) or [FakeMotor(1)], channel=f"test-{next(_channels)}").start()
        nets.append(net)
        return net
    yield make
    for n in nets:
        n.stop()


def test_scan_and_params(net_factory):
    net = net_factory(FakeMotor(1), FakeMotor(3, gear_ratio=36))
    with net.open_bus() as bus:
        found = bus.scan(range(1, 8))
        assert [(f.motor_id, f.host_id, f.version) for f in found] == [(1, 0, 0x21), (3, 0, 0x21)]
        m = bus.motor(3)
        assert m.gear_ratio == 36 and m.kt == pytest.approx(0.08472)


def test_velocity_moves_and_with_block_stops(net_factory):
    fake = FakeMotor(1)
    net = net_factory(fake)
    with net.open_bus() as bus:
        m = bus.motor(1)
        with m.enabled():
            for _ in range(5):
                s = m.set_velocity(1.0, duration_s=0.3)
                time.sleep(0.05)
            assert s.velocity_rad_s == pytest.approx(1.0, abs=0.04)
            assert m.read(Ind.OUTPUT_ANGLE) > 0.15
        assert not fake.enabled
        assert fake.received[-2][0] == 0x97 and fake.received[-1][0] == 0x92


def test_with_block_stops_on_exception(net_factory):
    fake = FakeMotor(1)
    net = net_factory(fake)
    with net.open_bus() as bus:
        m = bus.motor(1)
        with pytest.raises(RuntimeError):
            with m.enabled():
                m.set_velocity(1.0)
                raise RuntimeError("boom")
        assert not fake.enabled


def test_limits_and_blocked_commands(net_factory):
    net = net_factory()
    with net.open_bus() as bus:
        m = bus.motor(1, max_velocity_rad_s=1.0)
        with pytest.raises(LimitExceeded):
            m.cmd_velocity(1.5)
        for cmd in (0x81, 0xB5, 0xC1):
            with pytest.raises(BlockedCommand):
                bus.transact(Command(Request(1, bytes([cmd]) + bytes(7), 0, "raw"), lambda r: r))


def test_no_reply_then_recovers(net_factory):
    net = net_factory()
    events = []
    with net.open_bus(on_event=events.append) as bus:
        with pytest.raises(NoReply):
            bus.motor(0x10, read_params=False).read_version()
        assert any(e.kind == "timeout" for e in events)
        assert bus.motor(1).read_version() == 0x21


def test_transact_many_returns_errors_in_place(net_factory):
    net = net_factory()
    with net.open_bus() as bus:
        m1, ghost = bus.motor(1), bus.motor(0x10, read_params=False)
        results = bus.transact_many([m1.cmd_version(), ghost.cmd_version(), m1.cmd_read(Ind.IQ)])
        assert results[0] == 0x21 and isinstance(results[1], NoReply) and results[2] == 0.0


def test_fault_rejects_control_until_cleared(net_factory):
    fake = FakeMotor(1)
    net = net_factory(fake)
    with net.open_bus() as bus:
        m = bus.motor(1)
        fake.inject_fault(Fault.OVER_TEMPERATURE)
        assert m.read_fault() == Fault.OVER_TEMPERATURE
        with pytest.raises(FaultActive):
            m.ensure_no_fault()
        with pytest.raises(CommandRejected):
            m.start()
        m.clear_fault()
        m.ensure_no_fault()


def test_write_conf_guards_and_read_back(net_factory):
    net = net_factory()
    with net.open_bus() as bus:
        m = bus.motor(1)
        with pytest.raises(GuardedSetting):
            m.write_conf(Conf.CAN_ID, 5)
        with pytest.raises(GuardedSetting):
            m.write_conf(Conf.GEAR_RATIO, 36)
        with pytest.raises(GuardedSetting):
            m.write_conf(Conf.POWER_OFF_POSITION, 0)
        assert m.write_conf(Conf.MAX_CURRENT, 2) == 8      # 書く前の値が返る
        assert m.read_conf(Conf.MAX_CURRENT) == 2
        m.write_conf(Conf.GEAR_RATIO, 36, allow_motor_model=True)
        assert m.gear_ratio == 36                           # Kt・減速比は読み直される


def test_unread_conf_items_come_back_as_errors(net_factory):
    net = net_factory()
    with net.open_bus() as bus:
        conf = bus.motor(1).read_all_conf()
        assert conf[Conf.PWM_FREQUENCY] == 30000
        assert isinstance(conf[Conf.AMP_GAIN], CommandRejected)   # 擬似ドライバは実機で読んでいない項目を返さない


def test_stale_indicator_warns(net_factory):
    net = net_factory()
    with net.open_bus() as bus:
        with pytest.warns(StaleIndicatorWarning):
            bus.motor(1).read(Ind.BUS_VOLTAGE)


def test_change_id_requires_single_driver(net_factory):
    net = net_factory(FakeMotor(1), FakeMotor(2))
    with net.open_bus() as bus:
        with pytest.raises(GuardedSetting):
            bus.change_id(1, 4, scan_ids=range(1, 8))


def test_change_id_and_host_id(net_factory):
    fake = FakeMotor(1)
    net = net_factory(fake)
    with net.open_bus() as bus:
        after = bus.change_id(1, 4, scan_ids=range(1, 8))
        assert after.motor_id == 4 and fake.id == 4
        after = bus.change_host_id(4, 0x104, scan_ids=range(1, 8))
        assert after.host_id == 0x104
        m = bus.motor(4, host_id=0x104)
        assert m.read_conf(Conf.HOST_CAN_ID) == 0x104


def test_concurrent_threads_do_not_mix_replies(net_factory):
    # 全軸が同じ送信元（0x000）から応答する状況で、2 スレッドが同時に使っても取り違えないこと
    net = net_factory(FakeMotor(1), FakeMotor(2))
    with net.open_bus() as bus:
        m1, m2 = bus.motor(1), bus.motor(2)
        bad = []

        def worker(m):
            for _ in range(50):
                if m.read_conf(Conf.CAN_ID) != m.id:
                    bad.append(m.id)
        threads = [threading.Thread(target=worker, args=(m,)) for m in (m1, m2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert bad == []
