from types import SimpleNamespace

import pytest

from lerobot_teleoperator_rebot_vr.tools import read_disabled_angles as probe


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def fake_controller(events, *, bad=None):
    # No enable/disable/zero/mode/control APIs exist on these fakes. Any
    # accidental attempt to use them, including via context manager, fails.
    class Motor:
        def __init__(self, mid, fid):
            self.mid, self.fid = mid, fid

        def request_feedback(self):
            events.append(("request", self.mid))

        def get_state(self):
            events.append(("state", self.mid))
            if bad == "missing" and self.mid == 3:
                return None
            if bad == "read_error" and self.mid == 3:
                raise RuntimeError("read failed")
            return SimpleNamespace(
                can_id=self.mid, arbitration_id=999 if bad == "ids" else self.fid,
                status_code=1 if bad == "enabled" and self.mid == 3 else 0,
                pos=float("nan") if bad == "nan" else -.1 * self.mid,
                vel=0., torq=0., t_mos=30., t_rotor=28.,
            )

        def close(self):
            events.append(("motor_close", self.mid))

    class Controller:
        def add_damiao_motor(self, mid, fid, model):
            events.append(("add", mid, fid, model))
            if bad == "add_error" and mid == 3:
                raise RuntimeError("add failed")
            return Motor(mid, fid)

        def poll_feedback_once(self):
            events.append(("poll",))

        def close_bus(self):
            events.append(("close_bus",))
            if bad == "close_error":
                raise RuntimeError("close failed")

        def close(self):
            events.append(("controller_close",))

    def open_controller():
        events.append(("open",))
        return Controller()

    return open_controller


def test_read_only_call_trace_and_explicit_unverified_freshness():
    events, rows, report = [], [], {}
    clock = Clock()
    probe.read_disabled_angles(fake_controller(events), rows.append, report,
                               duration_s=.5, clock=clock, sleep=clock.sleep)
    assert report["result"] == "read_complete_freshness_unverified"
    assert not report["torque_off_confirmed"]
    assert all(r["feedback_freshness"] == "unknown_no_receive_timestamp" for r in rows)
    assert set(rows[0]) == set(probe.FIELDS)
    assert report["joints"]["elbow_flex"]["last_deg"] == pytest.approx(-17.18873385)
    assert {e[0] for e in events} == {"open", "add", "poll", "request", "state", "close_bus", "motor_close", "controller_close"}
    assert events[-9] == ("close_bus",)
    assert events[-1] == ("controller_close",)


@pytest.mark.parametrize("bad", ["missing", "enabled", "ids", "nan", "read_error", "add_error", "close_error"])
def test_probe_fails_closed_and_releases_resources_without_control_commands(bad):
    events, rows, report = [], [], {}
    clock = Clock()
    with pytest.raises(RuntimeError):
        probe.read_disabled_angles(fake_controller(events, bad=bad), rows.append, report,
                                   duration_s=.5, clock=clock, sleep=clock.sleep)
    assert report["result"] == "failed"
    assert ("close_bus",) in events
    assert events[-1] == ("controller_close",)
    if bad == "enabled":
        assert rows[-1]["status_code"] == 1
    if bad == "missing":
        assert "elbow_flex" not in report["joints"]


@pytest.mark.parametrize("duration,rate", [(0, 10), (float("nan"), 10), (31, 10), (3, 0), (3, float("inf")), (3, 21)])
def test_invalid_limits_never_open_bus(duration, rate):
    events = []
    with pytest.raises(ValueError):
        probe.read_disabled_angles(fake_controller(events), lambda _: None, {},
                                   duration_s=duration, rate_hz=rate)
    assert not events


def test_keyboard_interrupt_still_closes_transport():
    events, report = [], {}

    def interrupted(_):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        probe.read_disabled_angles(fake_controller(events), interrupted, report)
    assert report["result"] == "failed"
    assert events[-1] == ("controller_close",)


def test_dry_run_and_support_guard_do_not_open_sdk(monkeypatch, capsys):
    import sys
    import json
    from pathlib import Path
    from lerobot_teleoperator_rebot_vr.runtime.dual_config import load_dual_config
    monkeypatch.setitem(sys.modules, "motorbridge", SimpleNamespace())
    base = ["--config", "config/dual_mit_split.yaml", "--arm", "right"]
    probe.main([*base, "--dry-run"])
    output = json.loads(capsys.readouterr().out)
    expected = load_dual_config(Path("config/dual_mit_split.yaml")).arms["right"]
    assert output["port"] == expected.robot_port
    assert output["robot_id"] == expected.robot_id
    with pytest.raises(SystemExit) as exc:
        probe.main(base)
    assert exc.value.code == 2
