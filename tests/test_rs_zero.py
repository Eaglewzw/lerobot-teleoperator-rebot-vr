import math
from types import SimpleNamespace

import pytest

from lerobot_teleoperator_rebot_vr.constants import JOINT_NAMES
from lerobot_teleoperator_rebot_vr.tools import rs_zero


class Connection:
    def __init__(self, *, disable_error=None, zero_error=None, moving=False):
        self.positions = {name: .2 for name in JOINT_NAMES}
        self.events = []
        self.motors = {}
        self.moving = moving
        self.reads = 0
        for name in JOINT_NAMES:
            def disable(name=name):
                self.events.append(("disable", name))
                if name == disable_error:
                    raise RuntimeError("disable failed")
            def zero(name=name):
                self.events.append(("zero", name))
                if name == zero_error:
                    raise RuntimeError("zero failed")
                self.positions[name] = 0.
            self.motors[name] = SimpleNamespace(disable=disable, set_zero_position=zero)

    def read_positions(self, timeout):
        self.reads += 1
        if self.moving and self.reads == 2:
            self.positions[JOINT_NAMES[1]] += math.radians(2)
        return {name: (pos, 1, 2) for name, pos in self.positions.items()}

    def read_position(self, name, timeout):
        return self.positions[name], 1, 2


@pytest.fixture(autouse=True)
def no_delay(monkeypatch):
    monkeypatch.setattr(rs_zero.time, "sleep", lambda _: None)


def test_preview_is_read_only():
    connection = Connection()
    rows = []
    rs_zero.calibrate(connection, rows.append)
    assert not connection.events
    assert rows[0]["event"] == "before"


@pytest.mark.parametrize("gripper", [False, True])
def test_zero_disables_all_before_writes_and_preserves_unselected_gripper(gripper):
    connection = Connection()
    rows = []
    rs_zero.calibrate(connection, rows.append, execute=True, include_gripper=gripper)
    assert connection.events[:7] == [("disable", name) for name in JOINT_NAMES]
    assert connection.events[7:] == [("zero", name) for name in (JOINT_NAMES if gripper else JOINT_NAMES[:6])]
    assert connection.positions["gripper"] == (0. if gripper else .2)
    assert rows[-1]["event"] == "complete"


@pytest.mark.parametrize("kwargs", [{"disable_error": JOINT_NAMES[2]}, {"moving": True}])
def test_disable_failure_or_moving_pose_prevents_all_zero_writes(kwargs):
    connection = Connection(**kwargs)
    with pytest.raises(RuntimeError):
        rs_zero.calibrate(connection, lambda _: None, execute=True)
    assert connection.events == [("disable", name) for name in JOINT_NAMES]


def test_partial_zero_failure_aborts_later_axes_and_records_prior_writes():
    connection = Connection(zero_error=JOINT_NAMES[2])
    rows = []
    with pytest.raises(RuntimeError, match="zero failed"):
        rs_zero.calibrate(connection, rows.append, execute=True)
    assert [name for event, name in connection.events if event == "zero"] == list(JOINT_NAMES[:3])
    assert [r["joint"] for r in rows if r["event"] == "zero_acknowledged"] == list(JOINT_NAMES[:2])
    assert not any(r["event"] == "complete" for r in rows)


def test_bad_readback_aborts_remaining_writes(monkeypatch):
    connection = Connection()
    monkeypatch.setattr(connection, "read_position", lambda *args: (.1, 1, 2))
    with pytest.raises(RuntimeError, match="readback"):
        rs_zero.calibrate(connection, lambda _: None, execute=True)
    assert connection.events[7:] == [("zero", JOINT_NAMES[0])]


def test_cancelled_confirmation_never_opens_transport(monkeypatch):
    monkeypatch.setattr("sys.argv", ["rs_zero", "--execute"])
    monkeypatch.setattr("builtins.input", lambda _: "no")
    monkeypatch.setattr(rs_zero, "RSConnection", lambda *a: pytest.fail("transport opened"))
    with pytest.raises(SystemExit, match="Cancelled"):
        rs_zero.main()
