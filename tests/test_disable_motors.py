from collections import Counter

import pytest

from lerobot_teleoperator_rebot_vr.tools import disable_motors as tool


def feedback(motor_id, status, feedback_id=None):
    return (
        b"\xaa\x11\x08"
        + (feedback_id if feedback_id is not None else 0x10 + motor_id).to_bytes(4, "little")
        + bytes([(status << 4) | motor_id]) + b"\x80\x00\x80\x08\x00\x20\x20\x55"
    )


class FakePort:
    def __init__(self):
        self.writes = []
        self.buffer = bytearray()

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, frame):
        self.writes.append(frame)
        can_id = int.from_bytes(frame[13:17], "little")
        motor_id = can_id & 0xFF
        enabled = motor_id >= 4 and can_id < 0x100
        self.buffer.extend(feedback(motor_id, int(enabled)))
        return len(frame)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, count):
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data


def test_mode_probe_disables_mixed_firmware_without_enable_or_motion(monkeypatch, capsys):
    monkeypatch.setattr(tool.time, "sleep", lambda delay: None)
    port = FakePort()
    targets = [
        tool.DisableTarget(name, i, 0x10 + i, "force_pos" if i == 7 else "pos_vel")
        for i, name in enumerate(tool.JOINTS, 1)
    ]
    assert tool.probe(port, targets, timeout_s=0.001)
    counts = Counter(int.from_bytes(frame[13:17], "little") for frame in port.writes)
    assert counts == {1: 1, 2: 1, 3: 1, 4: 2, 5: 2, 6: 2, 7: 2, 0x104: 1, 0x105: 1, 0x106: 1, 0x307: 1}
    for frame in port.writes:
        assert len(frame) == 30
        assert frame[:4] == bytes.fromhex("55 AA 1E 03")
        assert frame[12] == 0 and frame[18] == 8
        assert frame[21:29] == bytes.fromhex("FF FF FF FF FF FF FF FD")
    assert "mode_can_id=0x105 attempt=1/2 raw_status=DISABLED" in capsys.readouterr().out


def test_fragmented_and_corrupted_feedback_does_not_cross_motor_ids():
    buffer = bytearray(b"noise\xaa\x11\x00" + feedback(5, 0)[:10])
    assert tool.pop_feedback(buffer) == []
    buffer.extend(feedback(5, 0)[10:] + feedback(6, 1))
    assert tool.pop_feedback(buffer) == [(0x15, 5, 0), (0x16, 6, 1)]


def test_old_cache_and_other_motor_reply_cannot_confirm_disable(monkeypatch):
    port = FakePort()
    port.buffer.extend(feedback(5, 0))

    def wrong_reply(frame):
        port.buffer.extend(feedback(6, 0) + feedback(5, 0, feedback_id=0x16))
        return len(frame)

    monkeypatch.setattr(port, "write", wrong_reply)
    target = tool.DisableTarget("wrist_yaw", 5, 0x15, "pos_vel")
    assert tool.send_disable_and_read(port, target, 0x105, 0.002) is None


def test_register_reply_is_not_a_disabled_sensor_state():
    register_reply = b"\xaa\x11\x08\x15\x00\x00\x00\x05\x00\x33\x0a\x02\x00\x00\x00\x55"
    buffer = bytearray(register_reply + feedback(5, 1))
    assert tool.pop_feedback(buffer) == [(0x15, 5, 1)]


def test_one_motor_send_failure_does_not_skip_remaining_motors(monkeypatch):
    monkeypatch.setattr(tool.time, "sleep", lambda delay: None)
    port = FakePort()
    original_write = port.write

    def fail_first_motor(frame):
        if int.from_bytes(frame[13:17], "little") & 0xFF == 1:
            raise OSError("injected send failure")
        return original_write(frame)

    monkeypatch.setattr(port, "write", fail_first_motor)
    targets = [tool.DisableTarget("q1", 1, 0x11, "mit"), tool.DisableTarget("q2", 2, 0x12, "mit")]
    assert not tool.probe(port, targets, timeout_s=0.001)
    assert len(port.writes) == 1
    assert int.from_bytes(port.writes[0][13:17], "little") == 2


def test_default_cli_only_prints_plan(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["disable_motors", "--robot-port", "/must/not/open"])
    tool.main()
    assert "Plan only" in capsys.readouterr().out


@pytest.mark.parametrize("can_id", [-1, 0, 0x800])
def test_reject_nonstandard_disable_address(can_id):
    with pytest.raises(ValueError):
        tool.disable_frame(can_id)
