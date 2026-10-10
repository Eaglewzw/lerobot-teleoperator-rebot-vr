import json

import pytest

from lerobot_teleoperator_rebot_vr.tools import rs_diagnostics as diagnostic


def test_open_failure_is_not_hidden_by_empty_bus_cleanup_error():
    events = []
    class BrokenBus:
        def add_robstride_motor(self, *args):
            raise PermissionError("CAN access denied")
        def close_bus(self):
            raise RuntimeError("no motor")
        def close(self):
            events.append("closed")
    connection = diagnostic.RSConnection(controller_factory=lambda _: BrokenBus())
    with pytest.raises(PermissionError, match="CAN access denied") as caught:
        connection.open()
    assert "no motor" in caught.value.__notes__[0]
    assert connection.bus is None and events == ["closed"]


class ReadOnlyMotor:
    def robstride_get_param_f32_host_id(self, register, host, timeout_ms):
        assert host == 0xFD and timeout_ms == 100
        assert register in dict(diagnostic.FLOAT_REGISTERS)
        return .01 if register == 0x7019 else 1.

    def robstride_get_param_i8(self, register, timeout_ms):
        assert register == 0x7005 and timeout_ms == 100
        return 0


def test_parameters_use_typed_reads_and_no_control_methods():
    row = diagnostic.read_motor_diagnostics(ReadOnlyMotor(), port="can1",
        joint="wrist_flex", sample=0, timeout_ms=100)
    assert row["motor_id"] == 4 and row["run_mode_raw"] == 0
    assert row["position_deg"] == pytest.approx(.572957795)
    assert row["errors"] == {}
    assert row["read_finished_ns"] >= row["read_started_ns"]


@pytest.mark.parametrize("value", [float("nan"), RuntimeError("unsupported")])
def test_unavailable_register_is_an_error_not_a_zero(value):
    motor = ReadOnlyMotor()
    def read(register, host, timeout):
        if isinstance(value, Exception):
            raise value
        return value
    motor.robstride_get_param_f32_host_id = read
    row = diagnostic.read_motor_diagnostics(motor, port="can1", joint="elbow_flex",
        sample=0, timeout_ms=100)
    assert "iq_filtered_a" not in row
    assert "iq_filtered_a" in row["errors"]
    assert row["run_mode_raw"] == 0


def test_cli_isolates_ports_records_failures_and_always_closes(monkeypatch, tmp_path):
    events = []
    class Connection:
        def __init__(self, port):
            self.port = port
            self.motors = {"elbow_flex": ReadOnlyMotor(), "wrist_flex": ReadOnlyMotor()}
        def open(self):
            events.append((self.port, "open"))
            if self.port == "can0":
                raise RuntimeError("unavailable")
        def close(self):
            events.append((self.port, "close"))
    output = tmp_path / "diagnostics.jsonl"
    monkeypatch.setattr(diagnostic, "RSConnection", Connection)
    monkeypatch.setattr("sys.argv", ["rs_diagnostics", "--samples", "1", "--output", str(output)])
    with pytest.raises(SystemExit) as result:
        diagnostic.main()
    assert result.value.code == 1
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert rows[0] == {"port": "can0", "error": "unavailable"}
    assert {row["joint"] for row in rows[1:]} == {"elbow_flex", "wrist_flex"}
    assert events == [("can0", "open"), ("can0", "close"), ("can1", "open"), ("can1", "close")]
