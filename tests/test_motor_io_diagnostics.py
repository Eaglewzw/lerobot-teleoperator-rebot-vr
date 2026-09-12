import csv
import json
import math
from types import SimpleNamespace

import pytest

from lerobot_teleoperator_rebot_vr.diagnostics.motor_io import MotorIODiagnostics
from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser, validate_args


class Motor:
    def __init__(self):
        self.calls = []
        self.state = SimpleNamespace(can_id=4, arbitration_id=20, status_code=1,
                                     pos=0.1, vel=0.2, torq=0.3, t_mos=30, t_rotor=31)

    def send_pos_vel(self, *args):
        self.calls.append(('pos_vel', args))
        return 42

    def send_mit(self, *args):
        self.calls.append(('mit', args))

    def send_force_pos(self, *args):
        self.calls.append(('force_pos', args))

    def get_state(self):
        return self.state

    def request_feedback(self):
        raise OSError('serial disconnected')


def test_passthrough_events_errors_and_summary(tmp_path):
    original = Motor()
    robot = SimpleNamespace(motors={'wrist_flex': original},
                            config=SimpleNamespace(motor_can_ids={'wrist_flex': (4, 20)}))
    rec = MotorIODiagnostics(tmp_path/'run.csv', metadata={'mode': 'test'})
    rec.install(robot)
    rec.phase = 'active'
    rec.cycle_started_monotonic_ns = 123
    motor = robot.motors['wrist_flex']
    assert motor.send_pos_vel(.4, 2) == 42
    motor.send_mit(.5, 1, 30, 1, -.1)
    motor.send_force_pos(.6, 3, .2)
    assert motor.get_state() is original.state
    motor.get_state()
    original.state = None
    assert motor.get_state() is None
    with pytest.raises(OSError, match='serial disconnected'):
        motor.request_feedback()
    rec.restore()
    assert robot.motors['wrist_flex'] is original
    rec.close()
    rec.close()
    assert original.calls == [('pos_vel', (.4, 2)), ('mit', (.5, 1, 30, 1, -.1)), ('force_pos', (.6, 3, .2))]
    with rec.output_path.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 7
    assert rows[0]['position_rad'] == '0.4'
    assert rows[1]['kp'] == '30'
    assert rows[2]['force_ratio'] == '0.2'
    assert rows[3]['actual_torque_nm'] == '0.3'
    assert rows[3]['feedback_freshness_available'] == 'False'
    assert rows[5]['state_available'] == 'False'
    assert rows[-1]['ok'] == 'False'
    assert rows[-1]['error'] == 'OSError: serial disconnected'
    assert all(r['cycle_started_monotonic_ns'] == '123' for r in rows)
    report = json.loads(rec.summary_path.read_text())
    stats = report['statistics']['wrist_flex/active']
    assert stats['errors'] == 1
    assert stats['missing_state'] == 1
    assert stats['status_codes'] == {'1': 2}
    assert stats['max_unchanged_position_read_s'] > 0
    assert report['dropped_events'] == 0


def test_diagnostics_requires_csv():
    args = build_parser().parse_args(['--motor-diagnostics'])
    with pytest.raises(ValueError, match='requires --csv-log'):
        validate_args(args)
    args = build_parser().parse_args(['--motor-diagnostics', '--csv-log', '/tmp/test.csv'])
    validate_args(args)


def test_restore_does_not_reintroduce_disconnected_motors(tmp_path):
    robot = SimpleNamespace(motors={'wrist_flex': Motor()},
                            config=SimpleNamespace(motor_can_ids={'wrist_flex': (4, 20)}))
    rec = MotorIODiagnostics(tmp_path/'closed.csv', metadata={})
    rec.install(robot)
    robot.motors = {}  # The follower clears this mapping after disconnect.
    rec.restore()
    rec.close()
    assert robot.motors == {}


def test_writer_failure_is_reported_without_shutdown_deadlock(tmp_path):
    rec = MotorIODiagnostics(tmp_path/'run.csv', metadata={})
    rec.record({'unexpected_column': 'bad'})
    rec._thread.join(timeout=2)
    assert not rec._thread.is_alive()
    with pytest.raises(RuntimeError, match='writer failed'):
        rec.close()


def test_real_follower_logs_after_relative_clipping(tmp_path):
    from lerobot.robots.rebot_b601_follower import RebotB601Follower

    names = ['wrist_flex', 'wrist_yaw']
    motors = {name: Motor() for name in names}
    robot = SimpleNamespace(
        is_connected=True, motors=motors, motor_names=names,
        _present_pos=lambda: dict.fromkeys(names, 0.0),
        config=SimpleNamespace(
            motor_can_ids={'wrist_flex': (4, 20), 'wrist_yaw': (5, 21)},
            joint_limits=dict.fromkeys(names, (-90, 90)),
            max_relative_target=2.0, control_mode='pos_vel', pos_vel_velocity=[100, 100],
        ),
    )
    rec = MotorIODiagnostics(tmp_path/'clipped.csv', metadata={})
    rec.install(robot)
    try:
        result = RebotB601Follower.send_action(robot, {'wrist_flex.pos': 30, 'wrist_yaw.pos': 0})
        assert result['wrist_flex.pos'] == 2
    finally:
        rec.restore()
        rec.close()
    with rec.output_path.open() as f:
        rows = list(csv.DictReader(f))
    assert float(rows[0]['position_rad']) == pytest.approx(math.radians(2))
    assert float(rows[0]['velocity_rad_s']) == pytest.approx(math.radians(100))
