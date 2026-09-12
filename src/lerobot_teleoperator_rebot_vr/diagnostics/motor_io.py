"""Observe existing motor API calls, without sending extra hardware requests."""

from __future__ import annotations

import csv
import json
import math
import queue
import threading
import time
from collections import Counter
from pathlib import Path


FIELDS = (
    'started_monotonic_ns', 'finished_monotonic_ns', 'cycle_started_monotonic_ns',
    'phase', 'joint', 'configured_send_id', 'configured_receive_id', 'operation',
    'ok', 'error', 'position_rad', 'velocity_rad_s', 'kp', 'kd', 'feedforward_nm',
    'force_ratio', 'state_available', 'can_id', 'arbitration_id', 'status_code',
    'actual_position_rad', 'actual_velocity_rad_s', 'actual_torque_nm',
    'mos_temperature', 'rotor_temperature', 'feedback_freshness_available',
)


class MotorIODiagnostics:
    """Bounded, asynchronous event log and summary; get_state is a cache read."""

    def __init__(self, base_path: Path, *, metadata: dict, queue_size: int = 16384):
        base_path = Path(base_path)
        self.output_path = base_path.with_name(base_path.stem + '_motor_io.csv')
        self.summary_path = base_path.with_name(base_path.stem + '_motor_summary.json')
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open('w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._queue = queue.Queue(maxsize=queue_size)
        self._error = None
        self._closed = False
        self.dropped_events = 0
        self.phase = 'startup'
        self.cycle_started_monotonic_ns = 0
        self.metadata = metadata
        self._originals = {}
        self._robot = None
        self._thread = threading.Thread(target=self._run, name='rebot-motor-io', daemon=True)
        self._thread.start()

    def install(self, robot):
        self._robot = robot
        for name, motor in list(robot.motors.items()):
            self._originals[name] = motor
            robot.motors[name] = _ObservedMotor(motor, name, self, robot.config.motor_can_ids[name])

    def restore(self):
        if self._robot is not None:
            for name, original in self._originals.items():
                current = self._robot.motors.get(name)
                if isinstance(current, _ObservedMotor) and current._recorder is self:
                    self._robot.motors[name] = original
            self._robot = None

    def record(self, row):
        if self._closed or self._error is not None:
            self.dropped_events += 1
            return
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self.dropped_events += 1

    def close(self):
        if self._closed:
            return
        self._closed = True
        # A failed writer must never strand shutdown on a full queue.
        while self._thread.is_alive():
            try:
                self._queue.put(None, timeout=0.05)
                break
            except queue.Full:
                pass
        self._thread.join()
        if self._error is not None:
            raise RuntimeError(f'Motor diagnostics writer failed: {self._error}') from self._error

    def _run(self):
        stats = {}
        previous = {}
        try:
            while True:
                row = self._queue.get()
                if row is None:
                    break
                self._writer.writerow(row)
                key = row['joint'] + '/' + row['phase']
                s = stats.setdefault(key, {
                    'operations': Counter(), 'errors': 0, 'missing_state': 0,
                    'status_codes': Counter(), 'position_changes': 0,
                    'max_unchanged_position_read_s': 0.0,
                    'max_api_call_ms': 0.0, 'max_abs_torque_nm': 0.0,
                    'total_api_call_ms': 0.0, 'max_abs_velocity_rad_s': 0.0,
                    'max_mos_temperature': None, 'max_rotor_temperature': None,
                })
                s['operations'][row['operation']] += 1
                s['errors'] += int(not row['ok'])
                s['max_api_call_ms'] = max(s['max_api_call_ms'],
                    (row['finished_monotonic_ns'] - row['started_monotonic_ns']) / 1e6)
                s['total_api_call_ms'] += (row['finished_monotonic_ns'] - row['started_monotonic_ns']) / 1e6
                if row['operation'] == 'get_state' and row['ok']:
                    if not row['state_available']:
                        s['missing_state'] += 1
                        previous.pop(key, None)
                    else:
                        s['status_codes'][str(row['status_code'])] += 1
                        torque = row.get('actual_torque_nm')
                        if isinstance(torque, (int, float)) and math.isfinite(torque):
                            s['max_abs_torque_nm'] = max(s['max_abs_torque_nm'], abs(torque))
                        velocity = row.get('actual_velocity_rad_s')
                        if isinstance(velocity, (int, float)) and math.isfinite(velocity):
                            s['max_abs_velocity_rad_s'] = max(s['max_abs_velocity_rad_s'], abs(velocity))
                        for name in ('mos_temperature', 'rotor_temperature'):
                            value = row.get(name)
                            if isinstance(value, (int, float)) and math.isfinite(value):
                                old_max = s['max_' + name]
                                s['max_' + name] = value if old_max is None else max(old_max, value)
                        q = row['actual_position_rad']
                        now = row['finished_monotonic_ns']
                        old = previous.get(key)
                        if old is None or old[0] != q:
                            s['position_changes'] += int(old is not None)
                            previous[key] = (q, now)
                        else:
                            s['max_unchanged_position_read_s'] = max(
                                s['max_unchanged_position_read_s'], (now-old[1])/1e9)
                elif row['operation'] == 'get_state':
                    previous.pop(key, None)
                # Flush when caught up; file I/O never runs in the control loop.
                if self._queue.empty():
                    self._file.flush()
            for s in stats.values():
                s['mean_api_call_ms'] = s['total_api_call_ms'] / sum(s['operations'].values())
            report = {
                'schema_version': 1, 'metadata': self.metadata,
                'dropped_events': self.dropped_events,
                'feedback_freshness_available': False,
                'notes': ['API success is not motor acknowledgement.',
                          'get_state time is cache-read time, not CAN reception time.',
                          'Unchanged position does not prove missing feedback.',
                          'Statistics cover retained events, grouped by joint/phase.'],
                'statistics': stats,
            }
            self.summary_path.write_text(json.dumps(report, indent=2, default=str) + '\n', encoding='utf-8')
        except BaseException as exc:
            self._error = exc
        finally:
            self._file.close()


class _ObservedMotor:
    def __init__(self, motor, name, recorder, ids):
        self._motor, self._name, self._recorder, self._ids = motor, name, recorder, ids

    def __getattr__(self, name):
        return getattr(self._motor, name)

    def _call(self, operation, args, values):
        rec = self._recorder
        row = dict(values, started_monotonic_ns=time.monotonic_ns(),
                   cycle_started_monotonic_ns=rec.cycle_started_monotonic_ns,
                   phase=rec.phase, joint=self._name, configured_send_id=self._ids[0],
                   configured_receive_id=self._ids[1], operation=operation,
                   ok=False, error='', feedback_freshness_available=False)
        try:
            result = getattr(self._motor, operation)(*args)
        except Exception as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
            raise
        else:
            row['ok'] = True
            if operation == 'get_state':
                row['state_available'] = result is not None
                if result is not None:
                    for field, attr in (
                        ('can_id', 'can_id'), ('arbitration_id', 'arbitration_id'),
                        ('status_code', 'status_code'), ('actual_position_rad', 'pos'),
                        ('actual_velocity_rad_s', 'vel'), ('actual_torque_nm', 'torq'),
                        ('mos_temperature', 't_mos'), ('rotor_temperature', 't_rotor'),
                    ):
                        row[field] = getattr(result, attr, '')
            return result
        finally:
            row['finished_monotonic_ns'] = time.monotonic_ns()
            rec.record(row)

    def send_mit(self, pos, vel, kp, kd, tau):
        return self._call('send_mit', (pos, vel, kp, kd, tau),
                          dict(position_rad=pos, velocity_rad_s=vel, kp=kp, kd=kd, feedforward_nm=tau))

    def send_pos_vel(self, pos, vlim):
        return self._call('send_pos_vel', (pos, vlim), dict(position_rad=pos, velocity_rad_s=vlim))

    def send_force_pos(self, pos, vlim, ratio):
        return self._call('send_force_pos', (pos, vlim, ratio),
                          dict(position_rad=pos, velocity_rad_s=vlim, force_ratio=ratio))

    def get_state(self):
        return self._call('get_state', (), {})

    def request_feedback(self):
        return self._call('request_feedback', (), {})

    def disable(self):
        return self._call('disable', (), {})
