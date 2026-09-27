"""Bounded Cartesian feedforward; never modifies the mapped position target."""

import numpy as np


class LinearVelocityEstimator:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.position = None
        self.key = None
        self.velocity = np.zeros(3)
        self._brake_elapsed = np.zeros(3)
        self._brake_samples = np.zeros(3, dtype=int)
        self.diagnostics = {}

    def capture(self, position, key):
        self.reset()
        self.position = np.asarray(position).copy()
        self.key = key

    def update(self, position, key):
        position = np.asarray(position, dtype=float)
        if key == self.key and key is not None:
            return self.velocity.copy()
        previous, previous_key = self.position, self.key
        self.position, self.key = position.copy(), key
        dt = None
        source = "reset"
        if previous_key is not None and key is not None and key[0] == previous_key[0]:
            receive_dt = (key[2] - previous_key[2]) * 1e-9
            source_dt = (key[1] - previous_key[1]) * 1e-9
            # A receive gap also invalidates history, even if the sender clock
            # advanced by only one frame after reconnecting.
            if 0 < receive_dt <= self.config.stale_timeout_s:
                if key[1] > 0 and previous_key[1] > 0 and .001 <= source_dt <= self.config.stale_timeout_s:
                    dt, source = source_dt, "source"
                elif .001 <= receive_dt <= self.config.stale_timeout_s:
                    dt, source = receive_dt, "receive_fallback"
        raw = np.zeros(3)
        jump = speed_limited = rate_limited = False
        braking = np.zeros(3, dtype=bool)
        if dt is None or previous is None:
            self.velocity.fill(0.)
            self._clear_brake_confirmation()
        else:
            delta = position - previous
            raw = delta / dt
            jump = bool(np.linalg.norm(delta) > (
                self.config.linear_ff_jump_slack_m + self.config.linear_ff_jump_speed_m_s * dt
            ))
            if jump:
                # Drop this derivative and rebase once; do not turn one bad
                # position sample into a long, rate-limited velocity pulse.
                self.velocity.fill(0.)
                self._clear_brake_confirmation()
            else:
                norm = float(np.linalg.norm(raw))
                speed_limited = norm > self.config.linear_ff_max_speed_m_s
                bounded = raw * min(1., self.config.linear_ff_max_speed_m_s / max(norm, 1e-15))
                # Limit by wall receive time too: compressed arrivals must
                # not consume a whole source-frame acceleration budget at once.
                step = min(dt, receive_dt)
                # Confirm per axis: motion on Y/Z must not conceal an X turn.
                # Ignore tiny opposite noise; never fast-build the new direction.
                significant = abs(self.velocity) > .02
                stopped = abs(bounded) <= .02
                reversed_axis = (bounded * self.velocity < 0) & (abs(bounded) > .02)
                candidate = significant & (stopped | reversed_axis)
                self._brake_elapsed = np.where(candidate, self._brake_elapsed + step, 0.)
                self._brake_samples = np.where(candidate, self._brake_samples + 1, 0)
                braking = candidate & (self._brake_samples >= 2) & (
                    self._brake_elapsed >= self.config.linear_ff_brake_confirm_s
                )
                # Braking is toward zero only, not through it. New-direction
                # build-up starts on a later sample at the original slow rate.
                destination = np.where(braking, 0., bounded)
                change = destination - self.velocity
                brake_change = np.where(braking, change, 0.)
                normal_change = np.where(braking, 0., change)
                brake_norm = float(np.linalg.norm(brake_change))
                normal_norm = float(np.linalg.norm(normal_change))
                max_brake = self.config.linear_ff_max_deceleration_m_s2 * step
                max_change = self.config.linear_ff_max_acceleration_m_s2 * step
                brake_change *= min(1., max_brake / max(brake_norm, 1e-15))
                normal_change *= min(1., max_change / max(normal_norm, 1e-15))
                # One shared vector budget when normal and braking axes coexist.
                update = brake_change + normal_change
                budget = max(max_change, max_brake) if np.any(braking) else max_change
                update *= min(1., budget / max(float(np.linalg.norm(update)), 1e-15))
                self.velocity += update
                # Mixed axes can change direction while braking; retain the
                # Cartesian speed envelope without amplifying any component.
                self.velocity *= min(1., self.config.linear_ff_max_speed_m_s /
                                     max(float(np.linalg.norm(self.velocity)), 1e-15))
                rate_limited = not np.allclose(self.velocity, bounded, rtol=0., atol=1e-12)
        self.diagnostics = {
            "ik_request_linear_velocity_dt_s": dt,
            "ik_request_linear_velocity_time_source": source,
            "ik_request_linear_velocity_jump_rejected": jump,
            "ik_request_linear_velocity_speed_limited": speed_limited,
            "ik_request_linear_velocity_rate_limited": rate_limited,
            **{f"ik_request_linear_velocity_fast_brake_{a}_flag": bool(braking[i])
               for i, a in enumerate("xyz")},
            **{f"ik_request_raw_linear_velocity_{a}_m_s": float(raw[i]) for i, a in enumerate("xyz")},
        }
        return self.velocity.copy()

    def _clear_brake_confirmation(self):
        self._brake_elapsed.fill(0.)
        self._brake_samples.fill(0)
