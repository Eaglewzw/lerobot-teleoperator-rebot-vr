"""Bounded Cartesian reference motion; never modifies the raw VR target."""
from __future__ import annotations

import numpy as np


class CartesianReferenceMotion:
    """Generate position and velocity together, with stopping-distance braking.

    This is a reference governor, not a guarantee about physical stopping
    distance. Motor feedback and the existing hardware safety gates still apply.
    """

    def __init__(self, speed: float, acceleration: float):
        if not np.isfinite([speed, acceleration]).all() or min(speed, acceleration) <= 0:
            raise ValueError("reference speed and acceleration must be positive")
        self.speed = float(speed)
        self.acceleration = float(acceleration)
        self.position = None
        self.velocity = np.zeros(3)

    def reset(self, position=None):
        self.position = None if position is None else self._vector(position)
        self.velocity.fill(0.)

    @staticmethod
    def _vector(value):
        result = np.asarray(value, dtype=float)
        if result.shape != (3,) or not np.isfinite(result).all():
            raise ValueError("reference position must be a finite three-vector")
        return result.copy()

    def update(self, target, dt):
        target = self._vector(target)
        if not np.isfinite(dt) or dt <= 0 or dt > .05 + 1e-9:
            raise ValueError("reference dt must be in (0, 0.05]")
        if self.position is None:
            self.reset(target)
        # Substeps bound discretization error under variable IK cadence.
        steps = max(1, int(np.ceil(dt / .002)))
        h = dt / steps
        a = self.acceleration
        for _ in range(steps):
            error = target - self.position
            distance = float(np.linalg.norm(error))
            stop_speed = np.sqrt((a*h)**2 + 2*a*distance) - a*h
            desired = error * (min(self.speed, stop_speed) / max(distance, 1e-12))
            change = desired - self.velocity
            change *= min(1., a*h / max(float(np.linalg.norm(change)), 1e-12))
            previous = self.velocity.copy()
            self.velocity += change
            self.position += (previous + self.velocity) * (.5*h)
        return self.position.copy(), self.velocity.copy()
