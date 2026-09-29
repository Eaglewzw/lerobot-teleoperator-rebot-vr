import numpy as np
import pytest
from scipy.optimize import lsq_linear

from lerobot_teleoperator_rebot_vr.ik.box_qp import solve_box_qp3


def test_box_qp_matches_independent_bounded_least_squares():
    rng = np.random.default_rng(721)
    for _ in range(200):
        a = np.vstack((rng.normal(size=(5, 3)), .03 * np.eye(3)))
        b = rng.normal(size=8) * 4
        lower = rng.uniform(-2, 1, 3)
        upper = lower + rng.uniform(.001, 3, 3)
        h, g = a.T @ a, -a.T @ b
        actual = solve_box_qp3(h, g, lower, upper)
        reference = lsq_linear(a, b, bounds=(lower, upper), tol=1e-12)
        assert actual == pytest.approx(reference.x, abs=2e-6)
        assert np.all(actual >= lower) and np.all(actual <= upper)


@pytest.mark.parametrize("fixed", [1, 2, 3])
def test_box_qp_fixed_coordinates_and_coupling(fixed):
    h = np.array([[3., .4, -.3], [.4, 2., .2], [-.3, .2, 1.]])
    expected = np.array([.1, -.2, .3])
    lower, upper = np.full(3, -1.), np.full(3, 1.)
    lower[:fixed] = upper[:fixed] = expected[:fixed]
    actual = solve_box_qp3(h, -h @ expected, lower, upper)
    assert actual == pytest.approx(expected, abs=1e-12)


def test_box_qp_binding_acceleration_window():
    lower = np.array([.9, -.4, .2])
    upper = lower + .06
    actual = solve_box_qp3(np.eye(3), np.array([-4., 4., -4.]), lower, upper)
    assert actual == pytest.approx([upper[0], lower[1], upper[2]])
