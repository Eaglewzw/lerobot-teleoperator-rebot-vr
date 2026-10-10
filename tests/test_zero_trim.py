import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.control.zero_trim import ZeroPoseTrim


def test_trim_reduces_static_gravity_model_error_without_negative_position_targets():
    trim = ZeroPoseTrim([12, 12, 12, 3, 3, 3])
    trim.active = True
    kp = np.array([50, 50, 50, 30, 50, 50])
    residual = np.deg2rad([.4, .65, 1.78, 1.1, .3, .05])
    q = residual.copy()
    previous = np.zeros(6)
    for i in range(501):
        bias = trim.update(q, np.zeros(6), i * .02)
        assert np.max(np.abs(bias - previous)) <= .01 + 1e-10
        assert np.all(np.abs(bias) <= trim.limit_nm + 1e-10)
        # First-order plant with a fixed gravity mismatch; not a hardware proof.
        q += (residual + bias / kp - q) * .02 / .2
        previous = bias
    assert np.max(np.abs(np.rad2deg(q))) < .5
    retained = previous.copy()
    trim.active = False
    for i in range(501, 1001):
        bias = trim.update(q, np.zeros(6), i * .02)
        assert np.max(np.abs(bias - previous)) <= .01 + 1e-10
        previous = bias
    assert np.any(retained != 0)
    assert bias == pytest.approx(np.zeros(6))


def test_trim_is_bounded_at_stall_and_does_not_integrate_through_feedback_gap():
    trim = ZeroPoseTrim([12, 12, 12, 3, 3, 3])
    trim.active = True
    q = np.deg2rad([4.] * 6)
    for i in range(1001):
        bias = trim.update(q, np.zeros(6), i * .02)
    assert bias == pytest.approx(-trim.limit_nm)
    assert trim.update(q, np.zeros(6), 100.) == pytest.approx(bias)


@pytest.mark.parametrize("active,actual,command", [(False, 2., 0.), (True, 6., 0.), (True, 2., 1.)])
def test_trim_requires_explicit_zero_return_and_near_zero_positions(active, actual, command):
    trim = ZeroPoseTrim([12, 12, 12, 3, 3, 3])
    trim.active = active
    for i in range(50):
        bias = trim.update(np.deg2rad([actual] * 6), np.deg2rad([command] * 6), i * .02)
    assert bias == pytest.approx(np.zeros(6))
