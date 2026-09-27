import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.control.types import CartesianControlConfig
from lerobot_teleoperator_rebot_vr.vr.linear_velocity import LinearVelocityEstimator


def test_source_clock_prevents_compressed_receive_velocity_spike():
    estimator = LinearVelocityEstimator(CartesianControlConfig(linear_ff_max_acceleration_m_s2=100.))
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    velocity = estimator.update([.002, 0, 0], (1, 1_010_000_000, 2_004_000_000))
    assert velocity == pytest.approx([.2, 0, 0])
    assert estimator.diagnostics['ik_request_linear_velocity_time_source'] == 'source'
    assert estimator.diagnostics['ik_request_linear_velocity_dt_s'] == pytest.approx(.01)


@pytest.mark.parametrize('timestamp', [0, 1_000_000_000, 900_000_000, 5_000_000_000])
def test_invalid_source_uses_bounded_receive_fallback(timestamp):
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    velocity = estimator.update([.004, 0, 0], (1, timestamp, 2_010_000_000))
    assert velocity == pytest.approx([.01, 0, 0])
    assert estimator.diagnostics['ik_request_linear_velocity_time_source'] == 'receive_fallback'
    assert estimator.diagnostics['ik_request_linear_velocity_speed_limited']
    assert estimator.diagnostics['ik_request_linear_velocity_rate_limited']


def test_jump_is_rejected_and_rebased_without_changing_target():
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    position = np.array([0., 0., .025])
    assert estimator.update(position, (1, 1_010_000_000, 2_010_000_000)) == pytest.approx(np.zeros(3))
    assert estimator.diagnostics['ik_request_linear_velocity_jump_rejected']
    np.testing.assert_array_equal(position, [0., 0., .025])
    assert estimator.update(position + [0, 0, .001], (1, 1_020_000_000, 2_020_000_000)) == pytest.approx([0, 0, .01])


def test_limits_vector_norm_reversal_and_duplicate_sample():
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    p = np.zeros(3)
    key = (1, 1_000_000_000, 2_000_000_000)
    estimator.capture(p, key)
    previous = np.zeros(3)
    for i in range(1, 101):
        p += np.array([.004, .003, 0]) * (1 if i < 50 else -1)
        key = (1, 1_000_000_000+i*10_000_000, 2_000_000_000+i*10_000_000)
        velocity = estimator.update(p, key)
        assert np.linalg.norm(velocity) <= .3 + 1e-12
        assert np.linalg.norm(velocity-previous) <= .04 + 1e-12
        np.testing.assert_array_equal(estimator.update(p, key), velocity)
        previous = velocity
    assert velocity[0] < 0
    assert estimator.update(p, (2, key[1]+10_000_000, key[2]+10_000_000)) == pytest.approx(np.zeros(3))


def test_stale_gap_and_capture_reset_velocity():
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    assert estimator.update([.001,0,0], (1,1_010_000_000,2_500_000_000)) == pytest.approx(np.zeros(3))
    assert estimator.diagnostics['ik_request_linear_velocity_time_source'] == 'reset'
    estimator.capture(np.zeros(3), (2,0,3_000_000_000))
    np.testing.assert_array_equal(estimator.velocity, np.zeros(3))


@pytest.mark.parametrize('field', ['linear_ff_max_speed_m_s', 'linear_ff_max_acceleration_m_s2', 'linear_ff_max_deceleration_m_s2', 'linear_ff_brake_confirm_s', 'linear_ff_jump_speed_m_s', 'linear_ff_jump_slack_m'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_settings(field, value):
    with pytest.raises(ValueError):
        CartesianControlConfig(**{field: value})


@pytest.mark.parametrize('reverse', [False, True])
def test_confirmed_stop_or_reversal_brakes_without_fast_reverse_build_up(reverse):
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    estimator.velocity[:] = [.2, 0, 0]
    position = np.zeros(3)
    values = []
    for i in range(1, 31):
        position[0] += -.002 if reverse else 0.
        before = estimator.velocity.copy()
        value = estimator.update(position, (1, 1_000_000_000+i*10_000_000,
                                             2_000_000_000+i*10_000_000))
        fast = estimator.diagnostics['ik_request_linear_velocity_fast_brake_x_flag']
        if i <= 2:
            assert not fast
        if fast:
            assert 0 <= value[0] <= before[0]
        if value[0] < 0:
            assert abs(value[0]) - max(0., -before[0]) <= .01 + 1e-12
        values.append(value[0])
    assert next(i for i, x in enumerate(values) if x <= .02) < 10
    assert values[-1] == pytest.approx(-.2 if reverse else 0.)


def test_one_bad_frame_does_not_confirm_brake_even_with_long_interval():
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1, 1_000_000_000, 2_000_000_000))
    estimator.velocity[:] = [.2, 0, 0]
    estimator.update([0.,0,0], (1,1_050_000_000,2_050_000_000))
    assert not estimator.diagnostics['ik_request_linear_velocity_fast_brake_x_flag']
    estimator.update([.002,0,0], (1,1_060_000_000,2_060_000_000))
    assert not estimator.diagnostics['ik_request_linear_velocity_fast_brake_x_flag']
    assert estimator._brake_samples[0] == 0


def test_axis_reversal_braking_does_not_fast_accelerate_other_axis():
    estimator = LinearVelocityEstimator(CartesianControlConfig())
    estimator.capture(np.zeros(3), (1,1_000_000_000,2_000_000_000))
    estimator.velocity[:] = [.15,.1,0.]
    position = np.zeros(3)
    for i in range(1, 31):
        position += [-.001, .002, .001]
        before = estimator.velocity.copy()
        velocity = estimator.update(position, (1,1_000_000_000+i*10_000_000,
                                                 2_000_000_000+i*10_000_000))
        assert np.linalg.norm(velocity) <= .3 + 1e-12
        assert np.linalg.norm(velocity-before) <= .04 + 1e-12
        assert np.linalg.norm(velocity[1:]-before[1:]) <= .01 + 1e-12
        if i == 3:
            assert estimator.diagnostics['ik_request_linear_velocity_fast_brake_x_flag']
            assert not estimator.diagnostics['ik_request_linear_velocity_fast_brake_y_flag']
    assert velocity[0] < 0
