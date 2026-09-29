import numpy as np
import pytest
from test_mit_control import _FakeRobot, _dispatcher
from lerobot_teleoperator_rebot_vr.control import mit
from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser, validate_args


@pytest.mark.parametrize('limit',[1.,2.])
def test_q1_reference_bounds_reversal_stale_reset_and_packet_logging(monkeypatch,limit):
    clock=[1.];monkeypatch.setattr(mit.time,'monotonic',lambda:clock[0])
    robot=_FakeRobot(max_relative_target=5.)
    d=_dispatcher(robot,acceleration_limit_rad_s2=np.ones(6),
                  position_lookahead_s=np.full(6,.05),q1_reference_error_deg=limit)
    d.get_observation();action={f'{n}.pos':0. for n in robot.motor_names}
    previous=0.
    for _ in range(100):
        clock[0]+=.01;d.set_arm_velocity(np.array([.1,0,0,0,0,0]));sent=d.send_action(action)
        packet=robot.motors['shoulder_pan'].mit_calls[-1]
        assert 0<=packet[0]<=np.deg2rad(limit)+1e-10
        assert abs(packet[0]-previous)<=.001+1e-10
        assert sent['shoulder_pan.pos']==pytest.approx(np.rad2deg(packet[0]))
        previous=packet[0]
    assert previous==pytest.approx(np.deg2rad(limit))
    for _ in range(30):
        clock[0]+=.01;d.set_arm_velocity(np.array([-.1,0,0,0,0,0]));d.send_action(action)
    assert d.q1_reference<previous
    clock[0]+=.1
    assert d.stop_stale_arm_velocity(.05)
    assert d.q1_reference is None
    d.stop_arm_velocity(immediate=True)
    action['shoulder_pan.pos']=-.5;d.send_action(action)
    assert robot.motors['shoulder_pan'].mit_calls[-1][0]==pytest.approx(np.deg2rad(-.5))
    assert robot.motors['shoulder_pan'].mit_calls[-1][1]==0
    assert d.q1_reference is None


def test_reference_still_obeys_hardware_position_window(monkeypatch):
    clock=[1.];monkeypatch.setattr(mit.time,'monotonic',lambda:clock[0])
    r=_FakeRobot(max_relative_target=.2)
    d=_dispatcher(r,acceleration_limit_rad_s2=np.ones(6),
                  position_lookahead_s=np.full(6,.05),q1_reference_error_deg=2.)
    d.get_observation();action={f'{n}.pos':0. for n in r.motor_names}
    for _ in range(100):
        clock[0]+=.01;d.set_arm_velocity(np.array([.1,0,0,0,0,0]));sent=d.send_action(action)
        assert abs(sent['shoulder_pan.pos'])<=.2+1e-9
    assert d.q1_reference==pytest.approx(np.deg2rad(.2))


def test_feedback_fault_clears_reference_and_holds_last_packet():
    from lerobot_teleoperator_rebot_vr.runtime.safety import send_feedback_hold_action
    r=_FakeRobot();d=_dispatcher(r,acceleration_limit_rad_s2=np.ones(6),
        position_lookahead_s=np.full(6,.05),q1_reference_error_deg=2.)
    d.get_observation();d.set_arm_velocity(np.array([.1,0,0,0,0,0]))
    sent=d.send_action({f'{n}.pos':0. for n in r.motor_names})
    assert d.q1_reference is not None
    d.set_observation({'shoulder_pan.pos':float('nan')})
    assert send_feedback_hold_action(d,sent)==sent
    assert d.q1_reference is None
    assert r.motors['shoulder_pan'].mit_calls[-1][1]==0.


@pytest.mark.parametrize('value',[-1,2.1,float('nan')])
def test_reference_rejects_unbounded_settings(value):
    with pytest.raises(ValueError):
        _dispatcher(_FakeRobot(),q1_reference_error_deg=value)


def test_cli_rejects_reference_outside_mit_split():
    args=build_parser().parse_args([])
    args.mit_q1_reference_error_deg=2
    with pytest.raises(ValueError,match='MIT motor control'):
        validate_args(args)


def test_q1_profile_changes_only_validated_settings():
    from pathlib import Path
    from lerobot_teleoperator_rebot_vr.runtime.dual_config import load_dual_config
    root=Path(__file__).parents[1]/'config'
    baseline=load_dual_config(root/'dual_mit_split.yaml')
    profile=load_dual_config(root/'dual_mit_split_q1.yaml')
    for side in ('left','right'):
        a,b=vars(baseline.arms[side]),vars(profile.arms[side])
        changed={k for k in a if a[k]!=b[k]}-{'csv_log'}
        assert changed=={'split_contour_mode','split_contour_weight','mit_q1_reference_error_deg','mit_kd','qp_use_sent_velocity'}
        assert b['qp_use_sent_velocity'] is True
        assert b['split_contour_weight']==25
    assert profile.arms['left'].mit_q1_reference_error_deg==2
    assert profile.arms['left'].split_contour_mode=='shoulder_lateral'
    assert profile.arms['left'].split_contour_weight==25
