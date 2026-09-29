import numpy as np
import pytest
from types import SimpleNamespace

from lerobot_teleoperator_rebot_vr.control.reference_motion import CartesianReferenceMotion
from lerobot_teleoperator_rebot_vr.control.types import CartesianControlConfig
from lerobot_teleoperator_rebot_vr.ik.coordination import QPRequestCoordinator
from lerobot_teleoperator_rebot_vr.ik.kinematics import FullBodyQPIKSolver
from lerobot_teleoperator_rebot_vr.vr.pose_mapping import PoseTarget
from lerobot_teleoperator_rebot_vr.vr.pose_mapping import TeleopState
from lerobot_teleoperator_rebot_vr.ik.async_worker import IKResult
from lerobot_teleoperator_rebot_vr.vr.tracking import ControllerSample
from lerobot_teleoperator_rebot_vr.runtime.dual_config import load_dual_config
from pathlib import Path


@pytest.mark.parametrize('axis',[[1,0,0],[0,1,0],[1,2,-3]])
def test_reference_line_bounds_and_stop(axis):
    axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis)
    g=CartesianReferenceMotion(.65,1.5);g.reset(np.zeros(3))
    for i in range(800):
        dt=(.01,.025,.016)[i%3]
        previous=g.velocity.copy();old=g.position.copy()
        p,v=g.update(axis*.245,dt)
        assert np.linalg.norm(v)<=.65+1e-10
        assert np.linalg.norm(v-previous)<=1.5*dt+1e-10
        assert np.linalg.norm(np.cross(p,axis))<1e-10
        assert np.linalg.norm(p-old)<=.65*dt+1e-10
    assert p==pytest.approx(axis*.245,abs=1e-5)
    assert np.linalg.norm(v)<1e-4


def test_reversal_is_bounded_and_reset_discards_old_motion():
    g=CartesianReferenceMotion(.65,1.5);g.reset([0,0,0])
    for _ in range(35):g.update([.245,0,0],.01)
    for _ in range(600):
        previous=g.velocity.copy()
        p,v=g.update([-.1,.1,0],.01)
        assert np.linalg.norm(v-previous)<=.015+1e-10
    assert p==pytest.approx([-.1,.1,0],abs=1e-5)
    g.reset([1,2,3]);assert np.all(g.velocity==0)
    assert g.position==pytest.approx([1,2,3])


@pytest.mark.parametrize('dt',[0,-1,.1,np.nan])
def test_reference_invalid_dt(dt):
    with pytest.raises(ValueError):CartesianReferenceMotion(.5,1).update([0,0,0],dt)


def coordinator(config=None):
    worker=SimpleNamespace(submit=lambda r:setattr(worker,'request',r),clear=lambda:None)
    qp=QPRequestCoordinator(worker,config or CartesianControlConfig(),np.full(6,-3.),np.full(6,3.))
    return qp,worker


def submit(qp,now):
    frame=ControllerSample(now,now,1,'right',np.zeros(3),np.array([0,0,0,1]),1.,.5)
    return qp.submit_if_ready(target=PoseTarget(now,np.array([.2,0,0]),np.eye(3)),frame=frame,
       q_seed_rad=np.zeros(6),q_actual_rad=np.zeros(6),q_nominal_rad=np.zeros(6),dt_s=.01,now_ns=now,
       actual_position_m=np.zeros(3))


def test_qp_uses_sent_not_accepted_velocity_without_overwriting_accepted():
    qp,w=coordinator();qp._dq_rad_s[:]=.8
    qp.observe_sent_velocity(np.full(6,.2),1_000_000_000)
    assert submit(qp,1_010_000_000)
    assert w.request.dq_previous==pytest.approx(np.full(6,.2))
    assert qp.accepted_joint_velocity_rad_s==pytest.approx(np.full(6,.8))
    assert qp.request_diagnostics['ik_request_sent_velocity_age_ms']==pytest.approx(10)
    qp.begin_generation();assert qp._sent_velocity_ns is None


def test_governor_request_position_and_velocity_are_paired():
    qp,w=coordinator(CartesianControlConfig(split_reference_speed_m_s=.65))
    qp.reference_motion.reset([0,0,0]);submit(qp,1_000_000_000)
    assert 0<w.request.target_position[0]<.001
    assert 0<w.request.target_linear_velocity_m_s[0]<=.015+1e-10
    assert w.request.target_position[1:]==pytest.approx([0,0])
    assert qp.request_diagnostics['ik_request_position_error_x_m']==w.request.target_position[0]

@pytest.mark.parametrize('stamp',[900_000_000,1_100_000_000])
def test_stale_or_future_motor_snapshot_does_not_create_fake_zero_request(stamp):
    qp,w=coordinator();qp.observe_sent_velocity(np.ones(6),stamp)
    assert not submit(qp,1_000_000_000)
    assert not hasattr(w,'request')
    assert qp.sequence==0


def test_contour_cost_reduces_transverse_motion_under_joint_braking_constraint():
    class Model:
        lower_position_limit=np.full(6,-3.)
        upper_position_limit=np.full(6,3.)
        def tcp_pose_error(self,*args):return np.array([.03,0,0,0,0,0])
        def tcp_jacobian(self,q):
            j=np.zeros((6,6));j[:3,:3]=[[-.03,-.2,.1],[.03,0,0],[0,.1,-.1]]
            return j
    m=Model();args=dict(target_position=np.zeros(3),target_rotation=np.eye(3),q_actual=np.zeros(6),
        dq_previous=np.array([0,1,0,0,0,0]),dt=.02,q_nominal=np.zeros(6),max_joint_speed=np.full(6,2.),
        max_joint_acceleration=np.array([60.,6.,6.,6.,6.,6.]))
    def solve(weight):
        return FullBodyQPIKSolver(m,position_contour_weight=weight,
           max_solve_time_ms=100,position_gain=3,damping_min=.001,damping_max=.001).solve(**args)
    baseline=solve(1);guarded=solve(25)
    assert baseline.success and guarded.success
    assert abs(guarded.joint_velocity_rad_s[0])<abs(baseline.joint_velocity_rad_s[0])
    assert np.all(abs(guarded.joint_velocity_rad_s-args['dq_previous'])<=args['max_joint_acceleration']*.02+1e-8)


def test_failed_experimental_combination_is_not_enabled_by_default():
    cfg=load_dual_config(Path(__file__).parents[1]/'config/dual_mit_split.yaml')
    for args in cfg.arms.values():
        assert args.qp_use_sent_velocity is False
        assert args.split_contour_weight==1
        assert args.split_reference_speed_m_s==0


def test_deferred_request_uses_this_send_not_previous_send():
    qp,w=coordinator(CartesianControlConfig(qp_use_sent_velocity=True))
    qp.observe_sent_velocity(np.full(6,.2),990_000_000)
    assert submit(qp,1_000_000_000)
    assert not hasattr(w,'request')
    qp.observe_sent_velocity(np.full(6,.26),1_002_000_000)
    assert w.request.dq_previous==pytest.approx(np.full(6,.26))
    assert w.request.submitted_monotonic_ns==1_002_000_000
    assert qp.request_diagnostics['ik_request_dispatch_after_send'] is True
    assert qp.request_diagnostics['ik_request_sent_velocity_age_ms']==0


def test_dispatch_dt_uses_send_to_send_time_not_preparation_time():
    qp,w=coordinator(CartesianControlConfig(qp_use_sent_velocity=True))
    submit(qp,1_000_000_000)
    qp.observe_sent_velocity(np.zeros(6),1_004_000_000)
    qp._last_consumed_sequence=qp._last_submitted_sequence
    submit(qp,1_010_000_000)
    qp.observe_sent_velocity(np.full(6,.1),1_018_000_000)
    assert w.request.dt==pytest.approx(.014)
    assert qp.request_diagnostics['ik_request_prepared_monotonic_ns']==1_010_000_000
    assert qp.request_diagnostics['ik_request_submitted_monotonic_ns']==1_018_000_000
    assert qp._diagnostic_previous_velocity==pytest.approx(np.full(6,.1))


@pytest.mark.parametrize('reason',['hold','generation','slow_send'])
def test_deferred_request_cannot_escape_after_cancel(reason):
    qp,w=coordinator(CartesianControlConfig(qp_use_sent_velocity=True))
    submit(qp,1_000_000_000)
    if reason=='hold':qp.cancel_pending_motor_request()
    if reason=='generation':qp.begin_generation()
    qp.observe_sent_velocity(np.zeros(6),1_100_000_000 if reason=='slow_send' else 1_002_000_000)
    assert not hasattr(w,'request')
    assert submit(qp,1_110_000_000)
    qp.observe_sent_velocity(np.zeros(6),1_112_000_000)
    assert hasattr(w,'request')


def braking_pipeline(deferred):
    """One-cycle QP latency and the actual runtime's consume/submit/send order."""
    config=CartesianControlConfig(qp_use_sent_velocity=deferred,max_joint_speed_rad_s=2.,
                                  max_joint_acceleration_rad_s2=6.)
    qp,w=coordinator(config)
    w.result=None
    w.latest_result=lambda:w.result
    def solve(request):
        velocity=np.maximum(request.dq_previous-6*request.dt,0)
        w.result=IKResult(generation=request.generation,sequence=request.sequence,
            sample_id=request.sample_id,q_target_rad=request.q_actual+velocity*request.dt,
            joint_velocity_rad_s=velocity,success=True,position_error_m=0,solve_time_ms=1,
            submitted_monotonic_ns=request.submitted_monotonic_ns)
    w.submit=solve
    sent=np.full(6,2.);desired=sent.copy();qp._dq_rad_s=sent.copy()
    qp.observe_sent_velocity(sent,990_000_000)
    trace=[]
    for i in range(100):
        now=1_000_000_000+i*10_000_000
        result=qp.consume_latest(state=TeleopState.ACTIVE,q_actual_rad=np.zeros(6),now_ns=now)
        if result is not None:desired=qp.accepted_joint_velocity_rad_s
        submit(qp,now)
        sent+=np.clip(desired-sent,-.06,.06)
        qp.observe_sent_velocity(sent,now+1_000_000)
        trace.append(sent[0])
    return np.array(trace)


def test_motor_synchronized_dispatch_removes_two_cycle_braking_recurrence():
    old=braking_pipeline(False);new=braking_pipeline(True)
    assert np.flatnonzero(new<1e-8)[0]<=35
    assert np.flatnonzero(old<1e-8)[0]>=65
    assert new.sum()<old.sum()*.6
    assert np.max(abs(np.diff(new)))<=.06+1e-10
