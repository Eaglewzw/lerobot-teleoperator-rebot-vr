"""Run closed-loop PICO VR Cartesian teleoperation on a real reBot B601-DM."""

from __future__ import annotations

import logging
import signal
import sys
import time
from dataclasses import replace

import numpy as np

from ..control.controller import (
    FullBodyQPIKController,
)
from ..control.mit import MITCommandDispatcher
from ..config_rebot_vr import DEFAULT_BASE_T_ANCHOR, RebotVRConfig
from ..control.types import ARM_JOINT_NAMES, GRIPPER_NAME, CartesianControlConfig
from ..diagnostics.logger import CSVLogger, build_csv_row
from ..diagnostics.motor_io import MotorIODiagnostics
from ..ik.kinematics import B601Kinematics
from ..vr.controller import make_vr_controller
from .cli import (
    build_parser as _parser,
    follower_pos_vel_velocity as _follower_pos_vel_velocity,
    follower_relative_target as _follower_relative_target,
    status_line as _status_line,
    validate_args as _validate_args,
)
from .safety import (
    PersistentFeedbackFault,
    feedback_hold_action as _feedback_hold_action,
    move_to_initial_pose as _move_to_initial_pose,
    move_to_zero_pose as _move_to_zero_pose,
    send_feedback_hold_action as _send_feedback_hold_action,
    settle_persistent_feedback_fault as _settle_persistent_feedback_fault,
)
from .shutdown import ManagedFollower, ShutdownPolicy
from .feedback_requests import limit_startup_feedback_requests


logger = logging.getLogger(__name__)


def main() -> None:
    args = _parser().parse_args()
    _validate_args(args)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Build the model before opening the CAN device so dependency/model errors
    # cannot leave a powered robot connected without a control loop.
    kinematics = B601Kinematics(args.urdf, "gripper_end")
    vr_config = RebotVRConfig(
        hand_side=args.hand,
        clutch_threshold=args.grip_press,
        clutch_release_threshold=args.grip_release,
        stale_timeout=args.stale_timeout,
        ws_host=args.host,
        ws_port=args.port,
    )
    wrist_speed = (
        args.wrist_speed_rad_s
        if args.wrist_speed_rad_s is not None
        else args.max_joint_speed_rad_s
    )
    wrist_acceleration = (
        args.wrist_acceleration_rad_s2
        if args.wrist_acceleration_rad_s2 is not None
        else args.max_joint_acceleration_rad_s2
    )
    wrist_rel_target = (
        args.wrist_relative_target_deg
        if args.wrist_relative_target_deg is not None
        else args.max_relative_target_deg
    )
    gripper_rel_target = (
        args.gripper_relative_target_deg
        if args.gripper_relative_target_deg is not None
        else args.max_relative_target_deg
    )
    control_config = CartesianControlConfig(
        qp_solver=args.qp_solver,
        ik_mode=args.ik_mode,
        qp_position_cost=args.qp_position_cost,
        qp_orientation_cost=args.qp_orientation_cost,
        qp_orientation_cost_min=args.qp_orientation_cost_min,
        qp_position_gain=args.qp_position_gain,
        qp_orientation_gain=args.qp_orientation_gain,
        qp_damping=args.qp_damping,
        qp_damping_max=args.qp_damping_max,
        qp_smoothness_cost=args.qp_smoothness_cost,
        qp_posture_cost=args.qp_posture_cost,
        singularity_threshold=args.singularity_threshold,
        singularity_critical_threshold=args.singularity_critical_threshold,
        singularity_characteristic_length_m=(
            args.singularity_characteristic_length_m
        ),
        joint_limit_margin_deg=args.joint_limit_margin_deg,
        qp_max_solve_time_ms=args.qp_max_solve_time_ms,
        position_scale=args.position_scale,
        orientation_scale=args.orientation_scale,
        position_filter_hz=args.position_filter_hz,
        orientation_filter_hz=args.orientation_filter_hz,
        position_deadband_m=args.position_deadband_m,
        orientation_deadband_rad=np.deg2rad(args.orientation_deadband_deg),
        grip_press_threshold=args.grip_press,
        grip_release_threshold=args.grip_release,
        stale_timeout_s=args.stale_timeout,
        max_joint_speed_rad_s=args.max_joint_speed_rad_s,
        max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        wrist_speed_rad_s=wrist_speed,
        wrist_acceleration_rad_s2=wrist_acceleration,
        wrist_command_feedback_error_deg=wrist_rel_target * 0.9,
        arm_command_lookahead_s=args.arm_command_lookahead_ms * 1e-3,
        wrist_command_lookahead_s=args.wrist_command_lookahead_ms * 1e-3,
        gripper_command_feedback_error_deg=gripper_rel_target * 0.9,
        feedback_fault_max_consecutive=args.feedback_fault_max_consecutive,
        initial_q_rad=tuple(args.initial_q),
        gripper_max_speed_deg_s=args.gripper_max_speed_deg_s,
        gripper_max_acceleration_deg_s2=args.gripper_max_acceleration_deg_s2,
        gripper_open_deg=args.gripper_open_deg,
        gripper_closed_deg=args.gripper_closed_deg,
        max_command_feedback_error_deg=args.max_relative_target_deg * 0.9,
    )

    try:
        from lerobot.robots.rebot_b601_follower import (
            RebotB601Follower,
            RebotB601FollowerRobotConfig,
        )
    except ImportError as exc:
        raise ImportError(
            "This command requires LeRobot with rebot_b601_follower support (0.6.x)."
        ) from exc

    robot_config = RebotB601FollowerRobotConfig(
        port=args.robot_port,
        id=args.robot_id,
        can_adapter=args.can_adapter,
        dm_serial_baud=args.dm_serial_baud,
        control_mode=args.motor_control_mode,
        mit_kp=[*map(float, args.mit_kp), 8.0],
        mit_kd=[*map(float, args.mit_kd), 0.3],
        gripper_control_mode=args.gripper_control_mode,
        gripper_torque_ratio=args.gripper_torque_ratio,
        pos_vel_velocity=_follower_pos_vel_velocity(
            args.max_joint_speed_rad_s,
            wrist_speed,
            args.gripper_max_speed_deg_s,
        ),
        max_relative_target=_follower_relative_target(
            args.max_relative_target_deg,
            wrist_rel_target,
            gripper_rel_target,
        ),
        disable_torque_on_disconnect=args.disable_torque_on_disconnect,
    )
    robot = ManagedFollower(
        RebotB601Follower(robot_config),
        policy=ShutdownPolicy(args.disable_attempts, args.disable_interval_s,
                              args.disable_feedback_wait_s,
                              request_feedback=args.disable_request_feedback),
        report_path=(args.csv_log.with_name(args.csv_log.stem + "_shutdown.json")
                     if args.csv_log is not None else None),
    )
    if args.motor_control_mode == "mit":
        robot_io = MITCommandDispatcher(
            robot,
            kp=np.asarray(args.mit_kp, dtype=np.float64),
            kd=np.asarray(args.mit_kd, dtype=np.float64),
            torque_limit_nm=np.asarray(args.mit_torque_limit_nm, dtype=np.float64),
            arm_velocity_limit_rad_s=np.array(
                [
                    *([args.max_joint_speed_rad_s] * 3),
                    *([wrist_speed] * 3),
                ],
                dtype=np.float64,
            ),
            gravity_scale=args.mit_gravity_scale,
            gravity_ramp_s=args.mit_gravity_ramp_s,
            dynamics_urdf=args.mit_dynamics_urdf,
        )
    else:
        robot_io = robot
    vr_controller = make_vr_controller(vr_config)
    arm_controller = FullBodyQPIKController(
        kinematics,
        xr_to_base_rotation=np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3],
        config=control_config,
    )

    stop = False
    stop_signal = None
    abort_zero_return = False

    def stop_now(_signal_number, _frame) -> None:
        nonlocal stop, stop_signal, abort_zero_return
        if stop or _signal_number == signal.SIGTERM:
            abort_zero_return = True
        if not stop:
            stop_signal = _signal_number
        stop = True

    signal.signal(signal.SIGINT, stop_now)
    signal.signal(signal.SIGTERM, stop_now)

    vr_connected = False
    robot_connected = False
    arm_started = False
    preserve_torque_for_feedback_fault = False
    feedback_fault_active = False
    print("Support the arm before exit: torque is disabled on disconnect by default.")
    if args.return_to_zero_on_exit:
        print("Ctrl+C returns q1-q6 to zero before disconnecting; press Ctrl+C again to abort the return.")
    if args.motor_control_mode == "mit":
        print(
            "Arm motor mode: MIT with q1-q6 velocity targets and Pinocchio "
            f"gravity feedforward (scale={args.mit_gravity_scale:.2f})."
        )
        print(
            "MIT mode is experimental: support the arm and validate gravity signs "
            "at low gains before unrestricted motion."
        )
    else:
        print("Arm motor mode: POS_VEL.")
    print("Release Grip fully after tracking starts; hold Grip only when ready to move.")
    print(
        "Gripper mapping (deg): "
        f"Trigger 0 -> {control_config.gripper_open_deg:.1f}, "
        f"Trigger 1 -> {control_config.gripper_closed_deg:.1f}; "
        "fresh Tracking applies this mapping immediately."
    )
    initial_target_rad = None
    if args.move_to_initial:
        initial_target_rad = arm_controller.home_q_rad.copy()
        print(
            "Initial pose RS reference (rad): "
            f"{np.array2string(np.asarray(args.initial_q), precision=3, suppress_small=True)}; "
            "q2/q3 are sign-converted for B601-DM."
        )
    else:
        print("Initial-pose motion is disabled; VR control will start from actual feedback.")
    csv_logger = CSVLogger(args.csv_log) if args.csv_log is not None else None
    motor_diagnostics = None
    if csv_logger is not None:
        print(
            "CSV logging enabled: "
            f"frames={csv_logger.output_path} "
            f"latency_summary={csv_logger.summary_path}",
            flush=True,
        )
    try:
        vr_controller.connect()
        vr_connected = True
        robot_io.connect(calibrate=not args.no_calibrate)
        robot_connected = True
        if args.motor_diagnostics:
            motor_diagnostics = MotorIODiagnostics(args.csv_log, metadata=vars(args).copy())
            motor_diagnostics.install(robot)
            print(f"Motor API diagnostics: {motor_diagnostics.output_path}; "
                  "CAN reception timestamps/counters unavailable in this MotorBridge API.", flush=True)
        if args.move_to_initial:
            assert initial_target_rad is not None
            print(f"Initial feedback request cap: {args.initial_feedback_request_hz:g} Hz per motor "
                  "(0 = unlimited); motion loop frequency unchanged.", flush=True)
            with limit_startup_feedback_requests(robot, args.initial_feedback_request_hz):
                reached = _move_to_initial_pose(
                    robot_io,
                    target_rad=initial_target_rad,
                    lower_limit_rad=arm_controller.lower_limit_rad,
                    upper_limit_rad=arm_controller.upper_limit_rad,
                    args=args,
                    should_stop=lambda: stop,
                )
            if not reached:
                return
        arm_controller.start()
        arm_started = True

        started_s = time.monotonic()
        previous_loop_s = started_s
        next_status_s = started_s
        previous_command_send_finished_ns: int | None = None
        while not stop:
            loop_started_ns = time.monotonic_ns()
            loop_started_s = time.monotonic()
            if args.duration and loop_started_s - started_s >= args.duration:
                break
            dt_s = loop_started_s - previous_loop_s
            previous_loop_s = loop_started_s
            if motor_diagnostics is not None:
                motor_diagnostics.cycle_started_monotonic_ns = loop_started_ns
                motor_diagnostics.phase = 'loop_feedback'
            feedback_started_ns = time.monotonic_ns()
            try:
                observation = robot_io.get_observation()
            except Exception as exc:
                logger.warning(
                    "robot feedback read failed; entering transient HOLD: %s",
                    exc,
                )
                observation = {}
            feedback_finished_ns = time.monotonic_ns()
            feedback_read_ms = (feedback_finished_ns - feedback_started_ns) * 1e-6
            sample = vr_controller.latest_sample()
            frame = sample
            sample_pickup_ns = time.monotonic_ns()
            controller_started_ns = time.monotonic_ns()
            action, status = arm_controller.update(frame, observation, dt_s)
            feedback_fault_active = not status.feedback_valid
            controller_finished_ns = time.monotonic_ns()
            if stop and not status.feedback_abort_requested:
                break
            if motor_diagnostics is not None:
                motor_diagnostics.phase = status.state.value
            send_started_ns = time.monotonic_ns()
            if isinstance(robot_io, MITCommandDispatcher):
                if status.feedback_valid and status.state.value == "active":
                    robot_io.set_arm_velocity_from_position_error(
                        status.command_deg[:6],
                        status.actual_deg[:6],
                        np.array(
                            [
                                *([control_config.arm_command_lookahead_s] * 3),
                                *([control_config.wrist_command_lookahead_s] * 3),
                            ],
                            dtype=np.float64,
                        ),
                    )
                else:
                    robot_io.set_arm_velocity(None)
            if action is None:
                sent_action = None
            elif status.feedback_valid:
                sent_action = robot_io.send_action(action)
            else:
                sent_action = _send_feedback_hold_action(robot_io, action)
            send_finished_ns = time.monotonic_ns()
            send_action_ms = (send_finished_ns - send_started_ns) * 1e-6

            sample_received_ns = (
                0 if frame is None else int(frame.received_monotonic_ns)
            )
            sample_published_ns = (
                0
                if frame is None
                else int(getattr(frame, "published_monotonic_ns", 0))
            )
            command_sent = action is not None
            ik_command_sent = (
                command_sent
                and status.ik_result_consumed_this_cycle
                and status.ik_success is True
                and status.ik_sample_received_monotonic_ns is not None
            )
            status = replace(
                status,
                loop_started_monotonic_ns=loop_started_ns,
                tracking_sample_pickup_monotonic_ns=sample_pickup_ns,
                feedback_started_monotonic_ns=feedback_started_ns,
                feedback_finished_monotonic_ns=feedback_finished_ns,
                controller_started_monotonic_ns=controller_started_ns,
                controller_finished_monotonic_ns=controller_finished_ns,
                command_send_started_monotonic_ns=send_started_ns,
                command_send_finished_monotonic_ns=send_finished_ns,
                feedback_read_ms=feedback_read_ms,
                controller_update_ms=(
                    controller_finished_ns - controller_started_ns
                )
                * 1e-6,
                send_action_ms=send_action_ms,
                cycle_work_ms=(send_finished_ns - loop_started_ns) * 1e-6,
                vr_decode_ms=(
                    None
                    if sample_received_ns <= 0 or sample_published_ns <= 0
                    else max(
                        0.0,
                        (sample_published_ns - sample_received_ns) * 1e-6,
                    )
                ),
                latest_sample_wait_ms=(
                    None
                    if sample_published_ns <= 0
                    else max(
                        0.0,
                        (sample_pickup_ns - sample_published_ns) * 1e-6,
                    )
                ),
                tracking_receive_to_pickup_ms=(
                    None
                    if sample_received_ns <= 0
                    else max(
                        0.0,
                        (sample_pickup_ns - sample_received_ns) * 1e-6,
                    )
                ),
                tracking_receive_to_send_ms=(
                    None
                    if not command_sent or sample_received_ns <= 0
                    else max(
                        0.0,
                        (send_finished_ns - sample_received_ns) * 1e-6,
                    )
                ),
                ik_receive_to_send_ms=(
                    None
                    if not ik_command_sent
                    else max(
                        0.0,
                        (
                            send_finished_ns
                            - int(status.ik_sample_received_monotonic_ns)
                        )
                        * 1e-6,
                    )
                ),
                command_to_next_feedback_ms=(
                    None
                    if previous_command_send_finished_ns is None
                    else max(
                        0.0,
                        (
                            feedback_finished_ns
                            - previous_command_send_finished_ns
                        )
                        * 1e-6,
                    )
                ),
                motor_control_mode=args.motor_control_mode,
                mit_desired_velocity_rad_s=(
                    robot_io.desired_velocity_rad_s
                    if isinstance(robot_io, MITCommandDispatcher)
                    else None
                ),
                mit_gravity_torque_nm=(
                    robot_io.last_gravity_torque_nm.copy()
                    if isinstance(robot_io, MITCommandDispatcher)
                    and action is not None
                    else None
                ),
                mit_feedforward_torque_nm=(
                    robot_io.last_feedforward_torque_nm.copy()
                    if isinstance(robot_io, MITCommandDispatcher)
                    and action is not None
                    else None
                ),
            )
            if command_sent:
                previous_command_send_finished_ns = send_finished_ns
            if csv_logger is not None:
                csv_logger.write_row(build_csv_row(status))

            if status.feedback_abort_requested:
                print(_status_line(status, sent_action), flush=True)
                preserve_torque_for_feedback_fault = True
                _settle_persistent_feedback_fault(
                    robot_io,
                    observation,
                    sent_action if sent_action is not None else action,
                    duration_s=args.feedback_fault_settle_time,
                    fps=args.fps,
                )
                raise PersistentFeedbackFault(
                    "robot feedback remained invalid for "
                    f"{status.feedback_fault_count} consecutive frames: "
                    f"{status.feedback_fault_reason}"
                )

            if loop_started_s >= next_status_s:
                print(_status_line(status, sent_action), flush=True)
                next_status_s = loop_started_s + 1.0 / args.status_rate
            sleep_s = 1.0 / args.fps - (time.monotonic() - loop_started_s)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    finally:
        if motor_diagnostics is not None:
            motor_diagnostics.phase = 'shutdown'
        try:
            try:
                if arm_started:
                    arm_controller.stop()
            finally:
                try:
                    if vr_connected:
                        vr_controller.disconnect()
                finally:
                    try:
                        if robot_connected or robot.needs_cleanup:
                            try:
                                # Return only for a normal first SIGINT, never while
                                # unwinding an error or when feedback is in HOLD.
                                if (
                                    args.return_to_zero_on_exit
                                    and stop_signal == signal.SIGINT
                                    and not abort_zero_return
                                    and robot_connected
                                    and not feedback_fault_active
                                    and not preserve_torque_for_feedback_fault
                                    and sys.exc_info()[0] is None
                                ):
                                    if motor_diagnostics is not None:
                                        motor_diagnostics.phase = 'exit_zero'
                                    try:
                                        if isinstance(robot_io, MITCommandDispatcher):
                                            robot_io.set_arm_velocity(None)
                                        reached = _move_to_zero_pose(
                                            robot_io,
                                            lower_limit_rad=arm_controller.lower_limit_rad,
                                            upper_limit_rad=arm_controller.upper_limit_rad,
                                            args=args,
                                            should_stop=lambda: abort_zero_return,
                                        )
                                        if not reached:
                                            logger.warning("Return to zero interrupted; proceeding to disconnect.")
                                    except Exception:
                                        logger.exception("Return to zero failed; proceeding to disconnect.")
                            finally:
                                if motor_diagnostics is not None:
                                    motor_diagnostics.phase = 'shutdown'
                                if preserve_torque_for_feedback_fault:
                                    print(
                                        "Persistent feedback fault: retaining motor torque at the "
                                        "last HOLD command; support the arm before disabling power.",
                                        flush=True,
                                    )
                                    robot_io.config.disable_torque_on_disconnect = False
                                robot_io.disconnect()
                    finally:
                        kinematics.close()
        finally:
            if motor_diagnostics is not None:
                motor_diagnostics.restore()
                try:
                    motor_diagnostics.close()
                    print(f"Motor diagnostics summary: {motor_diagnostics.summary_path}", flush=True)
                except RuntimeError:
                    logger.exception("failed to finish motor diagnostics")
            if csv_logger is not None:
                try:
                    csv_logger.close()
                    print(
                        f"Latency summary written: {csv_logger.summary_path}",
                        flush=True,
                    )
                except RuntimeError:
                    logger.exception("failed to finalize CSV log")


if __name__ == "__main__":
    main()
