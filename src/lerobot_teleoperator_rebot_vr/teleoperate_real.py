"""Run closed-loop PICO VR Cartesian teleoperation on a real reBot B601-DM."""

from __future__ import annotations

import logging
import signal
import time

import numpy as np

from .cartesian_controller import (
    ARM_JOINT_NAMES,
    GRIPPER_NAME,
    CartesianControlConfig,
    SplitArmWristController,
    vr_frame_from_raw_action,
)
from .config_rebot_vr import DEFAULT_BASE_T_ANCHOR, RebotVRConfig
from .kinematics import B601Kinematics
from .teleop_cli import (
    build_parser as _parser,
    follower_pos_vel_velocity as _follower_pos_vel_velocity,
    follower_relative_target as _follower_relative_target,
    status_line as _status_line,
    validate_args as _validate_args,
)
from .teleop_runtime import (
    PersistentFeedbackFault,
    feedback_hold_action as _feedback_hold_action,
    move_to_initial_pose as _move_to_initial_pose,
    send_feedback_hold_action as _send_feedback_hold_action,
    settle_persistent_feedback_fault as _settle_persistent_feedback_fault,
)
from .vr_controller import make_vr_controller


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
        vr_backend=args.backend,
        hand_side=args.hand,
        clutch_threshold=args.grip_press,
        clutch_release_threshold=args.grip_release,
        stale_timeout=args.stale_timeout,
        ws_host=args.host,
        ws_port=args.port,
        auto_launch_cloudxr=not args.no_cloudxr_launch,
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
        position_scale=args.position_scale,
        orientation_scale=args.orientation_scale,
        position_filter_hz=args.position_filter_hz,
        orientation_filter_hz=args.orientation_filter_hz,
        position_deadband_m=args.position_deadband_m,
        orientation_deadband_rad=np.deg2rad(args.orientation_deadband_deg),
        grip_press_threshold=args.grip_press,
        grip_release_threshold=args.grip_release,
        stale_timeout_s=args.stale_timeout,
        ik_rate_hz=args.ik_rate,
        ik_max_iterations=args.ik_max_iterations,
        ik_tolerance_m=args.ik_tolerance_m,
        ik_damping=args.ik_damping,
        max_solution_jump_rad=args.max_solution_jump_rad,
        max_joint_speed_rad_s=args.max_joint_speed_rad_s,
        max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        wrist_speed_rad_s=wrist_speed,
        wrist_acceleration_rad_s2=wrist_acceleration,
        wrist_command_feedback_error_deg=wrist_rel_target * 0.9,
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
        control_mode="pos_vel",
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
    robot = RebotB601Follower(robot_config)
    vr_controller = make_vr_controller(vr_config)
    arm_controller = SplitArmWristController(
        kinematics,
        xr_to_base_rotation=np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3],
        config=control_config,
    )

    stop = False

    def stop_now(_signal_number, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, stop_now)
    signal.signal(signal.SIGTERM, stop_now)

    vr_connected = False
    robot_connected = False
    arm_started = False
    preserve_torque_for_feedback_fault = False
    print("Support the arm before exit: torque is disabled on disconnect by default.")
    print("Release Grip fully after tracking starts; hold Grip only when ready to move.")
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
    try:
        vr_controller.connect()
        vr_connected = True
        robot.connect(calibrate=not args.no_calibrate)
        robot_connected = True
        if args.move_to_initial:
            assert initial_target_rad is not None
            reached = _move_to_initial_pose(
                robot,
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
        while not stop:
            loop_started_s = time.monotonic()
            if args.duration and loop_started_s - started_s >= args.duration:
                break
            dt_s = loop_started_s - previous_loop_s
            previous_loop_s = loop_started_s
            try:
                observation = robot.get_observation()
            except Exception as exc:
                logger.warning(
                    "robot feedback read failed; entering transient HOLD: %s",
                    exc,
                )
                observation = {}
            sample = vr_controller.latest_sample()
            frame = (
                sample
                if sample is not None or args.backend == "xrobotoolkit_v1"
                else vr_frame_from_raw_action(vr_controller.get_action())
            )
            action, status = arm_controller.update(frame, observation, dt_s)
            if action is None:
                sent_action = None
            elif status.feedback_valid:
                sent_action = robot.send_action(action)
            else:
                sent_action = _send_feedback_hold_action(robot, action)

            if status.feedback_abort_requested:
                print(_status_line(status, sent_action), flush=True)
                preserve_torque_for_feedback_fault = True
                _settle_persistent_feedback_fault(
                    robot,
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
        try:
            if arm_started:
                arm_controller.stop()
        finally:
            try:
                if vr_connected:
                    vr_controller.disconnect()
            finally:
                try:
                    if robot_connected or robot.is_connected:
                        if preserve_torque_for_feedback_fault:
                            print(
                                "Persistent feedback fault: retaining motor torque at the "
                                "last HOLD command; support the arm before disabling power.",
                                flush=True,
                            )
                            robot.config.disable_torque_on_disconnect = False
                        robot.disconnect()
                finally:
                    kinematics.close()


if __name__ == "__main__":
    main()
