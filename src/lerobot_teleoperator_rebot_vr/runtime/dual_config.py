"""Validated dual-arm assembly configuration, reusing single-arm profiles."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import math
import uuid

import yaml

from .cli import _flatten_mode_config, build_parser, validate_args


@dataclass(frozen=True)
class DualConfig:
    host: str
    port: int
    duration: float
    heartbeat_timeout_s: float
    arms: dict[str, argparse.Namespace]


def startup_zero_test_config(config: DualConfig) -> DualConfig:
    """An explicit diagnostic run: unchanged gains/tolerances, reduced motion."""
    arms = {}
    for side, original in config.arms.items():
        if original.csv_log is None:
            raise ValueError("startup-zero test requires runtime.log_dir for both arms")
        args = argparse.Namespace(**vars(original))
        args.move_to_initial = True
        args.return_to_zero_on_exit = True
        args.duration = 0.0
        for name, ceiling in (
            ("max_joint_speed_rad_s", .25), ("wrist_speed_rad_s", .25),
            ("exit_zero_speed_rad_s", .25),
            ("max_joint_acceleration_rad_s2", .5), ("wrist_acceleration_rad_s2", .5),
            ("exit_zero_acceleration_rad_s2", .5),
        ):
            value = getattr(args, name)
            setattr(args, name, ceiling if value is None else min(value, ceiling))
        validate_args(args)
        arms[side] = args
    return replace(config, duration=0.0, arms=arms)


def _mapping(value, allowed, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f"unknown {name} keys: {sorted(unknown)}")
    return value


def _path(value, directory):
    path = Path(value).expanduser()
    return (directory / path).resolve() if not path.is_absolute() else path.resolve()


def _typed_value(action, value):
    """Validate YAML values as strictly as explicit CLI arguments."""
    if isinstance(action, (argparse.BooleanOptionalAction,
                           argparse._StoreTrueAction, argparse._StoreFalseAction)):
        if not isinstance(value, bool):
            raise ValueError(f"{action.dest} must be a YAML boolean")
        return value
    if value is None:
        if action.default is not None:
            raise ValueError(f"{action.dest} cannot be null")
        return None
    values = value if isinstance(action.nargs, int) else [value]
    if not isinstance(values, (list, tuple)) or (
        isinstance(action.nargs, int) and len(values) != action.nargs
    ):
        raise ValueError(f"{action.dest} requires {action.nargs} values")
    converted = []
    for item in values:
        if isinstance(item, bool):
            raise ValueError(f"{action.dest} cannot be boolean")
        if action.type is int and (not isinstance(item, int)):
            raise ValueError(f"{action.dest} must be an integer")
        result = action.type(item) if action.type is not None else item
        if action.choices is not None and result not in action.choices:
            raise ValueError(f"invalid {action.dest}: {result!r}")
        if isinstance(result, float) and not math.isfinite(result):
            raise ValueError(f"{action.dest} must be finite")
        converted.append(result)
    return converted if isinstance(action.nargs, int) else converted[0]


def load_dual_config(path: Path) -> DualConfig:
    path = path.expanduser().resolve()
    data = _mapping(yaml.safe_load(path.read_text()),
                    {"vr", "arms", "runtime", "coordination"}, "dual config")
    vr = _mapping(data.get("vr", {}), {"host", "port"}, "vr")
    runtime = _mapping(data.get("runtime", {}),
                       {"duration", "log_dir", "heartbeat_timeout_s"}, "runtime")
    coordination = _mapping(data.get("coordination", {}), {"fault_policy"}, "coordination")
    if coordination.get("fault_policy", "hold_both") != "hold_both":
        raise ValueError("only coordination.fault_policy: hold_both is supported")
    host, port = vr.get("host", "0.0.0.0"), vr.get("port", 63901)
    if not isinstance(host, str) or not host:
        raise ValueError("vr.host must be a nonempty string")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("vr.port must be an integer in [1, 65535]")
    duration = float(runtime.get("duration", 0))
    heartbeat = float(runtime.get("heartbeat_timeout_s", 0.5))
    if not math.isfinite(duration) or duration < 0:
        raise ValueError("runtime.duration must be finite and nonnegative")
    if not math.isfinite(heartbeat) or heartbeat <= 0:
        raise ValueError("runtime.heartbeat_timeout_s must be finite and positive")
    arms = _mapping(data.get("arms"), {"left", "right"}, "arms")
    if set(arms) != {"left", "right"}:
        raise ValueError("arms must contain both left and right")
    parsed = {}
    run_id = uuid.uuid4().hex[:12]
    # These settings have exactly one owner: the dual assembly/session.
    reserved = {"hand", "host", "port", "robot_port", "robot_id", "control_config",
                "duration", "csv_log"}
    for side in ("left", "right"):
        arm = _mapping(arms[side],
                       {"hand", "robot_port", "robot_id", "control_config", "overrides"}, side)
        if arm.get("hand", side) != side:
            raise ValueError(f"{side}.hand must be {side}")
        for required in ("robot_port", "robot_id", "control_config"):
            if not isinstance(arm.get(required), str) or not arm[required].strip():
                raise ValueError(f"{side}.{required} must be a nonempty string")
        profile = _path(arm["control_config"], path.parent)
        defaults = _flatten_mode_config(yaml.safe_load(profile.read_text()), profile)
        overrides = _flatten_mode_config(arm.get("overrides", {}), path)
        if reserved & overrides.keys():
            raise ValueError(f"{side} overrides contain session-owned keys: {sorted(reserved & overrides.keys())}")
        parser = build_parser()
        actions = {a.dest: a for a in parser._actions if a.dest != "help"}
        unknown = (defaults.keys() | overrides.keys()) - actions.keys()
        if unknown:
            raise ValueError(f"unknown {side} parameters: {sorted(unknown)}")
        merged = {**defaults, **overrides}
        # Build a full namespace without writing a generated YAML file. The
        # single-arm validation remains the source of truth for control limits.
        args = argparse.Namespace(**{key: action.default for key, action in actions.items()})
        for key, value in merged.items():
            setattr(args, key, _typed_value(actions[key], value))
        for key in ("urdf", "mit_dynamics_urdf"):
            value = getattr(args, key)
            if value is not None:
                setattr(args, key, _path(value, path.parent if key in overrides else profile.parent))
        args.control_config = profile
        args.robot_port = arm["robot_port"]
        args.robot_id = arm["robot_id"]
        args.hand, args.host, args.port = side, host, port
        args.duration = 0.0  # Duration is measured once by the session.
        log_dir = runtime.get("log_dir")
        args.csv_log = (None if log_dir is None else
                        _path(log_dir, path.parent) / run_id / f"{side}.csv")
        validate_args(args)
        if heartbeat < 3.0 / args.fps:
            raise ValueError("heartbeat_timeout_s must allow at least three control cycles")
        parsed[side] = args
    if parsed["left"].robot_id == parsed["right"].robot_id:
        raise ValueError("left and right robot_id must differ (independent calibration)")
    left, right = parsed["left"].robot_port, parsed["right"].robot_port
    if left == right or Path(left).resolve() == Path(right).resolve():
        raise ValueError("left and right must use different CAN adapters")
    return DualConfig(host, port, duration, heartbeat, parsed)
