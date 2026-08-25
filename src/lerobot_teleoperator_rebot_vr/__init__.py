"""LeRobot third-party PICO 4 teleoperator for reBot B601-DM."""

import sys as _sys

from .control import controller as _cartesian_controller
from .control import joint_command as _joint_command
from .control import startup as _startup_pose
from .control import status as _control_status
from .control import types as _control_types
from .control.controller import FullBodyQPIKController
from .control.types import CartesianControlConfig
from .config_rebot_vr import RebotVRConfig, RebotVRTeleopConfig
from .diagnostics import analysis as _csv_analysis
from .diagnostics import logger as _csv_logger
from .ik import async_worker as _async_ik
from .ik import coordination as _qp_coordination
from .ik import kinematics as _kinematics
from .ik.kinematics import B601Kinematics
from .rebot_vr import RebotVRTeleop
from .runtime import cli as _teleop_cli
from .runtime import safety as _teleop_runtime
from .vr import adapter as _vr_adapter
from .vr import controller as _vr_controller
from .vr import models as _processor
from .vr import pose_mapping as _pose_mapping
from .vr import tracking as _tracking
from .vr import xr_v1 as _xr_v1
from .vr.controller import Pico4VRController, XRoboToolkitV1Controller
from .vr.models import VRFrame
from .vr.tracking import (
    ControllerSample,
    LatestSampleBuffer,
    TrackingSampleError,
    parse_controller_sample,
)
from .vr.xr_v1 import PacketParser, PacketStreamDecoder, TrackingDecoder, V1TrackingSource


# Preserve pre-0.4 module imports while implementation lives in domain packages.
_LEGACY_MODULES = {
    "async_ik": _async_ik,
    "cartesian_controller": _cartesian_controller,
    "control_status": _control_status,
    "control_types": _control_types,
    "csv_analysis": _csv_analysis,
    "csv_logger": _csv_logger,
    "joint_command": _joint_command,
    "kinematics": _kinematics,
    "pose_mapping": _pose_mapping,
    "processor": _processor,
    "qp_coordination": _qp_coordination,
    "startup_pose": _startup_pose,
    "teleop_cli": _teleop_cli,
    "teleop_runtime": _teleop_runtime,
    "tracking": _tracking,
    "vr_adapter": _vr_adapter,
    "vr_controller": _vr_controller,
    "xr_v1": _xr_v1,
}
for _legacy_name, _legacy_module in _LEGACY_MODULES.items():
    _sys.modules.setdefault(f"{__name__}.{_legacy_name}", _legacy_module)


__all__ = [
    "B601Kinematics",
    "CartesianControlConfig",
    "ControllerSample",
    "LatestSampleBuffer",
    "PacketParser",
    "PacketStreamDecoder",
    "Pico4VRController",
    "RebotVRConfig",
    "RebotVRTeleop",
    "RebotVRTeleopConfig",
    "FullBodyQPIKController",
    "TrackingDecoder",
    "TrackingSampleError",
    "VRFrame",
    "V1TrackingSource",
    "XRoboToolkitV1Controller",
    "parse_controller_sample",
]
