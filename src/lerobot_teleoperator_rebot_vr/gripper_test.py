"""Compatibility entry point for the standalone gripper test."""

from .tools.gripper_test import *  # noqa: F403
from .tools.gripper_test import main


__all__ = ["main"]
