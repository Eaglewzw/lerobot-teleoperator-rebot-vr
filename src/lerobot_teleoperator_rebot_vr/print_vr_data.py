"""Compatibility entry point for the standalone VR data printer."""

from .tools.print_vr_data import *  # noqa: F403
from .tools.print_vr_data import main


__all__ = ["main"]
