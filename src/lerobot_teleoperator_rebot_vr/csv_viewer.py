"""Compatibility entry point for the interactive CSV viewer."""

from .diagnostics.viewer import *  # noqa: F403
from .diagnostics.viewer import main


__all__ = ["main"]
