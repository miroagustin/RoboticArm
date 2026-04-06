"""Perception backends for the robot arm MCP server."""

from perception.base import DetectedObject, PerceptionBackend, scan_payload
from perception.mock import MockPerception

__all__ = [
    "DetectedObject",
    "PerceptionBackend",
    "scan_payload",
    "MockPerception",
]

# ArucoPerception is only importable when opencv-contrib-python is available.
try:
    from perception.aruco import ArucoPerception  # noqa: F401

    __all__.append("ArucoPerception")
except ImportError:  # pragma: no cover - optional dependency
    pass
