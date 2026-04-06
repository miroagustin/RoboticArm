"""Abstract base class and data model for perception backends.

A perception backend turns sensor data (camera frames, simulated scene, etc.)
into a list of objects in the robot's base frame. The MCP server exposes this
as the `scan_workspace()` tool.

Coordinate convention:
    - All positions are in millimetres, expressed in the robot base frame.
    - Orientations (when provided) are in degrees, matching the cartesian
      convention used by move_cartesian (rx, ry, rz).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime, timezone


@dataclass
class DetectedObject:
    """A single object detected in the robot workspace.

    Attributes:
        name:        Human-readable label the agent can reason about.
        position:    {"x", "y", "z"} in mm, in the robot base frame.
        confidence:  Detector confidence in [0, 1].
        orientation: Optional {"rx", "ry", "rz"} in degrees.
        source:      Which detector produced this (e.g. "aruco", "mock").
        metadata:    Backend-specific extras (marker id, class id, ...).
    """

    name: str
    position: dict
    confidence: float
    orientation: dict | None = None
    source: str = ""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        payload: dict = {
            "name": self.name,
            "position": self.position,
            "confidence": round(float(self.confidence), 4),
            "source": self.source,
        }
        if self.orientation is not None:
            payload["orientation"] = self.orientation
        if self.metadata:
            payload["metadata"] = self.metadata
        return payload


def scan_payload(objects: list[DetectedObject], backend: str) -> dict:
    """Wrap a list of detections in the standard scan_workspace response."""
    return {
        "objects": [o.to_dict() for o in objects],
        "count": len(objects),
        "backend": backend,
        "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }


class PerceptionBackend(ABC):
    """Interface every perception backend must implement.

    Implementations are expected to be cheap on construction and do the
    heavy work inside :meth:`scan`. If resources (camera, network) cannot
    be acquired, raise during __init__ so the server fails loudly instead
    of silently registering a broken tool.
    """

    @abstractmethod
    def scan(self) -> list[DetectedObject]:
        """Capture the current workspace and return detected objects."""

    def capabilities(self) -> dict:
        """Extra fields merged into the arm capabilities dict.

        Useful for advertising the set of known markers, camera info, etc.
        Override when the backend exposes metadata the agent can reason about.
        """
        return {}

    def close(self) -> None:
        """Release any resources held by the backend (cameras, sockets)."""
