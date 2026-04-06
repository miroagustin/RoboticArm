"""Mock perception backend.

Returns a deterministic list of objects so the agent pipeline can be developed
and validated without a camera. Objects can be loaded from a JSON file via
MCP_PERCEPTION_MOCK_FILE; otherwise a sensible default scene is used.

The default scene roughly matches what a myCobot 280 Pi can reach:
positions inside a ~280 mm radius, z slightly above the base plane.
"""

import json
import logging
import os

from perception.base import DetectedObject, PerceptionBackend

logger = logging.getLogger(__name__)

# A stable workspace the agent can reason about during development.
_DEFAULT_SCENE: list[dict] = [
    {
        "name": "taza",
        "position": {"x": 180.0, "y": 60.0, "z": 40.0},
        "confidence": 0.97,
        "source": "mock",
        "metadata": {"class": "cup"},
    },
    {
        "name": "lapicera",
        "position": {"x": 210.0, "y": -40.0, "z": 12.0},
        "confidence": 0.94,
        "source": "mock",
        "metadata": {"class": "pen"},
    },
    {
        "name": "cuaderno",
        "position": {"x": 150.0, "y": 95.0, "z": 6.0},
        "confidence": 0.92,
        "source": "mock",
        "metadata": {"class": "notebook"},
    },
]


class MockPerception(PerceptionBackend):
    """In-memory perception that returns a fixed (optionally file-backed) scene.

    Args:
        scene_file: Optional path to a JSON array with the same shape as the
                    default scene above. When the file is missing or invalid
                    the default scene is used and a warning is logged.
    """

    def __init__(self, scene_file: str | None = None) -> None:
        self._scene_file = scene_file
        self._objects: list[DetectedObject] = self._load(scene_file)
        logger.info(
            "MockPerception ready with %d objects (source: %s)",
            len(self._objects),
            scene_file or "built-in defaults",
        )

    @staticmethod
    def _load(scene_file: str | None) -> list[DetectedObject]:
        raw_items: list[dict]
        if scene_file and os.path.isfile(scene_file):
            try:
                with open(scene_file, encoding="utf-8") as f:
                    raw_items = json.load(f)
                if not isinstance(raw_items, list):
                    raise ValueError("scene file must contain a JSON array")
            except Exception as exc:
                logger.warning(
                    "Failed to load mock scene from %s (%s). Using defaults.",
                    scene_file,
                    exc,
                )
                raw_items = list(_DEFAULT_SCENE)
        else:
            if scene_file:
                logger.warning(
                    "Mock scene file %s not found. Using defaults.", scene_file
                )
            raw_items = list(_DEFAULT_SCENE)

        objects: list[DetectedObject] = []
        for item in raw_items:
            try:
                objects.append(
                    DetectedObject(
                        name=item["name"],
                        position=item["position"],
                        confidence=float(item.get("confidence", 1.0)),
                        orientation=item.get("orientation"),
                        source=item.get("source", "mock"),
                        metadata=item.get("metadata", {}),
                    )
                )
            except KeyError as exc:
                logger.warning("Skipping invalid scene entry %s: missing %s", item, exc)
        return objects

    def scan(self) -> list[DetectedObject]:
        # Return copies so callers cannot mutate internal state.
        return [
            DetectedObject(
                name=o.name,
                position=dict(o.position),
                confidence=o.confidence,
                orientation=dict(o.orientation) if o.orientation else None,
                source=o.source,
                metadata=dict(o.metadata),
            )
            for o in self._objects
        ]

    def capabilities(self) -> dict:
        return {
            "perception_backend": "mock",
            "perception_objects": [o.name for o in self._objects],
        }
