"""Shared collision-scene and basic planning helpers for arm backends."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from typing import Any

logger = logging.getLogger(__name__)

_SUPPORTED_SHAPES = {"box", "cylinder", "sphere"}
_DEFAULT_BOX_SIZE_MM = {"x": 40.0, "y": 40.0, "z": 40.0}
_SCENE_DIMENSIONS_BY_CLASS = {
    "cup": {"x": 80.0, "y": 80.0, "z": 100.0},
    "pen": {"x": 160.0, "y": 18.0, "z": 18.0},
    "notebook": {"x": 180.0, "y": 130.0, "z": 20.0},
}


@dataclass
class CollisionObject:
    """Normalized obstacle stored in the planner scene."""

    name: str
    shape_type: str
    dimensions: dict[str, float]
    pose: dict[str, float]
    source: str = "manual"
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        shape: dict[str, Any] = {"type": self.shape_type}
        if self.shape_type == "box":
            shape["size"] = dict(self.dimensions)
        else:
            shape.update(self.dimensions)

        payload: dict[str, Any] = {
            "name": self.name,
            "source": self.source,
            "shape": shape,
            "pose": dict(self.pose),
        }
        if self.metadata:
            payload["metadata"] = dict(self.metadata)
        return payload


class BasicPlannerMixin:
    """A lightweight planner that reasons over axis-aligned obstacles.

    This is intentionally simple: it validates direct cartesian segments and,
    when blocked, tries a safe "lift-cruise-descend" trajectory above the
    current obstacle set. It gives us a usable Phase 5 slice now while keeping
    the MCP contract ready for a future MoveIt2-backed implementation.
    """

    def __init__(
        self,
        planner_mode: str = "none",
        planner_clearance_mm: float = 80.0,
        collision_padding_mm: float = 15.0,
        supported_planner_modes: tuple[str, ...] = ("basic",),
    ) -> None:
        self._planner_mode = planner_mode
        self._supported_planner_modes = tuple(supported_planner_modes)
        self._planner_enabled = planner_mode in self._supported_planner_modes
        self._planner_clearance_mm = float(planner_clearance_mm)
        self._collision_padding_mm = float(collision_padding_mm)
        self._collision_objects: dict[str, CollisionObject] = {}

    def planner_capabilities(self) -> dict:
        """Extra capabilities merged into the backend capabilities dict."""
        return {
            "has_planner": self._planner_enabled,
            "planner_backend": self._planner_mode if self._planner_enabled else "none",
            "planner_features": (
                [
                    "plan_and_execute",
                    "plan_cartesian_path",
                    "collision_scene",
                ]
                if self._planner_enabled
                else []
            ),
        }

    def add_collision_object(self, name: str, shape: dict, pose: dict) -> dict:
        if not self._planner_enabled:
            return {"success": False, "error": "Planner not supported by this backend"}

        try:
            obj = self._build_collision_object(
                name=name,
                shape=shape,
                pose=pose,
                source="manual",
            )
        except ValueError as exc:
            return {"success": False, "error": str(exc)}

        self._collision_objects[self._scene_key(name, "manual")] = obj
        return {
            "success": True,
            "collision_object": obj.to_dict(),
            "count": len(self._collision_objects),
        }

    def clear_collision_objects(self) -> dict:
        if not self._planner_enabled:
            return {"success": False, "error": "Planner not supported by this backend"}

        removed = len(self._collision_objects)
        self._collision_objects.clear()
        return {"success": True, "removed": removed, "count": 0}

    def sync_collision_objects(
        self,
        objects: list[dict],
        source: str = "perception",
    ) -> dict:
        if not self._planner_enabled:
            return {"success": False, "error": "Planner not supported by this backend"}

        retained = {
            key: value
            for key, value in self._collision_objects.items()
            if value.source != source
        }

        synced: list[CollisionObject] = []
        for item in objects:
            try:
                synced.append(self._build_collision_object_from_detection(item, source))
            except ValueError as exc:
                logger.warning("Skipping perception object %s: %s", item, exc)

        self._collision_objects = retained
        for obj in synced:
            self._collision_objects[self._scene_key(obj.name, source)] = obj

        return {
            "success": True,
            "source": source,
            "synced": len(synced),
            "count": len(self._collision_objects),
            "collision_objects": [obj.to_dict() for obj in self._collision_objects.values()],
        }

    def plan_and_execute(
        self,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
        speed: float,
    ) -> dict:
        if not self._planner_enabled:
            return {"success": False, "error": "Planner not supported by this backend"}
        if self._planner_mode != "basic":
            return {
                "success": False,
                "error": f"Planner mode '{self._planner_mode}' must be implemented by the backend",
            }

        try:
            target = self._normalize_pose(
                {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}
            )
        except ValueError as exc:
            return {"success": False, "error": str(exc)}

        planning_result = self._plan_to_pose(target)
        if not planning_result["success"]:
            return planning_result

        executed: list[dict[str, float]] = []
        final_position: dict | None = None
        for waypoint in planning_result["trajectory"]:
            result = self.move_cartesian(
                waypoint["x"],
                waypoint["y"],
                waypoint["z"],
                waypoint["rx"],
                waypoint["ry"],
                waypoint["rz"],
                speed,
            )
            if not result.get("success"):
                return {
                    "success": False,
                    "error": result.get("error", "Failed to execute planned waypoint"),
                    "planner": "basic",
                    "trajectory": planning_result["trajectory"],
                    "executed_waypoints": executed,
                }
            executed.append(dict(waypoint))
            final_position = result.get("final_position")

        return {
            "success": True,
            "planner": "basic",
            "trajectory": planning_result["trajectory"],
            "avoided_objects": planning_result["avoided_objects"],
            "final_position": final_position or self.get_position(),
        }

    def plan_cartesian_path(
        self,
        waypoints: list[dict],
        speed: float,
    ) -> dict:
        if not self._planner_enabled:
            return {"success": False, "error": "Planner not supported by this backend"}
        if self._planner_mode != "basic":
            return {
                "success": False,
                "error": f"Planner mode '{self._planner_mode}' must be implemented by the backend",
            }
        if not isinstance(waypoints, list) or not waypoints:
            return {"success": False, "error": "waypoints must be a non-empty list"}

        current = self._normalize_pose(self.get_position()["cartesian"])
        normalized_waypoints: list[dict[str, float]] = []
        for item in waypoints:
            try:
                waypoint = self._normalize_pose(item)
            except ValueError as exc:
                return {"success": False, "error": str(exc)}

            blockers = self._blocking_objects_for_target(waypoint)
            if blockers:
                return {
                    "success": False,
                    "error": "Waypoint intersects a collision object",
                    "blocking_objects": blockers,
                    "waypoint": waypoint,
                }

            segment_blockers = self._segment_blockers(current, waypoint)
            if segment_blockers:
                return {
                    "success": False,
                    "error": "Cartesian path segment intersects a collision object",
                    "blocking_objects": segment_blockers,
                    "waypoint": waypoint,
                }

            normalized_waypoints.append(waypoint)
            current = waypoint

        executed: list[dict[str, float]] = []
        final_position: dict | None = None
        for waypoint in normalized_waypoints:
            result = self.move_cartesian(
                waypoint["x"],
                waypoint["y"],
                waypoint["z"],
                waypoint["rx"],
                waypoint["ry"],
                waypoint["rz"],
                speed,
            )
            if not result.get("success"):
                return {
                    "success": False,
                    "error": result.get("error", "Failed to execute cartesian waypoint"),
                    "planner": "basic",
                    "trajectory": normalized_waypoints,
                    "executed_waypoints": executed,
                }
            executed.append(dict(waypoint))
            final_position = result.get("final_position")

        return {
            "success": True,
            "planner": "basic",
            "trajectory": normalized_waypoints,
            "final_position": final_position or self.get_position(),
        }

    # -- planning internals -------------------------------------------------

    def _plan_to_pose(self, target: dict[str, float]) -> dict:
        current = self._normalize_pose(self.get_position()["cartesian"])
        blockers = self._blocking_objects_for_target(target)
        if blockers:
            return {
                "success": False,
                "error": "Target pose collides with an obstacle",
                "blocking_objects": blockers,
            }

        direct_blockers = self._segment_blockers(current, target)
        if not direct_blockers:
            return {
                "success": True,
                "trajectory": [target],
                "avoided_objects": [],
            }

        safe_z = max(
            current["z"],
            target["z"],
            *[
                obj.pose["z"] + self._object_half_extents(obj)["z"] + self._planner_clearance_mm
                for obj in self._objects_by_name(direct_blockers)
            ],
        )
        lift_pose = dict(current)
        lift_pose["z"] = safe_z
        cruise_pose = dict(target)
        cruise_pose["z"] = safe_z

        lift_blockers = self._segment_blockers(current, lift_pose)
        cruise_blockers = self._segment_blockers(lift_pose, cruise_pose)
        descend_blockers = self._segment_blockers(cruise_pose, target)

        if lift_blockers or cruise_blockers or descend_blockers:
            return {
                "success": False,
                "error": "Planner could not find a safe lift-cruise-descend path",
                "blocking_objects": sorted(
                    set(lift_blockers + cruise_blockers + descend_blockers)
                ),
            }

        return {
            "success": True,
            "trajectory": [lift_pose, cruise_pose, target],
            "avoided_objects": sorted(set(direct_blockers)),
        }

    def _scene_key(self, name: str, source: str) -> str:
        return f"{source}:{name}"

    def _normalize_pose(self, pose: dict[str, Any]) -> dict[str, float]:
        if not isinstance(pose, dict):
            raise ValueError("pose must be an object with x, y, z fields")
        try:
            normalized = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose["z"]),
                "rx": float(pose.get("rx", 180.0)),
                "ry": float(pose.get("ry", 0.0)),
                "rz": float(pose.get("rz", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid pose {pose!r}: {exc}") from exc
        return normalized

    def _build_collision_object(
        self,
        name: str,
        shape: dict[str, Any],
        pose: dict[str, Any],
        source: str,
        metadata: dict[str, Any] | None = None,
    ) -> CollisionObject:
        if not name or not isinstance(name, str):
            raise ValueError("collision object name must be a non-empty string")
        shape_type, dimensions = self._normalize_shape(shape)
        return CollisionObject(
            name=name,
            shape_type=shape_type,
            dimensions=dimensions,
            pose=self._normalize_pose(pose),
            source=source,
            metadata=metadata or {},
        )

    def _build_collision_object_from_detection(
        self,
        item: dict[str, Any],
        source: str,
    ) -> CollisionObject:
        if not isinstance(item, dict):
            raise ValueError("detection must be a dict")

        metadata = dict(item.get("metadata", {}))
        object_class = str(metadata.get("class", "")).lower()
        inferred_dimensions = _SCENE_DIMENSIONS_BY_CLASS.get(
            object_class,
            _DEFAULT_BOX_SIZE_MM,
        )
        shape = {"type": "box", "size": dict(inferred_dimensions)}

        pose = dict(item.get("position", {}))
        if "orientation" in item and isinstance(item["orientation"], dict):
            pose.update(item["orientation"])

        return self._build_collision_object(
            name=str(item.get("name", "")).strip(),
            shape=shape,
            pose=pose,
            source=source,
            metadata=metadata,
        )

    def _normalize_shape(
        self,
        shape: dict[str, Any],
    ) -> tuple[str, dict[str, float]]:
        if not isinstance(shape, dict):
            raise ValueError("shape must be an object")

        shape_type = str(shape.get("type", "")).lower().strip()
        if shape_type not in _SUPPORTED_SHAPES:
            raise ValueError(
                f"unsupported shape type {shape_type!r}. Expected one of: "
                f"{', '.join(sorted(_SUPPORTED_SHAPES))}"
            )

        if shape_type == "box":
            size = shape.get("size", {})
            if not isinstance(size, dict):
                raise ValueError("box shape requires size={x,y,z}")
            try:
                return shape_type, {
                    "x": float(size["x"]),
                    "y": float(size["y"]),
                    "z": float(size["z"]),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid box size {size!r}: {exc}") from exc

        if shape_type == "cylinder":
            try:
                return shape_type, {
                    "radius": float(shape["radius"]),
                    "height": float(shape["height"]),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid cylinder shape {shape!r}: {exc}") from exc

        try:
            return shape_type, {"radius": float(shape["radius"])}
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid sphere shape {shape!r}: {exc}") from exc

    def _objects_by_name(self, names: list[str]) -> list[CollisionObject]:
        selected: list[CollisionObject] = []
        for obj in self._collision_objects.values():
            if obj.name in names:
                selected.append(obj)
        return selected

    def _object_half_extents(self, obj: CollisionObject) -> dict[str, float]:
        if obj.shape_type == "box":
            return {
                "x": obj.dimensions["x"] / 2.0 + self._collision_padding_mm,
                "y": obj.dimensions["y"] / 2.0 + self._collision_padding_mm,
                "z": obj.dimensions["z"] / 2.0 + self._collision_padding_mm,
            }
        if obj.shape_type == "cylinder":
            radius = obj.dimensions["radius"] + self._collision_padding_mm
            return {
                "x": radius,
                "y": radius,
                "z": obj.dimensions["height"] / 2.0 + self._collision_padding_mm,
            }
        radius = obj.dimensions["radius"] + self._collision_padding_mm
        return {"x": radius, "y": radius, "z": radius}

    def _aabb(self, obj: CollisionObject) -> dict[str, tuple[float, float]]:
        extents = self._object_half_extents(obj)
        return {
            axis: (obj.pose[axis] - extents[axis], obj.pose[axis] + extents[axis])
            for axis in ("x", "y", "z")
        }

    def _blocking_objects_for_target(self, target: dict[str, float]) -> list[str]:
        blockers: list[str] = []
        for obj in self._collision_objects.values():
            bounds = self._aabb(obj)
            if (
                bounds["x"][0] <= target["x"] <= bounds["x"][1]
                and bounds["y"][0] <= target["y"] <= bounds["y"][1]
                and bounds["z"][0] <= target["z"] <= bounds["z"][1]
            ):
                blockers.append(obj.name)
        return sorted(set(blockers))

    def _segment_blockers(
        self,
        start: dict[str, float],
        end: dict[str, float],
    ) -> list[str]:
        blockers: list[str] = []
        for obj in self._collision_objects.values():
            if self._segment_intersects_aabb(start, end, self._aabb(obj)):
                blockers.append(obj.name)
        return sorted(set(blockers))

    @staticmethod
    def _segment_intersects_aabb(
        start: dict[str, float],
        end: dict[str, float],
        bounds: dict[str, tuple[float, float]],
    ) -> bool:
        t_min = 0.0
        t_max = 1.0
        for axis in ("x", "y", "z"):
            delta = end[axis] - start[axis]
            axis_min, axis_max = bounds[axis]
            if abs(delta) < 1e-9:
                if start[axis] < axis_min or start[axis] > axis_max:
                    return False
                continue

            inv_delta = 1.0 / delta
            t1 = (axis_min - start[axis]) * inv_delta
            t2 = (axis_max - start[axis]) * inv_delta
            t_low = min(t1, t2)
            t_high = max(t1, t2)
            t_min = max(t_min, t_low)
            t_max = min(t_max, t_high)
            if t_min > t_max:
                return False
        return True
