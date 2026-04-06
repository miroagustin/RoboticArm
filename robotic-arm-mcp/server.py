"""MCP server for generic robot arm control.

Exposes a hardware-agnostic tool contract via the Model Context Protocol.
The active backend (mock, ROS 2, pymycobot, ...) is selected at startup.

Run with:
    python server.py                     # default: mock backend, testing profile
    python server.py --backend mock      # explicit
    MCP_BACKEND=ros2 python server.py    # via env var
"""

import argparse
import hmac
import logging
import os
import sys

# Ensure imports resolve correctly regardless of working directory
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from mcp.server.fastmcp import FastMCP

import config
from backends import MockBackend
from backends.base import ArmBackend
from perception import MockPerception, PerceptionBackend, scan_payload
from safety import SafetyError, SafetyLayer
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse, Response
import uvicorn

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


class MCPAPIKeyMiddleware(BaseHTTPMiddleware):
    """Protect MCP requests with a static bearer token while keeping health open."""

    def __init__(self, app, expected_token: str, protected_path: str = "/mcp"):
        super().__init__(app)
        self._expected_token = expected_token
        self._protected_path = protected_path.rstrip("/") or "/"

    async def dispatch(self, request, call_next):
        if request.url.path.startswith(self._protected_path):
            token = request.headers.get("authorization", "").removeprefix("Bearer ").strip()
            if not hmac.compare_digest(token, self._expected_token):
                return Response(status_code=401, content="Unauthorized")
        return await call_next(request)

# ---------------------------------------------------------------------------
# Backend factory
# ---------------------------------------------------------------------------


def _create_backend(
    name: str,
    planner: str,
    planner_clearance_mm: float,
    collision_padding_mm: float,
) -> ArmBackend:
    """Instantiate the requested backend."""
    if name == "mock":
        if planner == "moveit2":
            raise ValueError("Planner 'moveit2' is only supported with the ROS2 backend")
        return MockBackend(
            planner_mode=planner,
            planner_clearance_mm=planner_clearance_mm,
            collision_padding_mm=collision_padding_mm,
        )
    if name == "ros2":
        from backends.ros2 import ROS2Backend
        return ROS2Backend(
            planner_mode=planner,
            planner_clearance_mm=planner_clearance_mm,
            collision_padding_mm=collision_padding_mm,
        )
    raise ValueError(f"Unknown backend: {name!r}. Available: mock, ros2")


def _create_perception(
    name: str,
    calibration_path: str,
    camera_device: str,
    mock_scene_file: str,
) -> PerceptionBackend | None:
    """Instantiate the requested perception backend, or None when disabled."""
    if not name or name == "none":
        return None
    if name == "mock":
        return MockPerception(scene_file=mock_scene_file or None)
    if name == "aruco":
        if not calibration_path:
            raise ValueError(
                "Aruco perception requires --perception-calibration "
                "(or MCP_PERCEPTION_CALIBRATION)."
            )
        from perception.aruco import ArucoPerception

        # Allow integer V4L indices ("0", "1", ...) or arbitrary source strings.
        device: int | str
        try:
            device = int(camera_device)
        except (TypeError, ValueError):
            device = camera_device
        return ArucoPerception(config_path=calibration_path, camera_device=device)
    raise ValueError(
        f"Unknown perception backend: {name!r}. Available: none, mock, aruco"
    )


# ---------------------------------------------------------------------------
# Parse CLI args (override env vars)
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Robot arm MCP server")
    p.add_argument("--backend", default=config.BACKEND, help="Backend to use (default: %(default)s)")
    p.add_argument("--host", default=config.MCP_HOST, help="Bind host (default: %(default)s)")
    p.add_argument("--port", type=int, default=config.MCP_PORT, help="Bind port (default: %(default)s)")
    p.add_argument("--safety", default=config.SAFETY_PROFILE, help="Safety profile (default: %(default)s)")
    p.add_argument("--api-key", default=config.MCP_API_KEY, help="API key for auth (default: from env)")
    p.add_argument(
        "--planner",
        default=config.PLANNER,
        choices=["none", "basic", "moveit2"],
        help="Planner mode (default: %(default)s)",
    )
    p.add_argument(
        "--planner-sync-perception",
        default=config.PLANNER_SYNC_PERCEPTION,
        type=lambda value: str(value).strip().lower() in {"1", "true", "yes", "on"},
        help="Sync perception detections into the collision scene automatically (default: %(default)s)",
    )
    p.add_argument(
        "--planner-clearance-mm",
        default=config.PLANNER_CLEARANCE_MM,
        type=float,
        help="Vertical clearance used by the basic planner (default: %(default)s)",
    )
    p.add_argument(
        "--collision-padding-mm",
        default=config.COLLISION_PADDING_MM,
        type=float,
        help="Extra padding around collision objects (default: %(default)s)",
    )
    p.add_argument(
        "--perception",
        default=config.PERCEPTION,
        choices=["none", "mock", "aruco"],
        help="Perception backend (default: %(default)s)",
    )
    p.add_argument(
        "--perception-calibration",
        default=config.PERCEPTION_CALIBRATION,
        help="Path to aruco calibration JSON (required for --perception aruco)",
    )
    p.add_argument(
        "--perception-camera",
        default=config.PERCEPTION_CAMERA,
        help="Camera device for aruco (V4L index or source string, default: %(default)s)",
    )
    p.add_argument(
        "--perception-mock-file",
        default=config.PERCEPTION_MOCK_FILE,
        help="Optional JSON file with fake objects for --perception mock",
    )
    return p.parse_args()


args = _parse_args()

# ---------------------------------------------------------------------------
# Instantiate backend + safety
# ---------------------------------------------------------------------------

backend = _create_backend(
    args.backend,
    planner=args.planner,
    planner_clearance_mm=args.planner_clearance_mm,
    collision_padding_mm=args.collision_padding_mm,
)
capabilities = backend.get_capabilities()
safety = SafetyLayer(capabilities, profile=args.safety)

perception: PerceptionBackend | None = _create_perception(
    args.perception,
    args.perception_calibration,
    args.perception_camera,
    args.perception_mock_file,
)
if perception is not None:
    capabilities = dict(capabilities)
    capabilities["has_camera"] = True
    capabilities.update(perception.capabilities())

logger.info("Backend: %s (%s)", capabilities["name"], args.backend)
logger.info(
    "DOF: %s | Gripper: %s | Planner: %s (%s) | Camera: %s",
    capabilities["dof"],
    capabilities["has_gripper"],
    capabilities["has_planner"],
    capabilities.get("planner_backend", "none"),
    capabilities.get("has_camera", False),
)

# ---------------------------------------------------------------------------
# MCP server
# ---------------------------------------------------------------------------

mcp = FastMCP(
    name="RobotArmMCP",
    host=args.host,
    port=args.port,
    stateless_http=True,
)

# ---------------------------------------------------------------------------
# Auth middleware (if API key is set)
# ---------------------------------------------------------------------------

if args.api_key:
    logger.info("API key authentication enabled")
else:
    logger.warning("No API key set — MCP server is open (dev mode)")


@mcp.custom_route("/healthz", methods=["GET"], include_in_schema=False)
async def healthcheck(_request):
    """Liveness/readiness probe for container orchestration."""
    payload = {
        "status": "ok",
        "backend": args.backend,
        "planner": args.planner,
        "robot": capabilities["name"],
        "safety_profile": args.safety,
    }
    status_code = 200

    try:
        backend.get_position()
    except Exception as exc:
        payload["status"] = "degraded"
        payload["error"] = str(exc)
        status_code = 503

    return JSONResponse(payload, status_code=status_code)


# ---------------------------------------------------------------------------
# Tool: get_capabilities
# ---------------------------------------------------------------------------


@mcp.tool()
def get_capabilities() -> dict:
    """Get the robot arm's capabilities.

    Returns the arm's name, degrees of freedom, joint names, joint limits,
    payload, reach, and supported features (gripper, planner, camera, etc.).
    Call this once at startup to understand what the arm can do.
    """
    return capabilities


# ---------------------------------------------------------------------------
# Tool: get_position
# ---------------------------------------------------------------------------


@mcp.tool()
def get_position() -> dict:
    """Get the current position of the robot arm.

    Returns joint angles (degrees) and cartesian pose (mm, degrees).
    """
    return backend.get_position()


def _sync_collision_scene_from_perception() -> dict | None:
    """Refresh dynamic collision objects from the active perception backend."""
    if perception is None or not capabilities.get("has_planner") or not args.planner_sync_perception:
        return None

    objects = perception.scan()
    result = backend.sync_collision_objects(
        [obj.to_dict() for obj in objects],
        source=args.perception,
    )
    if not result.get("success"):
        logger.warning("Planner scene sync from perception failed: %s", result)
    return result


# ---------------------------------------------------------------------------
# Tool: move_joints
# ---------------------------------------------------------------------------


@mcp.tool()
def move_joints(angles: list[float], speed: float = 30) -> dict:
    """Move the robot arm to the given joint angles.

    Args:
        angles: Target angle for each joint in degrees.
                Must have exactly as many values as the arm has DOF.
        speed:  Movement speed from 0 to 100 (will be clamped to safety max).

    Returns:
        {"success": bool, "final_position": ...} or {"success": false, "error": ...}
    """
    try:
        safety.check_rate_limit()
        safety.validate_joints(angles)
        speed = safety.clamp_speed(speed)
    except SafetyError as e:
        return {"success": False, "error": str(e)}

    return backend.move_joints(angles, speed)


# ---------------------------------------------------------------------------
# Tool: move_cartesian
# ---------------------------------------------------------------------------


@mcp.tool()
def move_cartesian(
    x: float,
    y: float,
    z: float,
    rx: float = 180.0,
    ry: float = 0.0,
    rz: float = 0.0,
    speed: float = 30,
) -> dict:
    """Move the robot arm to a cartesian pose.

    Args:
        x:  X position in mm (forward/back from base).
        y:  Y position in mm (left/right from base).
        z:  Z position in mm (up/down from base).
        rx: Roll in degrees (default 180 = gripper pointing down).
        ry: Pitch in degrees.
        rz: Yaw in degrees.
        speed: Movement speed 0-100 (clamped to safety max).

    Returns:
        {"success": bool, "final_position": ...} or {"success": false, "error": ...}
    """
    try:
        safety.check_rate_limit()
        safety.validate_cartesian(x, y, z)
        speed = safety.clamp_speed(speed)
    except SafetyError as e:
        return {"success": False, "error": str(e)}

    return backend.move_cartesian(x, y, z, rx, ry, rz, speed)


# ---------------------------------------------------------------------------
# Tool: go_home
# ---------------------------------------------------------------------------


@mcp.tool()
def go_home(speed: float = 20) -> dict:
    """Move the robot arm to its safe home/zero position.

    Args:
        speed: Movement speed 0-100 (clamped to safety max).

    Returns:
        {"success": bool}
    """
    try:
        speed = safety.clamp_speed(speed)
    except SafetyError as e:
        return {"success": False, "error": str(e)}

    return backend.go_home(speed)


# ---------------------------------------------------------------------------
# Tool: emergency_stop
# ---------------------------------------------------------------------------


@mcp.tool()
def emergency_stop() -> dict:
    """Immediately stop all robot movement.

    Toggles emergency stop: first call activates, second call clears.
    No parameters needed — call this whenever something goes wrong.

    Returns:
        {"success": bool, "state": "activated" | "cleared"}
    """
    return backend.emergency_stop()


# ---------------------------------------------------------------------------
# Tool: gripper (conditional on capabilities)
# ---------------------------------------------------------------------------

if capabilities.get("has_gripper"):

    @mcp.tool()
    def gripper(action: str) -> dict:
        """Control the robot gripper.

        Args:
            action: "open", "close", or a percentage "0"-"100" for partial grip.

        Returns:
            {"success": bool, "state": str}
        """
        return backend.gripper(action)


# ---------------------------------------------------------------------------
# Planner tools (conditional on planner capability)
# ---------------------------------------------------------------------------

if capabilities.get("has_planner"):

    @mcp.tool()
    def plan_and_execute(
        x: float,
        y: float,
        z: float,
        rx: float = 180.0,
        ry: float = 0.0,
        rz: float = 0.0,
        speed: float = 20.0,
        sync_with_perception: bool = True,
    ) -> dict:
        """Plan a collision-aware path to the target pose and execute it.

        When perception is enabled, the collision scene can be refreshed
        automatically before planning so recently detected objects are avoided.
        """
        try:
            safety.check_rate_limit()
            safety.validate_cartesian(x, y, z)
            speed = safety.clamp_speed(speed)
        except SafetyError as e:
            return {"success": False, "error": str(e)}

        scene_sync = None
        if sync_with_perception:
            try:
                scene_sync = _sync_collision_scene_from_perception()
            except Exception as exc:
                logger.exception("Collision scene sync failed before planning: %s", exc)
                return {"success": False, "error": f"Failed to sync collision scene: {exc}"}

        result = backend.plan_and_execute(x, y, z, rx, ry, rz, speed)
        if scene_sync is not None:
            result["scene_sync"] = scene_sync
        return result

    @mcp.tool()
    def plan_cartesian_path(
        waypoints: list[dict],
        speed: float = 20.0,
        sync_with_perception: bool = False,
    ) -> dict:
        """Execute a straight-line cartesian path through the given waypoints."""
        if not isinstance(waypoints, list) or not waypoints:
            return {"success": False, "error": "waypoints must be a non-empty list"}

        try:
            safety.check_rate_limit()
            speed = safety.clamp_speed(speed)
            for waypoint in waypoints:
                safety.validate_cartesian(
                    float(waypoint["x"]),
                    float(waypoint["y"]),
                    float(waypoint["z"]),
                )
        except (KeyError, TypeError, ValueError) as exc:
            return {"success": False, "error": f"invalid waypoint payload: {exc}"}
        except SafetyError as exc:
            return {"success": False, "error": str(exc)}

        scene_sync = None
        if sync_with_perception:
            try:
                scene_sync = _sync_collision_scene_from_perception()
            except Exception as exc:
                logger.exception("Collision scene sync failed before cartesian planning: %s", exc)
                return {"success": False, "error": f"Failed to sync collision scene: {exc}"}

        result = backend.plan_cartesian_path(waypoints, speed)
        if scene_sync is not None:
            result["scene_sync"] = scene_sync
        return result

    @mcp.tool()
    def add_collision_object(name: str, shape: dict, pose: dict) -> dict:
        """Add an obstacle to the planning scene.

        Example shape payloads:
        - {"type": "box", "size": {"x": 80, "y": 60, "z": 120}}
        - {"type": "cylinder", "radius": 30, "height": 100}
        - {"type": "sphere", "radius": 40}
        """
        return backend.add_collision_object(name, shape, pose)

    @mcp.tool()
    def clear_collision_objects() -> dict:
        """Clear the planning scene obstacle set."""
        return backend.clear_collision_objects()


# ---------------------------------------------------------------------------
# Tool: scan_workspace (conditional on perception backend)
# ---------------------------------------------------------------------------

if perception is not None:

    @mcp.tool()
    def scan_workspace() -> dict:
        """Detect objects in the robot workspace and return their positions.

        Uses the active perception backend (mock or aruco) to produce a list
        of detected objects expressed in the robot base frame.

        Returns:
            {
                "objects": [
                    {
                        "name": str,                  # object label
                        "position": {"x","y","z"},    # mm, robot base frame
                        "confidence": float,          # 0-1
                        "source": str,                # "aruco" | "mock" | ...
                        # optional:
                        "orientation": {"rx","ry","rz"},
                        "metadata": {...}
                    },
                    ...
                ],
                "count": int,
                "backend": str,
                "timestamp": "YYYY-MM-DDTHH:MM:SSZ"
            }
        """
        try:
            objects = perception.scan()
        except Exception as exc:  # surface perception errors to the agent
            logger.exception("scan_workspace failed: %s", exc)
            return {
                "objects": [],
                "count": 0,
                "backend": args.perception,
                "error": str(exc),
            }
        payload = scan_payload(objects, backend=args.perception)
        if capabilities.get("has_planner") and args.planner_sync_perception:
            sync_result = backend.sync_collision_objects(
                [obj.to_dict() for obj in objects],
                source=args.perception,
            )
            payload["scene_sync"] = sync_result
        return payload


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"Robot Arm MCP Server")
    print(f"  Backend:     {capabilities['name']} ({args.backend})")
    print(f"  Address:     http://{args.host}:{args.port}/mcp")
    print(f"  Safety:      {args.safety}")
    print(f"  Auth:        {'enabled' if args.api_key else 'disabled (dev mode)'}")
    print(f"  Planner:     {args.planner}")
    print(f"  Perception:  {args.perception}")
    print()

    app = mcp.streamable_http_app()
    if args.api_key:
        app.add_middleware(
            MCPAPIKeyMiddleware,
            expected_token=args.api_key,
            protected_path="/mcp",
        )

    uvicorn.run(app, host=args.host, port=args.port)
