"""MCP server configuration.

All values can be overridden via environment variables or CLI flags.
"""

import os

# -- Network ----------------------------------------------------------------
MCP_HOST: str = os.environ.get("MCP_HOST", "0.0.0.0")
MCP_PORT: int = int(os.environ.get("MCP_PORT", "8010"))

# -- Authentication ---------------------------------------------------------
# Set MCP_API_KEY to require Bearer token auth. Empty = no auth (dev only).
MCP_API_KEY: str = os.environ.get("MCP_API_KEY", "")

# -- Backend ----------------------------------------------------------------
# Options: "mock", "ros2" (future), "pymycobot" (future)
BACKEND: str = os.environ.get("MCP_BACKEND", "mock")

# -- Safety -----------------------------------------------------------------
# Options: "testing", "normal", "production"
SAFETY_PROFILE: str = os.environ.get("MCP_SAFETY_PROFILE", "testing")

# -- Planning ---------------------------------------------------------------
# Options: "none", "basic", "moveit2"
PLANNER: str = os.environ.get("MCP_PLANNER", "none")
PLANNER_SYNC_PERCEPTION: bool = os.environ.get(
    "MCP_PLANNER_SYNC_PERCEPTION",
    "true",
).strip().lower() in {"1", "true", "yes", "on"}
PLANNER_CLEARANCE_MM: float = float(
    os.environ.get("MCP_PLANNER_CLEARANCE_MM", "80")
)
COLLISION_PADDING_MM: float = float(
    os.environ.get("MCP_COLLISION_PADDING_MM", "15")
)

# -- Perception -------------------------------------------------------------
# Options: "none" (disabled), "mock", "aruco"
PERCEPTION: str = os.environ.get("MCP_PERCEPTION", "none")

# Path to the Aruco calibration JSON (see perception/calibration.example.json)
PERCEPTION_CALIBRATION: str = os.environ.get("MCP_PERCEPTION_CALIBRATION", "")

# Camera source for the aruco backend — int index for V4L (e.g. "0") or a
# GStreamer/ffmpeg source string.
PERCEPTION_CAMERA: str = os.environ.get("MCP_PERCEPTION_CAMERA", "0")

# Optional JSON file listing fake objects for the mock perception backend.
PERCEPTION_MOCK_FILE: str = os.environ.get("MCP_PERCEPTION_MOCK_FILE", "")
