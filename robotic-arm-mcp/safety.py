"""Safety layer for the MCP server.

Validates all commands BEFORE they reach the backend. The robot protects itself
regardless of what the agent sends.
"""

import logging
import time

logger = logging.getLogger(__name__)


class SafetyError(Exception):
    """Raised when a command violates safety constraints."""


class SafetyLayer:
    """Validates movements against arm capabilities and safety profiles.

    Args:
        capabilities: The dict returned by backend.get_capabilities().
        profile: Safety profile name — one of "testing", "normal", "production".
    """

    PROFILES: dict[str, dict] = {
        "testing": {"max_speed": 15, "limit_margin_deg": 15, "rate_limit_hz": 2},
        "normal": {"max_speed": 50, "limit_margin_deg": 10, "rate_limit_hz": 5},
        "production": {"max_speed": 80, "limit_margin_deg": 5, "rate_limit_hz": 10},
    }

    def __init__(self, capabilities: dict, profile: str = "testing") -> None:
        if profile not in self.PROFILES:
            raise ValueError(f"Unknown safety profile: {profile!r}. Options: {list(self.PROFILES)}")

        self._caps = capabilities
        self._profile = self.PROFILES[profile]
        self._profile_name = profile
        self._last_command_time: float = 0.0

        logger.info(
            "SafetyLayer initialized: profile=%s, max_speed=%s, margin=%s deg",
            profile,
            self._profile["max_speed"],
            self._profile["limit_margin_deg"],
        )

    # -- public API ----------------------------------------------------------

    def validate_joints(self, angles: list[float]) -> None:
        """Raise SafetyError if any angle is outside safe limits."""
        joint_limits = self._caps.get("joint_limits", {})
        joint_names = self._caps.get("joint_names", [])
        margin = self._profile["limit_margin_deg"]

        if len(angles) != self._caps.get("dof", 0):
            raise SafetyError(
                f"Expected {self._caps['dof']} joint angles, got {len(angles)}"
            )

        for i, angle in enumerate(angles):
            name = joint_names[i] if i < len(joint_names) else f"j{i+1}"
            limits = joint_limits.get(name)
            if limits is None:
                continue
            safe_min = limits[0] + margin
            safe_max = limits[1] - margin
            if not (safe_min <= angle <= safe_max):
                raise SafetyError(
                    f"Joint {name}: {angle:.1f} deg outside safe range "
                    f"[{safe_min:.1f}, {safe_max:.1f}] "
                    f"(hard limits [{limits[0]}, {limits[1]}], margin {margin} deg)"
                )

    def clamp_speed(self, speed: float) -> float:
        """Clamp speed to the profile maximum. Returns the clamped value."""
        max_speed = self._profile["max_speed"]
        if speed > max_speed:
            logger.warning("Speed %.1f clamped to %.1f (profile: %s)", speed, max_speed, self._profile_name)
            return max_speed
        if speed <= 0:
            raise SafetyError("Speed must be > 0")
        return speed

    def validate_cartesian(self, x: float, y: float, z: float) -> None:
        """Raise SafetyError if position is outside workspace bounds."""
        reach = self._caps.get("reach_mm", float("inf"))
        dist_xy = (x**2 + y**2) ** 0.5
        if dist_xy > reach:
            raise SafetyError(
                f"Position ({x:.1f}, {y:.1f}) is {dist_xy:.1f}mm from base, "
                f"exceeds reach of {reach}mm"
            )
        if z < 0:
            raise SafetyError(f"Z={z:.1f}mm is below the base plane")

    def check_rate_limit(self) -> None:
        """Raise SafetyError if commands are arriving too fast."""
        now = time.monotonic()
        min_interval = 1.0 / self._profile["rate_limit_hz"]
        elapsed = now - self._last_command_time
        if elapsed < min_interval:
            raise SafetyError(
                f"Rate limit: {elapsed:.2f}s since last command, "
                f"minimum interval is {min_interval:.2f}s "
                f"({self._profile['rate_limit_hz']} Hz)"
            )
        self._last_command_time = now
