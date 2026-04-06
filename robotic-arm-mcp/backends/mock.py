"""Mock backend for development and testing without hardware."""

import logging
import time

from backends.base import ArmBackend
from backends.planning import BasicPlannerMixin

logger = logging.getLogger(__name__)

# myCobot 280 Pi specifications
_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
_JOINT_LIMITS = {
    "j1": [-165.0, 165.0],
    "j2": [-165.0, 165.0],
    "j3": [-165.0, 165.0],
    "j4": [-165.0, 165.0],
    "j5": [-165.0, 165.0],
    "j6": [-175.0, 175.0],
}
_HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_HOME_CARTESIAN = {"x": 156.5, "y": 0.0, "z": 253.2, "rx": 180.0, "ry": 0.0, "rz": 0.0}


class MockBackend(BasicPlannerMixin, ArmBackend):
    """Simulated arm backend that keeps state in memory.

    Useful for developing and testing the agent without a physical robot.
    """

    def __init__(
        self,
        planner_mode: str = "none",
        planner_clearance_mm: float = 80.0,
        collision_padding_mm: float = 15.0,
    ) -> None:
        super().__init__(
            planner_mode=planner_mode,
            planner_clearance_mm=planner_clearance_mm,
            collision_padding_mm=collision_padding_mm,
            supported_planner_modes=("basic",),
        )
        self._joints: list[float] = list(_HOME_ANGLES)
        self._cartesian: dict[str, float] = dict(_HOME_CARTESIAN)
        self._gripper_state: str = "open"
        self._stopped: bool = False
        logger.info("MockBackend initialized at home position")

    # -- helpers -------------------------------------------------------------

    def _build_position(self) -> dict:
        return {
            "joints": list(self._joints),
            "cartesian": dict(self._cartesian),
        }

    def _simulate_move(self, speed: float) -> None:
        """Simulate movement delay proportional to speed."""
        delay = max(0.05, 0.5 * (1 - speed / 100))
        time.sleep(delay)

    # -- required methods ----------------------------------------------------

    def get_capabilities(self) -> dict:
        capabilities = {
            "name": "myCobot 280 Pi (mock)",
            "dof": 6,
            "joint_names": list(_JOINT_NAMES),
            "joint_limits": dict(_JOINT_LIMITS),
            "max_payload_g": 250,
            "reach_mm": 280,
            "has_gripper": True,
            "has_planner": False,
            "has_camera": False,
            "has_freedrive": False,
            "backend": "mock",
        }
        capabilities.update(self.planner_capabilities())
        return capabilities

    def get_position(self) -> dict:
        return self._build_position()

    def move_joints(self, angles: list[float], speed: float) -> dict:
        if self._stopped:
            return {"success": False, "error": "E-stop active. Call emergency_stop to clear, then retry."}

        logger.info("MOCK move_joints: angles=%s speed=%s", angles, speed)
        self._simulate_move(speed)
        self._joints = list(angles)
        # Simplified forward kinematics — just shift cartesian a bit
        self._cartesian["x"] = _HOME_CARTESIAN["x"] + angles[0] * 0.5
        self._cartesian["z"] = _HOME_CARTESIAN["z"] + angles[1] * 0.3
        return {"success": True, "final_position": self._build_position()}

    def move_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
        speed: float,
    ) -> dict:
        if self._stopped:
            return {"success": False, "error": "E-stop active. Call emergency_stop to clear, then retry."}

        logger.info("MOCK move_cartesian: (%s,%s,%s,%s,%s,%s) speed=%s", x, y, z, rx, ry, rz, speed)
        self._simulate_move(speed)
        self._cartesian = {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}
        return {"success": True, "final_position": self._build_position()}

    def go_home(self, speed: float) -> dict:
        logger.info("MOCK go_home speed=%s", speed)
        self._stopped = False
        self._simulate_move(speed)
        self._joints = list(_HOME_ANGLES)
        self._cartesian = dict(_HOME_CARTESIAN)
        return {"success": True}

    def emergency_stop(self) -> dict:
        logger.info("MOCK emergency_stop")
        self._stopped = not self._stopped
        state = "activated" if self._stopped else "cleared"
        return {"success": True, "state": state}

    # -- optional ------------------------------------------------------------

    def gripper(self, action: str) -> dict:
        logger.info("MOCK gripper: %s", action)
        if action in ("open", "close"):
            self._gripper_state = action
        elif action.isdigit() and 0 <= int(action) <= 100:
            self._gripper_state = f"{action}%"
        else:
            return {"success": False, "error": f"Invalid gripper action: {action}"}
        return {"success": True, "state": self._gripper_state}
