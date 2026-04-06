"""Abstract base class for robot arm backends."""

from abc import ABC, abstractmethod


class ArmBackend(ABC):
    """Interface that every robot arm backend must implement.

    The MCP server delegates all hardware interaction to a backend.
    Implement this class to support a new arm (ROS 2, pymycobot, serial, etc.).
    """

    @abstractmethod
    def get_capabilities(self) -> dict:
        """Return a description of the arm's capabilities.

        Must include at minimum:
            name, dof, joint_names, joint_limits, max_payload_g,
            reach_mm, has_gripper, has_planner, has_camera, has_freedrive,
            backend.
        """

    @abstractmethod
    def get_position(self) -> dict:
        """Return the current arm position.

        Returns:
            {
                "joints": [float, ...],          # current joint angles (degrees)
                "cartesian": {
                    "x": float, "y": float, "z": float,
                    "rx": float, "ry": float, "rz": float,
                },
            }
        """

    @abstractmethod
    def move_joints(self, angles: list[float], speed: float) -> dict:
        """Move to the given joint angles.

        Args:
            angles: Target angle for each joint (degrees).
            speed: Movement speed (0-100).

        Returns:
            {"success": bool, "final_position": <same shape as get_position>}
        """

    @abstractmethod
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
        """Move to the given cartesian pose.

        Args:
            x, y, z: Position in mm.
            rx, ry, rz: Orientation in degrees.
            speed: Movement speed (0-100).

        Returns:
            {"success": bool, "final_position": <same shape as get_position>}
        """

    @abstractmethod
    def go_home(self, speed: float) -> dict:
        """Move the arm to its home/zero position.

        Returns:
            {"success": bool}
        """

    @abstractmethod
    def emergency_stop(self) -> dict:
        """Immediately stop all movement.

        Returns:
            {"success": bool}
        """

    # -- Optional capabilities (override when supported) ---------------------

    def gripper(self, action: str) -> dict:
        """Control the gripper.

        Args:
            action: "open", "close", or a string percentage "0"-"100".

        Returns:
            {"success": bool, "state": str}
        """
        return {"success": False, "error": "Gripper not supported by this backend"}

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
        """Plan a collision-aware path to the target pose and execute it."""
        return {"success": False, "error": "Planner not supported by this backend"}

    def plan_cartesian_path(
        self,
        waypoints: list[dict],
        speed: float,
    ) -> dict:
        """Execute a straight-line multi-waypoint cartesian path."""
        return {"success": False, "error": "Planner not supported by this backend"}

    def add_collision_object(self, name: str, shape: dict, pose: dict) -> dict:
        """Add an obstacle to the planning scene."""
        return {"success": False, "error": "Planner not supported by this backend"}

    def clear_collision_objects(self) -> dict:
        """Remove all obstacles from the planning scene."""
        return {"success": False, "error": "Planner not supported by this backend"}

    def sync_collision_objects(
        self,
        objects: list[dict],
        source: str = "perception",
    ) -> dict:
        """Replace planner obstacles from a dynamic source such as perception."""
        return {"success": False, "error": "Planner not supported by this backend"}
