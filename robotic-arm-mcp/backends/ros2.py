"""ROS 2 backend for myCobot 280 Pi.

Connects to the ROS 2 node (either sim_driver_node or listen_real_service)
via the mycobot_interfaces service contract.

Requires:
  - ROS 2 Jazzy sourced in the environment
  - mycobot_interfaces built and sourced
  - A running node advertising the services (sim or real)
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from backends.base import ArmBackend
from backends.planning import BasicPlannerMixin, CollisionObject

logger = logging.getLogger(__name__)

_JOINT_NAMES_ROS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
_JOINT_NAMES_CAPABILITIES = ["j1", "j2", "j3", "j4", "j5", "j6"]
_HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_HOME_CARTESIAN = {"x": 156.5, "y": 0.0, "z": 253.2, "rx": 180.0, "ry": 0.0, "rz": 0.0}
_SERVICE_TIMEOUT = 5.0  # seconds

_MOVEIT_GROUP = "arm_group"
_MOVEIT_PACKAGE = "mycobot_280_moveit2"
_MOVEIT_POSE_LINK = "joint6_flange"
_MOVEIT_PLANNING_FRAME = "g_base"
_MOVEIT_ROBOT_NAME = "firefighter"
_MOVEIT_SUCCESS_CODE = 1
_MOVEIT_MAX_EXECUTION_WAYPOINTS = 8
_MOVEIT_START_STATE_EPSILON_DEG = 0.5
_MOVEIT_RESPONSE_ADAPTERS = [
    "default_planning_response_adapters/ValidateSolution",
    "default_planning_response_adapters/DisplayMotionPath",
]


class ROS2Backend(BasicPlannerMixin, ArmBackend):
    """Delegates all arm commands to a ROS 2 node via mycobot_interfaces services.

    The node can be sim_driver_node (simulation) or listen_real_service (hardware).
    The MCP server doesn't care which one is running.

    Usage:
        backend = ROS2Backend()   # spins rclpy in background thread
    """

    def __init__(
        self,
        node_name: str = "mcp_arm_client",
        planner_mode: str = "none",
        planner_clearance_mm: float = 80.0,
        collision_padding_mm: float = 15.0,
    ) -> None:
        super().__init__(
            planner_mode=planner_mode,
            planner_clearance_mm=planner_clearance_mm,
            collision_padding_mm=collision_padding_mm,
            supported_planner_modes=("basic", "moveit2"),
        )
        try:
            import rclpy
            from mycobot_interfaces.srv import (
                GetAngles,
                GetCoords,
                GripperStatus,
                SetAngles,
                SetCoords,
            )
        except ImportError as e:
            raise ImportError(
                f"ROS 2 backend requires rclpy and mycobot_interfaces: {e}\n"
                "Source your ROS 2 workspace: source ~/mycobot_ws/install/setup.bash"
            ) from e

        if not rclpy.ok():
            rclpy.init()

        self._rclpy = rclpy
        self.node = rclpy.create_node(node_name)

        self._SetAngles = SetAngles
        self._GetAngles = GetAngles
        self._SetCoords = SetCoords
        self._GetCoords = GetCoords
        self._GripperStatus = GripperStatus

        self._cli_set_angles = self.node.create_client(SetAngles, "set_angles")
        self._cli_get_angles = self.node.create_client(GetAngles, "get_angles")
        self._cli_set_coords = self.node.create_client(SetCoords, "set_coords")
        self._cli_get_coords = self.node.create_client(GetCoords, "get_coords")
        self._cli_gripper = self.node.create_client(GripperStatus, "set_gripper")

        self._stopped = False

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="rclpy_spin"
        )
        self._spin_thread.start()

        self._moveit_lock = threading.RLock()
        self._moveit: Any | None = None
        self._moveit_planning_component: Any | None = None
        self._moveit_planning_scene_monitor: Any | None = None
        self._moveit_robot_model: Any | None = None
        self._MoveItPy: Any | None = None
        self._MoveItRobotState: Any | None = None
        self._MoveItCollisionObject: Any | None = None
        self._MoveItSolidPrimitive: Any | None = None
        self._MoveItPoseStamped: Any | None = None

        if self._planner_mode == "moveit2":
            self._ensure_moveit_runtime()

        logger.info("ROS2Backend ready (node: %s, planner: %s)", node_name, planner_mode)

    # -- internal helpers ----------------------------------------------------

    def _call(self, client, request):
        """Send a service request and block until the response arrives."""
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=_SERVICE_TIMEOUT):
                raise RuntimeError(
                    f"Service '{client.srv_name}' not available after {_SERVICE_TIMEOUT}s. "
                    "Is the driver node running?"
                )
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=_SERVICE_TIMEOUT):
            raise RuntimeError(
                f"Service call to '{client.srv_name}' timed out after {_SERVICE_TIMEOUT}s"
            )
        if future.exception():
            raise future.exception()
        return future.result()

    def _build_position(self, angles: list[float], coords: dict) -> dict:
        return {
            "joints": angles,
            "cartesian": coords,
        }

    def _moveit_config(self) -> dict[str, Any]:
        from moveit_configs_utils import MoveItConfigsBuilder

        config = MoveItConfigsBuilder(
            _MOVEIT_ROBOT_NAME,
            package_name=_MOVEIT_PACKAGE,
        ).to_dict()
        config["planning_scene_monitor_options"] = {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "joint_states",
            "attached_collision_object_topic": "/attached_collision_object",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": _SERVICE_TIMEOUT,
        }
        config["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        config["default_planning_pipeline"] = "ompl"
        config["plan_request_params"] = {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0,
            "planning_time": 1.0,
        }
        config.setdefault("ompl", {})
        config["ompl"]["response_adapters"] = list(_MOVEIT_RESPONSE_ADAPTERS)
        return config

    def _ensure_moveit_runtime(self) -> None:
        if self._planner_mode != "moveit2":
            return
        if self._moveit is not None:
            return

        with self._moveit_lock:
            if self._moveit is not None:
                return

            try:
                from geometry_msgs.msg import PoseStamped
                from moveit.core.robot_state import RobotState
                from moveit.planning import MoveItPy
                from moveit_msgs.msg import CollisionObject as MoveItCollisionObject
                from shape_msgs.msg import SolidPrimitive
            except ImportError as exc:
                raise RuntimeError(
                    "MoveIt2 planner requires ros-jazzy-moveit, ros-jazzy-moveit-py, "
                    "and the mycobot MoveIt config package in the ROS workspace."
                ) from exc

            try:
                self._MoveItPy = MoveItPy
                self._MoveItRobotState = RobotState
                self._MoveItCollisionObject = MoveItCollisionObject
                self._MoveItSolidPrimitive = SolidPrimitive
                self._MoveItPoseStamped = PoseStamped

                self._moveit = MoveItPy(
                    node_name=f"{self.node.get_name()}_moveit",
                    config_dict=self._moveit_config(),
                )
                self._moveit_planning_component = self._moveit.get_planning_component(
                    _MOVEIT_GROUP
                )
                self._moveit_planning_scene_monitor = self._moveit.get_planning_scene_monitor()
                self._moveit_robot_model = self._moveit.get_robot_model()
                self._sync_moveit_scene_from_store()
            except Exception as exc:
                raise RuntimeError(f"Failed to initialize MoveIt2 planner: {exc}") from exc

    def _current_joint_map(self) -> tuple[dict[str, float], list[dict[str, float]]]:
        position = self.get_position()
        joints = position.get("joints", [])
        if len(joints) != len(_JOINT_NAMES_ROS):
            raise RuntimeError(
                f"Expected {len(_JOINT_NAMES_ROS)} joints from ROS driver, got {len(joints)}"
            )

        capability_limits = self.get_capabilities()["joint_limits"]
        joint_map: dict[str, float] = {}
        out_of_bounds: list[dict[str, float]] = []

        for ros_name, capability_name, angle_deg in zip(
            _JOINT_NAMES_ROS,
            _JOINT_NAMES_CAPABILITIES,
            joints,
            strict=True,
        ):
            lower, upper = capability_limits[capability_name]
            if (
                angle_deg < lower - _MOVEIT_START_STATE_EPSILON_DEG
                or angle_deg > upper + _MOVEIT_START_STATE_EPSILON_DEG
            ):
                out_of_bounds.append(
                    {
                        "joint": capability_name,
                        "ros_joint": ros_name,
                        "angle_deg": float(angle_deg),
                        "min_deg": float(lower),
                        "max_deg": float(upper),
                    }
                )
            joint_map[ros_name] = math.radians(min(max(angle_deg, lower), upper))

        return joint_map, out_of_bounds

    def _out_of_bounds_payload(self, out_of_bounds: list[dict[str, float]]) -> dict:
        return {
            "success": False,
            "error": (
                "Current robot state is outside MoveIt2 joint limits. "
                "Re-home the arm before planning."
            ),
            "out_of_bounds_joints": out_of_bounds,
            "recovery_suggestion": "Call go_home() and retry plan_and_execute().",
        }

    def _prime_moveit_start_state(self, joint_map: dict[str, float]) -> None:
        state = self._MoveItRobotState(self._moveit_robot_model)
        state.joint_positions = dict(joint_map)
        state.update()
        self._moveit_planning_component.set_start_state(robot_state=state)

        with self._moveit_planning_scene_monitor.read_write() as scene:
            scene.current_state.joint_positions = dict(joint_map)
            scene.current_state.update()

    def _sync_moveit_scene_from_store(self) -> None:
        if self._planner_mode != "moveit2" or self._moveit_planning_scene_monitor is None:
            return

        with self._moveit_lock:
            with self._moveit_planning_scene_monitor.read_write() as scene:
                scene.remove_all_collision_objects()
                for scene_id, obj in self._collision_objects.items():
                    scene.apply_collision_object(
                        self._build_moveit_collision_object(scene_id, obj)
                    )
                scene.current_state.update()

    def _build_moveit_collision_object(
        self,
        scene_id: str,
        obj: CollisionObject,
    ):
        collision_object = self._MoveItCollisionObject()
        collision_object.header.frame_id = _MOVEIT_PLANNING_FRAME
        collision_object.id = scene_id

        primitive = self._MoveItSolidPrimitive()
        if obj.shape_type == "box":
            primitive.type = self._MoveItSolidPrimitive.BOX
            primitive.dimensions = [
                obj.dimensions["x"] / 1000.0,
                obj.dimensions["y"] / 1000.0,
                obj.dimensions["z"] / 1000.0,
            ]
        elif obj.shape_type == "cylinder":
            primitive.type = self._MoveItSolidPrimitive.CYLINDER
            primitive.dimensions = [
                obj.dimensions["height"] / 1000.0,
                obj.dimensions["radius"] / 1000.0,
            ]
        else:
            primitive.type = self._MoveItSolidPrimitive.SPHERE
            primitive.dimensions = [obj.dimensions["radius"] / 1000.0]

        pose = self._MoveItPoseStamped().pose
        pose.position.x = obj.pose["x"] / 1000.0
        pose.position.y = obj.pose["y"] / 1000.0
        pose.position.z = obj.pose["z"] / 1000.0
        qx, qy, qz, qw = self._euler_degrees_to_quaternion(
            obj.pose.get("rx", 180.0),
            obj.pose.get("ry", 0.0),
            obj.pose.get("rz", 0.0),
        )
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = self._MoveItCollisionObject.ADD
        return collision_object

    def _plan_moveit_trajectory(
        self,
        target: dict[str, float],
        joint_map: dict[str, float],
    ) -> dict:
        blockers = self._blocking_objects_for_target(target)
        if blockers:
            return {
                "success": False,
                "error": "Target pose collides with an obstacle",
                "blocking_objects": blockers,
            }

        self._ensure_moveit_runtime()

        pose_goal = self._MoveItPoseStamped()
        pose_goal.header.frame_id = _MOVEIT_PLANNING_FRAME
        pose_goal.pose.position.x = target["x"] / 1000.0
        pose_goal.pose.position.y = target["y"] / 1000.0
        pose_goal.pose.position.z = target["z"] / 1000.0
        qx, qy, qz, qw = self._euler_degrees_to_quaternion(
            target["rx"],
            target["ry"],
            target["rz"],
        )
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        try:
            with self._moveit_lock:
                self._sync_moveit_scene_from_store()
                self._prime_moveit_start_state(joint_map)
                self._moveit_planning_component.set_goal_state(
                    pose_stamped_msg=pose_goal,
                    pose_link=_MOVEIT_POSE_LINK,
                )
                plan_result = self._moveit_planning_component.plan()
        except Exception as exc:
            return {"success": False, "error": f"MoveIt2 planning failed: {exc}"}

        trajectory = getattr(plan_result, "trajectory", None)
        if trajectory is None:
            return {
                "success": False,
                "error": "MoveIt2 could not find a valid trajectory",
                "blocking_objects": blockers,
            }

        trajectory_msg = trajectory.get_robot_trajectory_msg()
        if not trajectory_msg.joint_trajectory.points:
            return {"success": False, "error": "MoveIt2 returned an empty trajectory"}

        error_code = getattr(getattr(plan_result, "error_code", None), "val", None)
        if error_code not in (None, _MOVEIT_SUCCESS_CODE):
            logger.warning("MoveIt2 returned non-success error code %s with a trajectory", error_code)

        return {
            "success": True,
            "planner": "moveit2",
            "error_code": error_code,
            "trajectory_msg": trajectory_msg,
            "trajectory": self._trajectory_summary(trajectory_msg),
            "blocking_objects": blockers,
        }

    def _trajectory_summary(self, trajectory_msg) -> dict[str, Any]:
        joint_names = list(trajectory_msg.joint_trajectory.joint_names)
        points = []
        for point in trajectory_msg.joint_trajectory.points:
            point_by_name = {
                joint_name: math.degrees(position)
                for joint_name, position in zip(
                    joint_names,
                    point.positions,
                    strict=True,
                )
            }
            points.append(
                {
                    "positions_deg": [point_by_name.get(name, 0.0) for name in joint_names],
                    "time_from_start_s": self._duration_seconds(point.time_from_start),
                }
            )
        return {
            "joint_names": joint_names,
            "point_count": len(points),
            "points": points,
        }

    def _execution_waypoints_from_trajectory(self, trajectory_msg) -> list[dict[str, Any]]:
        points = list(trajectory_msg.joint_trajectory.points)
        if not points:
            return []

        if len(points) <= _MOVEIT_MAX_EXECUTION_WAYPOINTS:
            indices = list(range(len(points)))
        else:
            indices = sorted(
                {
                    round(i * (len(points) - 1) / (_MOVEIT_MAX_EXECUTION_WAYPOINTS - 1))
                    for i in range(_MOVEIT_MAX_EXECUTION_WAYPOINTS)
                }
            )

        if len(indices) > 1 and indices[0] == 0:
            indices = indices[1:]
        if not indices:
            indices = [len(points) - 1]

        joint_names = list(trajectory_msg.joint_trajectory.joint_names)
        name_to_index = {name: index for index, name in enumerate(joint_names)}
        selected: list[dict[str, Any]] = []

        for index in indices:
            point = points[index]
            positions_deg = [
                math.degrees(point.positions[name_to_index[name]])
                for name in _JOINT_NAMES_ROS
            ]
            waypoint = {
                "positions_deg": positions_deg,
                "time_from_start_s": self._duration_seconds(point.time_from_start),
            }
            if selected and self._same_joint_goal(
                selected[-1]["positions_deg"],
                waypoint["positions_deg"],
            ):
                continue
            selected.append(waypoint)

        return selected

    def _trajectory_final_joint_map(self, trajectory_msg) -> dict[str, float]:
        final_point = trajectory_msg.joint_trajectory.points[-1]
        return {
            joint_name: position
            for joint_name, position in zip(
                trajectory_msg.joint_trajectory.joint_names,
                final_point.positions,
                strict=True,
            )
        }

    def _execute_moveit_trajectory(self, trajectory_msg, speed: float) -> dict:
        execution_waypoints = self._execution_waypoints_from_trajectory(trajectory_msg)
        executed_waypoints: list[dict[str, Any]] = []
        final_position: dict | None = None

        for waypoint in execution_waypoints:
            result = self.move_joints(waypoint["positions_deg"], speed)
            if not result.get("success"):
                return {
                    "success": False,
                    "error": result.get("error", "Failed to execute MoveIt2 waypoint"),
                    "executed_waypoints": executed_waypoints,
                }
            executed_waypoints.append(waypoint)
            final_position = result.get("final_position")

        return {
            "success": True,
            "executed_waypoints": executed_waypoints,
            "final_position": final_position or self.get_position(),
        }

    @staticmethod
    def _duration_seconds(duration_msg) -> float:
        return float(duration_msg.sec) + float(duration_msg.nanosec) / 1_000_000_000.0

    @staticmethod
    def _same_joint_goal(first: list[float], second: list[float], tolerance_deg: float = 0.5) -> bool:
        return all(abs(left - right) <= tolerance_deg for left, right in zip(first, second, strict=True))

    @staticmethod
    def _euler_degrees_to_quaternion(
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
    ) -> tuple[float, float, float, float]:
        rx = math.radians(rx_deg)
        ry = math.radians(ry_deg)
        rz = math.radians(rz_deg)

        cy = math.cos(rz * 0.5)
        sy = math.sin(rz * 0.5)
        cp = math.cos(ry * 0.5)
        sp = math.sin(ry * 0.5)
        cr = math.cos(rx * 0.5)
        sr = math.sin(rx * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    # -- ArmBackend implementation -------------------------------------------

    def get_capabilities(self) -> dict:
        capabilities = {
            "name": "myCobot 280 Pi",
            "dof": 6,
            "joint_names": list(_JOINT_NAMES_CAPABILITIES),
            "joint_limits": {
                "j1": [-165.0, 165.0],
                "j2": [-165.0, 165.0],
                "j3": [-165.0, 165.0],
                "j4": [-165.0, 165.0],
                "j5": [-165.0, 165.0],
                "j6": [-175.0, 175.0],
            },
            "max_payload_g": 250,
            "reach_mm": 280,
            "has_gripper": True,
            "has_planner": False,
            "has_camera": False,
            "has_freedrive": False,
            "backend": "ros2",
        }
        capabilities.update(self.planner_capabilities())
        return capabilities

    def get_position(self) -> dict:
        req_a = self._GetAngles.Request()
        req_c = self._GetCoords.Request()

        fut_a = self._cli_get_angles.call_async(req_a)
        fut_c = self._cli_get_coords.call_async(req_c)

        done_a, done_c = threading.Event(), threading.Event()
        fut_a.add_done_callback(lambda _: done_a.set())
        fut_c.add_done_callback(lambda _: done_c.set())

        done_a.wait(timeout=_SERVICE_TIMEOUT)
        done_c.wait(timeout=_SERVICE_TIMEOUT)

        res_a = fut_a.result()
        res_c = fut_c.result()

        joints = [
            res_a.joint_1,
            res_a.joint_2,
            res_a.joint_3,
            res_a.joint_4,
            res_a.joint_5,
            res_a.joint_6,
        ]
        cartesian = {
            "x": res_c.x,
            "y": res_c.y,
            "z": res_c.z,
            "rx": res_c.rx,
            "ry": res_c.ry,
            "rz": res_c.rz,
        }
        return self._build_position(joints, cartesian)

    def move_joints(self, angles: list[float], speed: float) -> dict:
        if self._stopped:
            return {"success": False, "error": "E-stop active. Call emergency_stop() to clear."}

        req = self._SetAngles.Request()
        req.joint_1 = float(angles[0])
        req.joint_2 = float(angles[1])
        req.joint_3 = float(angles[2])
        req.joint_4 = float(angles[3])
        req.joint_5 = float(angles[4])
        req.joint_6 = float(angles[5])
        req.speed = int(speed)

        try:
            res = self._call(self._cli_set_angles, req)
        except RuntimeError as e:
            return {"success": False, "error": str(e)}

        if not res.flag:
            return {"success": False, "error": "Driver node rejected the command"}

        time.sleep(max(0.5, (100 - speed) / 100))
        return {"success": True, "final_position": self.get_position()}

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
            return {"success": False, "error": "E-stop active. Call emergency_stop() to clear."}

        req = self._SetCoords.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.rx = float(rx)
        req.ry = float(ry)
        req.rz = float(rz)
        req.speed = int(speed)
        req.model = 0

        try:
            res = self._call(self._cli_set_coords, req)
        except RuntimeError as e:
            return {"success": False, "error": str(e)}

        if not res.flag:
            return {"success": False, "error": "Driver node rejected the command"}

        time.sleep(max(0.5, (100 - speed) / 100))
        return {"success": True, "final_position": self.get_position()}

    def go_home(self, speed: float) -> dict:
        return self.move_joints(_HOME_ANGLES, speed)

    def emergency_stop(self) -> dict:
        self._stopped = not self._stopped
        state = "activated" if self._stopped else "cleared"
        logger.warning("E-stop %s", state)
        return {"success": True, "state": state}

    def gripper(self, action: str) -> dict:
        req = self._GripperStatus.Request()
        if action == "open":
            req.status = True
        elif action == "close":
            req.status = False
        else:
            return {
                "success": False,
                "error": f"ROS 2 gripper only supports 'open'/'close', got: {action!r}",
            }

        try:
            res = self._call(self._cli_gripper, req)
        except RuntimeError as e:
            return {"success": False, "error": str(e)}

        return {"success": res.flag, "state": action if res.flag else "unknown"}

    # -- Planner overrides ---------------------------------------------------

    def add_collision_object(self, name: str, shape: dict, pose: dict) -> dict:
        result = super().add_collision_object(name, shape, pose)
        if result.get("success") and self._planner_mode == "moveit2":
            try:
                self._ensure_moveit_runtime()
                self._sync_moveit_scene_from_store()
            except Exception as exc:
                result = dict(result)
                result["success"] = False
                result["error"] = f"Failed to sync MoveIt2 collision scene: {exc}"
        return result

    def clear_collision_objects(self) -> dict:
        result = super().clear_collision_objects()
        if result.get("success") and self._planner_mode == "moveit2":
            try:
                self._ensure_moveit_runtime()
                self._sync_moveit_scene_from_store()
            except Exception as exc:
                result = dict(result)
                result["success"] = False
                result["error"] = f"Failed to clear MoveIt2 collision scene: {exc}"
        return result

    def sync_collision_objects(
        self,
        objects: list[dict],
        source: str = "perception",
    ) -> dict:
        result = super().sync_collision_objects(objects, source=source)
        if result.get("success") and self._planner_mode == "moveit2":
            try:
                self._ensure_moveit_runtime()
                self._sync_moveit_scene_from_store()
            except Exception as exc:
                result = dict(result)
                result["success"] = False
                result["error"] = f"Failed to sync MoveIt2 collision scene: {exc}"
        return result

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
        if self._planner_mode != "moveit2":
            return super().plan_and_execute(x, y, z, rx, ry, rz, speed)

        try:
            target = self._normalize_pose(
                {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}
            )
            joint_map, out_of_bounds = self._current_joint_map()
        except Exception as exc:
            return {"success": False, "error": str(exc)}

        if out_of_bounds:
            return self._out_of_bounds_payload(out_of_bounds)

        planning_result = self._plan_moveit_trajectory(target, joint_map)
        if not planning_result.get("success"):
            return planning_result

        execution_result = self._execute_moveit_trajectory(
            planning_result["trajectory_msg"],
            speed,
        )
        if not execution_result.get("success"):
            return {
                "success": False,
                "planner": "moveit2",
                "trajectory": planning_result["trajectory"],
                "error": execution_result["error"],
                "executed_waypoints": execution_result["executed_waypoints"],
            }

        return {
            "success": True,
            "planner": "moveit2",
            "trajectory": planning_result["trajectory"],
            "executed_waypoints": execution_result["executed_waypoints"],
            "final_position": execution_result["final_position"],
        }

    def plan_cartesian_path(
        self,
        waypoints: list[dict],
        speed: float,
    ) -> dict:
        if self._planner_mode != "moveit2":
            return super().plan_cartesian_path(waypoints, speed)
        if not isinstance(waypoints, list) or not waypoints:
            return {"success": False, "error": "waypoints must be a non-empty list"}

        try:
            joint_map, out_of_bounds = self._current_joint_map()
        except Exception as exc:
            return {"success": False, "error": str(exc)}

        if out_of_bounds:
            return self._out_of_bounds_payload(out_of_bounds)

        executed_waypoints: list[dict[str, Any]] = []
        segments: list[dict[str, Any]] = []
        final_position: dict | None = None

        for index, waypoint in enumerate(waypoints):
            try:
                target = self._normalize_pose(waypoint)
            except ValueError as exc:
                return {"success": False, "error": str(exc), "segment_index": index}

            planning_result = self._plan_moveit_trajectory(target, joint_map)
            if not planning_result.get("success"):
                planning_result = dict(planning_result)
                planning_result["segment_index"] = index
                planning_result["segments"] = segments
                planning_result["executed_waypoints"] = executed_waypoints
                return planning_result

            execution_result = self._execute_moveit_trajectory(
                planning_result["trajectory_msg"],
                speed,
            )
            if not execution_result.get("success"):
                return {
                    "success": False,
                    "planner": "moveit2",
                    "segment_index": index,
                    "segments": segments,
                    "trajectory": planning_result["trajectory"],
                    "error": execution_result["error"],
                    "executed_waypoints": executed_waypoints
                    + execution_result["executed_waypoints"],
                }

            executed_waypoints.extend(execution_result["executed_waypoints"])
            final_position = execution_result["final_position"]
            joint_map = self._trajectory_final_joint_map(planning_result["trajectory_msg"])
            segments.append(
                {
                    "target": target,
                    "trajectory": planning_result["trajectory"],
                }
            )

        return {
            "success": True,
            "planner": "moveit2",
            "segments": segments,
            "executed_waypoints": executed_waypoints,
            "final_position": final_position or self.get_position(),
        }
