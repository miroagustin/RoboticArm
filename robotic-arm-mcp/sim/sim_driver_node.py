"""Simulated myCobot 280 Pi driver node.

Mirrors the API of listen_real_service.py but requires no physical hardware.
Maintains internal joint/cartesian state and publishes joint_states at 50 Hz,
so RViz shows the simulated arm moving in response to commands.

Run with:
    source ~/mycobot_ws/install/setup.bash
    python sim/sim_driver_node.py

Then start the MCP server with --backend ros2 in another terminal.
"""

import math
import sys
import os

# Ensure ROS 2 Python path is resolved even when running outside colcon
_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from mycobot_interfaces.srv import (
    GetAngles,
    GetCoords,
    GripperStatus,
    SetAngles,
    SetCoords,
)

# Joint names must match the URDF shipped with mycobot_ros2
_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Home position: arm standing upright
_HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_HOME_COORDS = [156.5, 0.0, 253.2, 180.0, 0.0, 0.0]  # x,y,z,rx,ry,rz

_JOINT_LIMITS = [
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-175.0, 175.0),
]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class SimDriverNode(Node):
    """ROS 2 node that simulates the myCobot 280 Pi hardware services.

    Services provided (identical to listen_real_service.py):
      - set_angles  (SetAngles)
      - get_angles  (GetAngles)
      - set_coords  (SetCoords)
      - get_coords  (GetCoords)
      - set_gripper (GripperStatus)

    Topic published:
      - joint_states (sensor_msgs/JointState) @ 50 Hz
    """

    def __init__(self) -> None:
        super().__init__("sim_driver_node")

        # -- Internal state --------------------------------------------------
        self._angles: list[float] = list(_HOME_ANGLES)
        self._coords: list[float] = list(_HOME_COORDS)
        self._gripper_open: bool = True

        # -- Publishers ------------------------------------------------------
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(0.02, self._publish_joint_states)  # 50 Hz

        # -- Services --------------------------------------------------------
        self.create_service(SetAngles, "set_angles", self._set_angles_cb)
        self.create_service(GetAngles, "get_angles", self._get_angles_cb)
        self.create_service(SetCoords, "set_coords", self._set_coords_cb)
        self.create_service(GetCoords, "get_coords", self._get_coords_cb)
        self.create_service(GripperStatus, "set_gripper", self._set_gripper_cb)

        self.get_logger().info("SimDriverNode ready — advertising myCobot 280 Pi services")

    # -- Kinematic approximation --------------------------------------------

    def _coords_from_angles(self, angles: list[float]) -> list[float]:
        """Approximate FK just for visualization feedback in RViz.

        The mapping is intentionally simple but internally consistent with
        `_angles_from_coords`, so cartesian commands also move joint_states.
        """
        j1, j2, j3, j4, j5, j6 = angles
        return [
            _HOME_COORDS[0] + (0.6 * j2) + (1.2 * j3),
            _HOME_COORDS[1] + j1,
            _HOME_COORDS[2] + (-0.8 * j2) + (0.4 * j3),
            _HOME_COORDS[3] + j4,
            _HOME_COORDS[4] + j5,
            _HOME_COORDS[5] + j6,
        ]

    def _angles_from_coords(self, coords: list[float]) -> list[float]:
        """Approximate IK for the sim so cartesian moves are visible in RViz."""
        dx = coords[0] - _HOME_COORDS[0]
        dy = coords[1] - _HOME_COORDS[1]
        dz = coords[2] - _HOME_COORDS[2]

        j1 = dy
        j2 = (dx / 3.0) - dz
        j3 = ((2.0 * dx) / 3.0) + (dz / 2.0)
        j4 = coords[3] - _HOME_COORDS[3]
        j5 = coords[4] - _HOME_COORDS[4]
        j6 = coords[5] - _HOME_COORDS[5]

        raw_angles = [j1, j2, j3, j4, j5, j6]
        return [
            _clamp(angle, lower, upper)
            for angle, (lower, upper) in zip(raw_angles, _JOINT_LIMITS)
        ]

    # -- Publisher -----------------------------------------------------------

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = _JOINT_NAMES
        msg.position = [math.radians(a) for a in self._angles]
        self._joint_pub.publish(msg)

    # -- Service callbacks ---------------------------------------------------

    def _set_angles_cb(self, request: SetAngles.Request, response: SetAngles.Response):
        angles = [
            request.joint_1, request.joint_2, request.joint_3,
            request.joint_4, request.joint_5, request.joint_6,
        ]
        speed = request.speed
        self.get_logger().info(
            f"set_angles: {[f'{a:.1f}' for a in angles]} (speed={speed})"
        )
        self._angles = angles
        self._coords = self._coords_from_angles(angles)
        response.flag = True
        return response

    def _get_angles_cb(self, request: GetAngles.Request, response: GetAngles.Response):
        response.joint_1 = self._angles[0]
        response.joint_2 = self._angles[1]
        response.joint_3 = self._angles[2]
        response.joint_4 = self._angles[3]
        response.joint_5 = self._angles[4]
        response.joint_6 = self._angles[5]
        return response

    def _set_coords_cb(self, request: SetCoords.Request, response: SetCoords.Response):
        coords = [request.x, request.y, request.z, request.rx, request.ry, request.rz]
        speed = request.speed
        self.get_logger().info(
            f"set_coords: {[f'{c:.1f}' for c in coords]} "
            f"(speed={speed}, model={request.model})"
        )
        self._coords = coords
        self._angles = self._angles_from_coords(coords)
        response.flag = True
        return response

    def _get_coords_cb(self, request: GetCoords.Request, response: GetCoords.Response):
        response.x = self._coords[0]
        response.y = self._coords[1]
        response.z = self._coords[2]
        response.rx = self._coords[3]
        response.ry = self._coords[4]
        response.rz = self._coords[5]
        return response

    def _set_gripper_cb(self, request: GripperStatus.Request, response: GripperStatus.Response):
        self._gripper_open = request.status
        state = "open" if request.status else "close"
        self.get_logger().info(f"set_gripper: {state}")
        response.flag = True
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
