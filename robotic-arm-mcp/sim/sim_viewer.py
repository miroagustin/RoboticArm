"""Launch RViz visualization for the simulated myCobot 280 Pi.

This starts:
  - robot_state_publisher with the myCobot URDF
  - rviz2 with the upstream myCobot config

It intentionally does NOT start slider_control, because that node opens the
serial port for a physical robot (/dev/ttyAMA0 by default).

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/mycobot_ws/install/setup.bash
    python sim/sim_viewer.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# Ensure ROS 2 Python path is resolved even when running inside the venv
_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def _package_share(package_name: str) -> Path:
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"ROS package {package_name!r} not found. "
            "Did you source /opt/ros/jazzy/setup.bash and ~/mycobot_ws/install/setup.bash?"
        ) from exc


def generate_launch_description() -> LaunchDescription:
    description_share = _package_share("mycobot_description")
    mycobot_280pi_share = _package_share("mycobot_280pi")

    model_path = description_share / "urdf" / "mycobot_280_pi" / "mycobot_280_pi.urdf"
    rviz_config = mycobot_280pi_share / "config" / "mycobot_pi.rviz"

    if not model_path.is_file():
        raise RuntimeError(f"URDF not found: {model_path}")
    if not rviz_config.is_file():
        raise RuntimeError(f"RViz config not found: {rviz_config}")

    robot_description = model_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
        ]
    )


def main() -> int:
    try:
        launch_description = generate_launch_description()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    launch_service = LaunchService()
    launch_service.include_launch_description(launch_description)
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
