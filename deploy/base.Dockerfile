FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS_WS=/ws \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-moveit \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-py \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update --rosdistro "${ROS_DISTRO}"

RUN mkdir -p "${ROS_WS}/src" && \
    git clone --depth 1 --branch humble \
      https://github.com/elephantrobotics/mycobot_ros2.git \
      "${ROS_WS}/src/mycobot_ros2"

RUN source /opt/ros/"${ROS_DISTRO}"/setup.bash && \
    rosdep install --from-paths "${ROS_WS}/src" --ignore-src -r -y --rosdistro "${ROS_DISTRO}" || true && \
    colcon build \
      --base-paths "${ROS_WS}/src" \
      --build-base "${ROS_WS}/build" \
      --install-base "${ROS_WS}/install" \
      --merge-install \
      --symlink-install

WORKDIR /app
