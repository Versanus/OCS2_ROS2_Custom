ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV ROS_DISTRO=${ROS_DISTRO}
ENV WS_DIR=/workspaces/quad_ocs2_ws
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    tmux \
    build-essential \
    cmake \
    gdb \
    git \
    mesa-utils \
    nano \
    pkg-config \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    vim \
    x11-apps \
    libassimp-dev \
    libboost-all-dev \
    libgl1-mesa-dri \
    libeigen3-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    libglpk-dev \
    libglu1-mesa-dev \
    libmpfr-dev \
    liboctomap-dev \
    libpcap-dev \
    libqhull-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libx11-dev \
    libxcursor-dev \
    libxi-dev \
    libxinerama-dev \
    libxrandr-dev \
    ros-${ROS_DISTRO}-grid-map-cv \
    ros-${ROS_DISTRO}-grid-map-msgs \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-sdf \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USER_NAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USER_NAME}" \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}" \
    && chmod 0440 "/etc/sudoers.d/${USER_NAME}"

WORKDIR ${WS_DIR}
COPY . ${WS_DIR}

COPY docker/ros_entrypoint.sh /ros_entrypoint_local.sh
RUN chmod +x /ros_entrypoint_local.sh \
    && chown -R "${USER_NAME}:${USER_NAME}" "${WS_DIR}"

USER ${USER_NAME}
WORKDIR ${WS_DIR}

ENTRYPOINT ["/ros_entrypoint_local.sh"]
CMD ["bash"]