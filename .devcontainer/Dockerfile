# Development image for vectorfield_nav2 (Humble, Jazzy, Lyrical).
# Built per distro by .devcontainer/<distro>/devcontainer.json; also used for local CI reproduction.
ARG BASE=ros:jazzy-ros-base-noble
FROM ${BASE}

ARG ROS_DISTRO=jazzy
# Distro-specific extras (e.g. nav2_loopback_sim / nav2_minimal_tb3_sim exist only on Jazzy+).
ARG EXTRA_APT=""

ENV DEBIAN_FRONTEND=noninteractive \
    CCACHE_DIR=/ccache \
    PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-generate-parameter-library \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-launch-testing-ament-cmake \
      ros-${ROS_DISTRO}-ament-cmake-ros \
      ros-${ROS_DISTRO}-rviz2 \
      ccache gdb clang clang-tidy lcov \
      python3-pip python3-numpy python3-matplotlib python3-pytest \
      pybind11-dev libgtest-dev libbenchmark-dev \
      ${EXTRA_APT} \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro "${ROS_DISTRO}" || true \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /etc/bash.bashrc

WORKDIR /ws
