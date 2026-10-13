# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Rolling development image (Apple Silicon): compile + unit tests only.
#
# Nav2 is not released into Rolling. The Nav2 nightly arm64 image is ros-base
# plus a copied /root/nav2_ws/install (Nav2 main without bringup, sims or RViz),
# and none of Nav2's own apt dependencies. They are installed here with rosdep
# over install/*/share: colcon writes COLCON_IGNORE into the install root and
# rosdep's crawler honours it, so pointing at install/ alone finds nothing.
# Demos and system tests run in the Lyrical container. Never build Nav2 from
# source under Docker Desktop.
FROM ghcr.io/ros-navigation/nav2_docker:rolling-nightly-standard-arm64

# Keys skipped by nav2_docker's docker/Dockerfile.arm64 (GUI and simulation
# packages; not built into the image). Also used by postCreateCommand.
ENV DEBIAN_FRONTEND=noninteractive \
    CCACHE_DIR=/ccache \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    VF_ROLLING_SKIP_KEYS="slam_toolbox turtlebot3_gazebo gazebo_ros_pkgs ros_gz ros_gz_sim ros_gz_bridge rviz2 nav2_rviz_plugins nav2_bringup nav2_system_tests nav2_minimal_tb3_sim nav2_minimal_tb4_description nav2_minimal_tb4_sim"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && rosdep update --rosdistro rolling \
    && . /opt/ros/rolling/setup.sh \
    && . /root/nav2_ws/install/setup.sh \
    && rosdep install -y -r --ignore-src --rosdistro rolling \
         --from-paths /root/nav2_ws/install/*/share \
         -t build -t build_export -t buildtool_export -t exec \
         --skip-keys "${VF_ROLLING_SKIP_KEYS}" \
    && apt-get install -y --no-install-recommends \
         ros-rolling-generate-parameter-library \
         ros-rolling-foxglove-bridge \
         ros-rolling-launch-testing-ament-cmake \
         ros-rolling-ament-cmake-ros \
         ccache gdb clang clang-tidy lcov \
         python3-pip python3-numpy python3-matplotlib python3-pytest \
         pybind11-dev libgtest-dev libbenchmark-dev \
    && rm -rf /var/lib/apt/lists/*

# The image sources the Nav2 workspace only from /ros_entrypoint.sh, which
# VS Code terminals and `docker exec` never run.
RUN echo 'source /opt/ros/rolling/setup.bash' >> /etc/bash.bashrc \
    && echo 'source /root/nav2_ws/install/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /etc/bash.bashrc

WORKDIR /ws
