#########################################
# Stage 1: Development (“dev”)
#########################################
ARG ROS_DISTRO=kilted
FROM osrf/ros:${ROS_DISTRO}-desktop-full-noble AS dev

ARG USERNAME=sslab
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if id -u $USER_UID 2>/dev/null; then userdel $(id -un $USER_UID); fi \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

COPY apt-packages.txt /tmp/apt-packages.txt
RUN apt-get update \
 && xargs -a /tmp/apt-packages.txt apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      sudo \
      python3-rosdep \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros \
 && rm -rf /var/lib/apt/lists/* \
 && rosdep init || true \
 && rosdep update \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    COLCON_HOME=/home/${USERNAME}/.colcon \
    ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    SHELL=/bin/bash

# Source ROS on every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Switch to non-root user
USER ${USERNAME}
WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]


#########################################
# Stage 2: Builder (“builder”)
#########################################
FROM dev AS builder

WORKDIR /home/${USERNAME}/ros_ws
COPY --chown=${USERNAME}:${USERNAME} src/ src/
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#########################################
# Stage 3: Runtime (“runtime”)
#########################################
FROM osrf/ros:${ROS_DISTRO}-ros-base-noble AS runtime

ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Copy in only the built install space
COPY --from=builder /home/${USERNAME}/ros_ws/install /opt/ros_ws

# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2","launch","my_pkg","robot.launch.py"]
