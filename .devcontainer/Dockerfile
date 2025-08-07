#########################################
# Stage 1: Development (“dev”)
#########################################
ARG ROS_DISTRO=kilted
FROM osrf/ros:${ROS_DISTRO}-desktop-full-noble AS dev
#osrf/ros@sha256:923b7b953ada90a90535a211af71273a6b674d0098b1c0adc8e72962f48cd8c4

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# create user
RUN if id -u $USER_UID 2>/dev/null; then userdel $(id -un $USER_UID); fi \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# bring in opengl passthrough / misc tooling
RUN apt-get update && \
    apt-get install -y \
        curl gnupg lsb-release software-properties-common\
        python3-pip sudo tmux \
        libnvidia-gl-575 mesa-utils

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg2 ca-certificates

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg

# enable the building repo
RUN echo "deb [arch=$(dpkg --print-architecture) \
           signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
           http://repo.ros2.org/ubuntu/building noble main" \
        > /etc/apt/sources.list.d/ros2-building.list && \
    apt-get update && \
    sudo apt install --only-upgrade -y 'ros-kilted-rviz*'

RUN rosdep init || true && \
    rosdep update

RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
 chmod 0440 /etc/sudoers.d/$USERNAME

RUN echo "set -g mouse on" > /home/${USERNAME}/.tmux.conf \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.tmux.conf
    
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    COLCON_HOME=/home/${USERNAME}/.colcon \
    ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    SHELL=/bin/bash

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

RUN usermod -aG video ${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]


#########################################
# Stage 2: Builder (“builder”)
#########################################
FROM dev AS builder

WORKDIR /home/ws/ros_ws
COPY --chown=${USERNAME}:${USERNAME} src/ src/
RUN apt-get update
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
COPY --from=builder /home/ws/ros_ws/install /opt/ros_ws

RUN echo "source /opt/ros_ws/setup.bash" >> /etc/bash.bashrc
# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2","launch","my_pkg","robot.launch.py"]
