####################################
###  STAGE 1: Development        ###
####################################
ARG ROS_DISTRO=kilted
FROM osrf/ros:${ROS_DISTRO}-desktop-full-noble AS dev

ARG USERNAME=sslab
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN apt-get update

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends python3-pip python3-rosdep
RUN rosdep init || true && rosdep update

ENV DEBIAN_FRONTEND=noninteractive \
LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
COLCON_HOME=/home/ws/.colcon \
ROS_DOMAIN_ID=42 \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
SHELL=/bin/bash

RUN echo "source /opt/ros/kilted/setup.bash" >> /etc/bash.bashrc

USER $USERNAME
CMD ["/bin/bash"]

####################################
###  STAGE 2: Building           ###
####################################
FROM dev AS builder

WORKDIR /home/sslab/ros_ws
COPY --chown=sslab:sslab src/ src/

# Install package dependencies and build once
RUN rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

####################################
###  STAGE 3: Runtime            ###
####################################
ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base-noble AS runtime

ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Copy only the built artefacts
COPY --from=builder /home/sslab/ros_ws/install /opt/ros_ws

# Execute launchfile

# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2","launch","my_pkg","robot.launch.py"]
