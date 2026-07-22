# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=kilted
ARG ROS_BASE=ros:${ROS_DISTRO}-ros-core-noble
ARG MICRO_ROS_AGENT_REF=f0d809138de19a61fe1a640d5c5a3076cd648360

FROM ${ROS_BASE} AS runtime-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=kilted
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=0 \
    OMNISEER_ROS_SETUP=/opt/ros/${ROS_DISTRO}/setup.bash \
    OMNISEER_WS_SETUP=/ros_ws/install/setup.bash \
    OMNISEER_CURRENT_REAL_PROFILE=operator

RUN set -eux; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    locales \
    lsb-release \
    software-properties-common; \
  locale-gen en_US.UTF-8; \
  rm -rf /var/lib/apt/lists/*

RUN set -eux; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    bash-completion \
    ffmpeg \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    libgrpc++1.51t64 \
    libjpeg-turbo8 \
    libopencv-core406t64 \
    libopencv-imgcodecs406t64 \
    libopencv-imgproc406t64 \
    libopencv-videoio406t64 \
    libprotobuf32t64 \
    python3-grpcio \
    python3-opencv \
    python3-protobuf \
    python3-setuptools \
    python3-yaml \
    v4l-utils \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-mecanum-drive-controller \
    ros-${ROS_DISTRO}-micro-ros-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-rclcpp-action \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-rmw-dds-common \
    ros-${ROS_DISTRO}-rmw-fastrtps-shared-cpp \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rosidl-runtime-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-mux-msgs \
    ros-${ROS_DISTRO}-xacro; \
  rm -rf /var/lib/apt/lists/*

FROM runtime-base AS builder-base

RUN set -eux; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libgrpc++-dev \
    libjpeg-dev \
    libopencv-dev \
    libprotobuf-dev \
    pkg-config \
    protobuf-compiler \
    protobuf-compiler-grpc \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-rosidl-default-generators; \
  rm -rf /var/lib/apt/lists/*

RUN set -eux; \
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  fi; \
  rosdep update

FROM builder-base AS source

WORKDIR /opt/omniseer-src

ARG MICRO_ROS_AGENT_REF

COPY scripts ./scripts
COPY ros_ws/src ./ros_ws/src
COPY vision ./vision

RUN set -eux; \
  git init /tmp/micro-ROS-Agent; \
  git -C /tmp/micro-ROS-Agent remote add origin https://github.com/micro-ROS/micro-ROS-Agent.git; \
  git -C /tmp/micro-ROS-Agent fetch --depth 1 origin "${MICRO_ROS_AGENT_REF}"; \
  git -C /tmp/micro-ROS-Agent checkout --detach FETCH_HEAD; \
  test "$(git -C /tmp/micro-ROS-Agent rev-parse HEAD)" = "${MICRO_ROS_AGENT_REF}"; \
  cp -a /tmp/micro-ROS-Agent/micro_ros_agent /opt/omniseer-src/ros_ws/src/micro_ros_agent; \
  rm -rf /tmp/micro-ROS-Agent

FROM source AS portable-builder

RUN set -eux; \
  set +u; \
  source "/opt/ros/${ROS_DISTRO}/setup.bash"; \
  set -u; \
  cd /opt/omniseer-src/ros_ws; \
  colcon build --merge-install \
    --packages-ignore analysis rf2o_laser_odometry yolo_bringup yolo_ros omniseer_vision_bridge \
    --packages-select \
      omniseer_gz_assets \
      omniseer_msgs \
      yolo_msgs \
      omniseer_description \
      omniseer_experiments \
      bringup \
      robot_io_adapters \
      robot_diag_control \
      robot_diag_control_cpp \
      micro_ros_agent \
    --cmake-args -DBUILD_TESTING=OFF

FROM source AS robot-source

RUN set -eux; \
  add-apt-repository -y ppa:jjriek/rockchip-multimedia; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    libdrm-dev \
    libv4l-0 \
    librga-dev \
    librga2 \
    ros-${ROS_DISTRO}-rplidar-ros; \
  rm -rf /var/lib/apt/lists/*

FROM robot-source AS robot-builder

COPY --from=rknn_include /rknn_api.h /usr/include/rknn_api.h
COPY --from=rknn_lib /librknnrt.so /usr/lib/librknnrt.so

RUN set -eux; \
  ldconfig; \
  set +u; \
  source "/opt/ros/${ROS_DISTRO}/setup.bash"; \
  set -u; \
  cd /opt/omniseer-src/ros_ws; \
  colcon build --merge-install \
    --packages-ignore analysis rf2o_laser_odometry yolo_bringup yolo_ros \
    --packages-select \
      omniseer_gz_assets \
      omniseer_msgs \
      yolo_msgs \
      omniseer_description \
      omniseer_experiments \
      bringup \
      robot_io_adapters \
      robot_diag_control \
      robot_diag_control_cpp \
      micro_ros_agent \
      omniseer_vision_bridge \
    --cmake-args -DBUILD_TESTING=OFF

FROM runtime-base AS portable-runtime

WORKDIR /opt/omniseer

COPY --from=portable-builder /opt/omniseer-src/ros_ws/install /ros_ws/install
COPY scripts ./scripts
COPY docker/runtime/entrypoint.sh /usr/local/bin/omniseer-runtime-entrypoint

RUN set -eux; \
  chmod 0755 /usr/local/bin/omniseer-runtime-entrypoint; \
  mkdir -p /runs

ENTRYPOINT ["/usr/local/bin/omniseer-runtime-entrypoint"]
CMD ["run", "real", "--profile", "operator", "bringup", "start_vision:=false", "start_micro_ros_agent:=false", "require_teensy:=false", "start_lidar:=false", "wait_for_boundary_topics:=false"]

FROM runtime-base AS robot-runtime

RUN set -eux; \
  add-apt-repository -y ppa:jjriek/rockchip-multimedia; \
  apt-get update; \
  apt-get install -y --no-install-recommends \
    libdrm2 \
    libv4l-0 \
    librga2 \
    ros-${ROS_DISTRO}-rplidar-ros; \
  rm -rf /var/lib/apt/lists/*

WORKDIR /opt/omniseer

COPY --from=robot-builder /usr/lib/librknnrt.so /usr/lib/librknnrt.so
COPY --from=robot-builder /opt/omniseer-src/ros_ws/install /ros_ws/install
COPY scripts ./scripts
COPY docker/runtime/entrypoint.sh /usr/local/bin/omniseer-runtime-entrypoint

RUN set -eux; \
  chmod 0755 /usr/local/bin/omniseer-runtime-entrypoint; \
  mkdir -p /runs; \
  ldconfig

ENTRYPOINT ["/usr/local/bin/omniseer-runtime-entrypoint"]
CMD ["run", "real", "--profile", "operator", "bringup"]
