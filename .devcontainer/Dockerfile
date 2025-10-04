#########################################
# Development (“dev”) stage
#########################################
ARG ROS_DISTRO=kilted
ARG ROS_BASE="osrf/ros:${ROS_DISTRO}-desktop-full-noble"
FROM ${ROS_BASE} AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    ROS_DISTRO=${ROS_DISTRO}

# Add user to group
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN set -eux; \
  getent group "${USER_GID}" || groupadd -g "${USER_GID}" "${USERNAME}"; \
  id -u "${USERNAME}" >/dev/null 2>&1 || useradd -m -s /bin/bash -u "${USER_UID}" -g "${USER_GID}" "${USERNAME}"

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo=1.9.15p5-3ubuntu5.24.04.1 locales=2.39-0ubuntu8.5 \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && locale-gen en_US.UTF-8

# Base deps, tools, GL
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv=3.12.3-0ubuntu2 python3-rosdep=0.26.0-1 python3-lark=1.1.9-1 python3-pip=24.0+dfsg-1ubuntu1.2 python3-dev=3.12.3-0ubuntu2 build-essential=12.10ubuntu1 \
    git=1:2.43.0-1ubuntu7.3 curl=8.5.0-2ubuntu10.6 wget=1.21.4-1ubuntu4.1 gnupg=2.4.4-2ubuntu17.3 lsb-release=12.0-2 ca-certificates=20240203 software-properties-common=0.99.49.3 \
    htop=3.3.0-4build1 less=590-2ubuntu2.1 nano=7.2-2ubuntu0.1 rsync=3.2.7-1ubuntu1.2 tmux=3.4-1ubuntu0.1 tree=2.1.1-2ubuntu3 xclip=0.13-3 \
    libglvnd0=1.7.0-1build1 libgl1=1.7.0-1build1 libglx0=1.7.0-1build1 libegl1=1.7.0-1build1 \
    libx11-6=2:1.8.7-1build1 libxext6=2:1.3.4-1build2 libxrender1=1:0.9.10-1.1build1 libxrandr2=2:1.5.2-2build1 libxi6=2:1.8.1-1build1 mesa-utils=9.0.0-2 \
    && rm -rf /var/lib/apt/lists/*

# Create hybrid venv
RUN python3 -m venv /opt/venv --system-site-packages
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_NO_CACHE_DIR=1
RUN chown -R ${USERNAME}:${USERNAME} /opt/venv
RUN set -eux; \
  { \
    printf '%s\n' 'if [ -n "$ROS_DISTRO" ] && [ -r "/opt/ros/$ROS_DISTRO/setup.sh" ]; then'; \
    printf '%s\n' '  . "/opt/ros/$ROS_DISTRO/setup.sh"'; \
    printf '%s\n' 'fi'; \
  } > /etc/profile.d/10-ros.sh; \
  { \
    printf '%s\n' 'if [ -d /opt/venv/bin ]; then'; \
    printf '%s\n' '  case ":$PATH:" in *":/opt/venv/bin:"*) ;; *) PATH="/opt/venv/bin:$PATH";; esac'; \
    printf '%s\n' '  export PATH'; \
    printf '%s\n' 'fi'; \
  } > /etc/profile.d/20-venv.sh; \
  chmod 0644 /etc/profile.d/10-ros.sh /etc/profile.d/20-venv.sh


## Python dependencies via uv (lockfile)
# Copy project metadata and lockfile early for layer caching
COPY pyproject.toml /tmp/app/pyproject.toml
COPY uv.lock /tmp/app/uv.lock

USER ${USERNAME}
# Install uv and verify
RUN curl -LsSf https://astral.sh/uv/install.sh | sh && ~/.local/bin/uv --version

# Use the existing venv; sync base + dev deps from the lockfile
RUN source /opt/venv/bin/activate \
  && ~/.local/bin/uv sync --frozen --extra dev

# Install PyTorch CUDA wheels from the official index
RUN source /opt/venv/bin/activate \
  && ~/.local/bin/uv pip install --index-url https://download.pytorch.org/whl/cu121 \
     torch==2.8.0 torchvision==0.23.0 torchaudio==2.8.0
USER root

# Rosdep
RUN rosdep init || true

# Copy custom bash commands
COPY .devcontainer/bash_commands.txt /etc/profile.d/99-commands.sh
RUN chmod 0644 /etc/profile.d/99-commands.sh && \
    echo '[ -f /etc/profile.d/99-commands.sh ] && . /etc/profile.d/99-commands.sh' >> /etc/bash.bashrc

# Tmux
RUN printf '%s\n' 'set -g mouse on' 'set -g history-limit 20000' >> /etc/tmux.conf

USER ${USERNAME}
