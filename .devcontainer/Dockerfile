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
  apt-get install -y sudo locales \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && locale-gen en_US.UTF-8

# Base deps, tools, GL
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-venv python3-rosdep python3-lark python3-pip python3-dev python3-numpy build-essential \
  git curl wget gnupg lsb-release ca-certificates software-properties-common \
  htop less nano rsync tmux tree xclip \
  libglvnd0 libgl1 libglx0 libegl1 \
  libx11-6 libxext6 libxrender1 libxrandr2 libxi6 mesa-utils \
    && rm -rf /var/lib/apt/lists/*


# Create hybrid venv
RUN python3 -m venv /opt/venv --system-site-packages
# RUN python3 -m venv /opt/venv
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
# Ensure the non-root user can read/write here if tools emit aux files
RUN chown -R ${USERNAME}:${USERNAME} /tmp/app

USER ${USERNAME}
# Install uv and verify
RUN curl -LsSf https://astral.sh/uv/install.sh | sh && ~/.local/bin/uv --version

# Install deps defined in pyproject (dev + perception) *into /opt/venv*
RUN bash -lc 'source /opt/venv/bin/activate && cd /tmp/app && \
  ~/.local/bin/uv export --frozen --extra dev --extra perception -o /tmp/req.txt && \
  ~/.local/bin/uv pip install -U pip && \
  ~/.local/bin/uv pip install -r /tmp/req.txt'

# PyTorch CUDA wheels (CUDA 12.8 channel)
RUN bash -lc 'source /opt/venv/bin/activate && \
  ~/.local/bin/uv pip uninstall -y torch torchvision torchaudio || true && \
  ~/.local/bin/uv pip install \
  --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.8.0 torchvision==0.23.0 torchaudio==2.8.0'
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
