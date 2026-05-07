FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Build chain + libclang for r2r/bindgen + winit X11/Wayland deps
# + Vulkan loader & Mesa fallback ICD (NVIDIA Vulkan ICD is mounted in
# at runtime by nvidia-container-toolkit on hosts that have it).
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo curl ca-certificates git vim less \
        build-essential pkg-config \
        clang libclang-dev \
        libx11-dev libxcb1-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev \
        libxkbcommon-dev libwayland-dev libssl-dev \
        libvulkan1 vulkan-tools mesa-vulkan-drivers \
        libgl1 libglu1-mesa libglvnd0 libegl1 \
        nodejs npm \
    && rm -rf /var/lib/apt/lists/*

# ROS2 message packages we'll subscribe to in M0.5
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-tf2-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-sensor-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
# Ubuntu 24.04 ships a default `ubuntu` user at UID/GID 1000 — remove it
# (and any group occupying our target GID) before creating `dev`.
RUN if id -u ${USER_UID} >/dev/null 2>&1; then \
        existing="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
        userdel -r "$existing" 2>/dev/null || userdel "$existing"; \
    fi \
 && if getent group ${USER_GID} >/dev/null; then \
        groupdel "$(getent group ${USER_GID} | cut -d: -f1)"; \
    fi \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd  --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable \
 && /home/${USERNAME}/.cargo/bin/rustup component add clippy rustfmt
ENV PATH=/home/${USERNAME}/.cargo/bin:${PATH}

# Claude Code CLI so in-container sessions can resume from the mounted ~/.claude
USER root
RUN npm install -g @anthropic-ai/claude-code
USER ${USERNAME}

# Auto-source ROS2 in interactive shells; r2r needs LIBCLANG_PATH at build time.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo 'export LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu' >> /home/${USERNAME}/.bashrc

WORKDIR /workspaces/fastviz
CMD ["/bin/bash"]
