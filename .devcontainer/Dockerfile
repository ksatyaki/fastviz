ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Build chain + libclang for r2r/bindgen + winit X11/Wayland deps
# + Vulkan loader & Mesa fallback ICD. NVIDIA's Vulkan library
# (libGLX_nvidia.so.0) is bind-mounted by nvidia-container-toolkit at
# runtime; we bake the matching ICD manifest below so the loader can
# actually discover it (the toolkit on some hosts mounts the lib but
# not the manifest).
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo curl ca-certificates git vim less \
        build-essential pkg-config \
        clang libclang-dev \
        libx11-dev libxcursor-dev libxrandr-dev libxi-dev libxcb1-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libwayland-dev libssl-dev \
        libvulkan1 vulkan-tools mesa-vulkan-drivers \
        libgl1 libglu1-mesa libglvnd0 libegl1 \
        nodejs npm \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA Vulkan ICD manifest. library_path is the soname so ldconfig
# resolves it at load time — present on NVIDIA hosts (where the toolkit
# bind-mounts the lib), absent on non-NVIDIA hosts (loader logs a
# warning and falls back to Mesa). Same trick for EGL via GLVND.
RUN mkdir -p /etc/vulkan/icd.d /usr/share/glvnd/egl_vendor.d \
 && printf '%s\n' \
        '{' \
        '    "file_format_version": "1.0.0",' \
        '    "ICD": {' \
        '        "library_path": "libGLX_nvidia.so.0",' \
        '        "api_version": "1.3.289"' \
        '    }' \
        '}' \
        > /etc/vulkan/icd.d/nvidia_icd.json \
 && printf '%s\n' \
        '{' \
        '    "file_format_version" : "1.0.0",' \
        '    "ICD": {' \
        '        "library_path": "libEGL_nvidia.so.0"' \
        '    }' \
        '}' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# ROS2 message packages we'll subscribe to in M0.5
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-tf2-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot 4 simulator (Gazebo Harmonic + irobot_create_* pulled transitively).
# Kept on ros-base rather than switching to desktop-full: turtlebot4_simulator
# is third-party (Clearpath), so desktop-full would still need this apt line on
# top — and ship RViz/demos/tutorials we don't need. xacro is required because
# turtlebot4_description ships URDFs as .urdf.xacro.
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-turtlebot4-simulator \
            ros-${ROS_DISTRO}-turtlebot4-description \
            ros-${ROS_DISTRO}-xacro \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Clearpath simulator + Nav2 demos (Husky A200 etc.)
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-clearpath-simulator \
            ros-${ROS_DISTRO}-clearpath-nav2-demos \
        && rm -rf /var/lib/apt/lists/*; \
    fi

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
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
        npm install -g @anthropic-ai/claude-code; \
    fi
USER ${USERNAME}

# Auto-source ROS2 in interactive shells; r2r needs LIBCLANG_PATH at build time.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo 'export LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu' >> /home/${USERNAME}/.bashrc

# Seed Clearpath robot config (Husky A200 + Velodyne sample) and clear its
# namespace so topics publish at the root rather than under /a200_0000.
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
        mkdir -p /home/${USERNAME}/clearpath \
        && cp /opt/ros/${ROS_DISTRO}/share/clearpath_config/sample/a200_velodyne.yaml \
            /home/${USERNAME}/clearpath/robot.yaml \
        && sed -i 's|namespace: /a200_0000|namespace: /|' /home/${USERNAME}/clearpath/robot.yaml; \
    fi

WORKDIR /workspaces/fastviz
CMD ["/bin/bash"]
