FROM husarion/rosbot-gazebo:jazzy-rosbot-xl-merge

SHELL ["/bin/bash", "-c"]

# ------------------------------------------------------------
# Fix ROS2 apt source "Signed-By" conflicts in base image
# Keep ONE single ROS2 repo entry with ONE keyring.
# ------------------------------------------------------------
RUN set -eux; \
    apt-get update || true; \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg; \
    rm -rf /var/lib/apt/lists/*; \
    \
    # 1) Disable/remove any existing ROS2 source lists to avoid signed-by conflicts
    find /etc/apt/sources.list.d -maxdepth 1 -type f \
      \( -name "*ros2*.list" -o -name "*ros*.list" \) \
      -print -exec mv -f {} {}.disabled \; || true; \
    \
    # 2) Install ROS keyring (single source of truth)
    mkdir -p /etc/apt/keyrings; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg; \
    chmod 644 /etc/apt/keyrings/ros-archive-keyring.gpg; \
    \
    # 3) Add ONE ROS2 repo entry using that keyring
    . /etc/os-release; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
      > /etc/apt/sources.list.d/ros2.list; \
    \
    apt-get update; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      # ---- X11 / OpenGL ----
      x11-apps \
      mesa-utils \
      libgl1 \
      libgl1-mesa-dri \
      libglu1-mesa \
      libxkbcommon-x11-0 \
      libqt5x11extras5 \
      \
      # ---- Dev toolchain ----
      build-essential \
      cmake \
      gdb \
      git \
      unzip \
      iputils-ping \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      python3-colcon-common-extensions \
      \
      # ---- ROS GUI tools ----
      ros-jazzy-rviz2 \
      ros-jazzy-rqt \
      ros-jazzy-rqt-common-plugins \
    ; \
    rm -rf /var/lib/apt/lists/*

# Only init rosdep at build-time (avoid rosdep update here for stability)
RUN rosdep init || true

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
