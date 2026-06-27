FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y \
    git \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-dev-tools \
    nano \
    ros-jazzy-rmw-cyclonedds-cpp \
    vim \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

WORKDIR /rosbot_ws

RUN git clone -b jazzy https://github.com/husarion/rosbot_ros.git src/rosbot_ros

RUN . /opt/ros/jazzy/setup.sh && \
    vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos && \
    vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

RUN rosdep update --rosdistro jazzy || true

RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    apt-get install -y --fix-missing \
    ros-jazzy-controller-manager \
    ros-jazzy-gripper-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-moveit-servo \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep install -i --from-path src --rosdistro jazzy -y --skip-keys "controller_manager gripper_controllers joint_state_broadcaster joint_trajectory_controller moveit_servo" && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /rosbot_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]