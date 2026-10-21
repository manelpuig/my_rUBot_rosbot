#!/usr/bin/env bash
set -euo pipefail

readonly ROS_DISTRO_NAME="jazzy"
readonly WORKSPACE="${HOME}/rosbot_ws"
readonly ROSBOT_REPOSITORY="https://github.com/husarion/rosbot_ros.git"
readonly ROSBOT_BRANCH="jazzy"
readonly BASHRC_START="# >>> ROSbot native workspace >>>"

echo "=== Installing ROS 2 Jazzy and the native ROSbot workspace ==="

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: Cannot determine the operating system." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ERROR: This installer requires Ubuntu 24.04." >&2
  echo "Detected: ${PRETTY_NAME:-unknown operating system}" >&2
  exit 1
fi

architecture="$(dpkg --print-architecture)"
if [[ "${architecture}" != "amd64" ]]; then
  echo "ERROR: This installer currently supports amd64, not ${architecture}." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release \
  ca-certificates

sudo add-apt-repository universe -y

sudo install -d -m 0755 /usr/share/keyrings
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

ros_repository="deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main"
echo "${ros_repository}" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update
sudo apt-get install -y \
  ros-${ROS_DISTRO_NAME}-desktop \
  ros-dev-tools \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  git \
  wget \
  build-essential \
  ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO_NAME}-teleop-twist-keyboard \
  ros-${ROS_DISTRO_NAME}-xacro \
  ros-${ROS_DISTRO_NAME}-robot-state-publisher \
  ros-${ROS_DISTRO_NAME}-joint-state-publisher \
  ros-${ROS_DISTRO_NAME}-joint-state-publisher-gui \
  ros-${ROS_DISTRO_NAME}-rviz2 \
  ros-${ROS_DISTRO_NAME}-ros-gz \
  ros-${ROS_DISTRO_NAME}-navigation2 \
  ros-${ROS_DISTRO_NAME}-nav2-bringup \
  ros-${ROS_DISTRO_NAME}-slam-toolbox

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
else
  echo "rosdep is already initialized; keeping the existing configuration."
fi

rosdep update --rosdistro "${ROS_DISTRO_NAME}"

mkdir -p "${WORKSPACE}/src"

if [[ -d "${WORKSPACE}/src/rosbot_ros/.git" ]]; then
  existing_branch="$(git -C "${WORKSPACE}/src/rosbot_ros" branch --show-current)"
  if [[ "${existing_branch}" != "${ROSBOT_BRANCH}" ]]; then
    echo "ERROR: Existing rosbot_ros checkout uses branch '${existing_branch}'." >&2
    echo "Expected branch '${ROSBOT_BRANCH}'. Switch it manually and retry." >&2
    exit 1
  fi
  echo "Existing rosbot_ros Jazzy checkout found; keeping it unchanged."
elif [[ -e "${WORKSPACE}/src/rosbot_ros" ]]; then
  echo "ERROR: ${WORKSPACE}/src/rosbot_ros exists but is not a Git checkout." >&2
  echo "Move it aside and run the installer again." >&2
  exit 1
else
  git clone --branch "${ROSBOT_BRANCH}" \
    "${ROSBOT_REPOSITORY}" "${WORKSPACE}/src/rosbot_ros"
fi

cd "${WORKSPACE}"

# ROS setup scripts are not guaranteed to be compatible with Bash nounset.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

vcs import --skip-existing src \
  < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import --skip-existing src \
  < src/rosbot_ros/rosbot/rosbot_simulation.repos

# Required by the current upstream Husarion source-build instructions on
# Ubuntu 24.04 when a dependency must be installed through pip.
export PIP_BREAK_SYSTEM_PACKAGES=1

rosdep install \
  --ignore-src \
  --from-paths src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  -r \
  -y

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

if ! grep -Fq "${BASHRC_START}" "${HOME}/.bashrc"; then
  cat >> "${HOME}/.bashrc" <<'EOF'

# >>> ROSbot native workspace >>>
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/rosbot_ws/install/setup.bash" ]; then
  source "$HOME/rosbot_ws/install/setup.bash"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# <<< ROSbot native workspace <<<
EOF
else
  echo "ROSbot environment block already exists in ${HOME}/.bashrc."
fi

set +u
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u

echo
echo "=== Installation completed ==="
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "rosbot_gazebo share directory:"
ros2 pkg prefix --share rosbot_gazebo

echo
echo "Open a new terminal, then validate the official simulation with:"
cat <<'EOF'
ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl \
  use_sim:=True \
  configuration:=basic \
  arm_activate:=False \
  rviz:=True
EOF
