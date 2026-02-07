#!/usr/bin/env bash
set -e

# ------------------------------------------------------------
# ROSbot XL – Jazzy – Dev/Simulation Entrypoint
# ------------------------------------------------------------

echo "[entrypoint] Starting ROSbot XL Jazzy container..."

# ------------------------------------------------------------
# Source ROS 2 Jazzy
# ------------------------------------------------------------
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[entrypoint] ERROR: /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

# ------------------------------------------------------------
# Workspace overlay (if built)
# ------------------------------------------------------------
WS="/ws"
if [ -f "${WS}/install/setup.bash" ]; then
  echo "[entrypoint] Sourcing workspace overlay"
  source "${WS}/install/setup.bash"
fi

# ------------------------------------------------------------
# Environment sanity (useful debug info)
# ------------------------------------------------------------
echo "[entrypoint] ROS_DISTRO            = ${ROS_DISTRO:-unset}"
echo "[entrypoint] RMW_IMPLEMENTATION    = ${RMW_IMPLEMENTATION:-unset}"
echo "[entrypoint] ROS_DOMAIN_ID          = ${ROS_DOMAIN_ID:-unset}"
echo "[entrypoint] DISPLAY                = ${DISPLAY:-unset}"
echo "[entrypoint] HUSARION_ROS_BUILD_TYPE = ${HUSARION_ROS_BUILD_TYPE:-unset}"

# ------------------------------------------------------------
# Default behavior
# ------------------------------------------------------------
# - If a command is passed: execute it
# - Otherwise: open an interactive shell
#
# This is important for:
#   - VS Code Dev Containers
#   - Manual colcon build
#   - Manual ros2 launch
# ------------------------------------------------------------
if [ "$#" -gt 0 ]; then
  echo "[entrypoint] Executing command: $*"
  exec "$@"
else
  echo "[entrypoint] No command provided. Opening interactive shell."
  exec bash
fi
