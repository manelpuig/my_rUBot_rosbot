# How to open this project in a Dev Container (VS Code)

This project uses Docker + VS Code Dev Containers to provide the **same ROS 2 Jazzy environment** on Windows/macOS/Linux.

## Requirements (install once)
1. **Docker Desktop**
   - Windows/macOS: install Docker Desktop and ensure it is running.
   - Linux: install Docker Engine + Docker Compose plugin.
2. **Visual Studio Code**
3. VS Code extension: **Dev Containers**  
   (Extension ID: `ms-vscode-remote.remote-containers`)

## Windows (XLaunch / VcXsrv) setup for Gazebo/RViz GUI
1. Install and start **XLaunch (VcXsrv)**.
2. In XLaunch, enable:
   - "Multiple windows"
   - "Start no client"
   - **Disable access control** (for classroom simplicity)
3. Set the DISPLAY variable in the terminal you will use to start VS Code:

PowerShell:
```powershell
$env:DISPLAY="host.docker.internal:0.0"
code .
```

Linux setup (X11)

In a terminal:

export DISPLAY=:0
xhost +local:root
code .

Open in container

Open the repository folder in VS Code.

Press F1 → Dev Containers: Reopen in Container

Wait until the container is ready.

The project will automatically run a colcon build after container creation (only if /ws/src exists and is not empty).

Build (manual)

Inside the VS Code terminal (in the container):

source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install
source install/setup.bash

Run simulation (Gazebo Sim + RViz2)
source /opt/ros/jazzy/setup.bash
cd /ws
source install/setup.bash 2>/dev/null || true

ros2 launch rosbot_gazebo simulation.launch.py \
  robot_model:=rosbot_xl \
  rviz:=True \
  gz_headless_mode:=False

rosdep (manual, only when needed)

If you added new dependencies and build fails:

cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y


Then rebuild:

colcon build --symlink-install


---

## 4) (Opcional però útil) script curt per llançar sim
Per docència, molt útil tenir `scripts/run_sim.sh`:

```bash
#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
cd /ws
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot_xl rviz:=True gz_headless_mode:=False
