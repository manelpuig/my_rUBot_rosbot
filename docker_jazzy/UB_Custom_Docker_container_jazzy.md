## UB custom Docker-based ROS2 Jazzy environment

We have designed a University of Barcelona custom Docker-based ROS 2 jazzy environment to simplify student access to ROS 2 and ensure platform-independent workflows in robotics courses.

A proper Docker Image has been created with the custom configuration on Dockerfile and uploaded to my DockerHub account (https://hub.docker.com/r/manelpuig/ros2-humble-ub-biorob).


**PC-ubuntu/linux** will work on SIM and LAB use. docker-compose.yaml is configured by default for LAB use.
- In `~/my_rUBot_rosbot/docker` review on:
    - `docker-compose.yaml` file: 
        - `ROS_DOMAIN_ID=1` variable to match your Group number.
        - `ROS_AUTOMATIC_DISCOVERY_RANGE` SUBNET (SIM use) or OFF (LAB use).
        - `ROS_STATIC_PEERS` not set (SIM use) or set with your robot IP (LAB Use).
        - Be sure to include: CYCLONEDDS_URI=file:///config/cyclonedds_pc.xml

- Open a terminal in `~/my_rUBot_rosbot/docker` and run:
    ````bash
    xhost +local:root            # only in case of Host Ubuntu to allow X11 for Docker 
    docker compose up
    ````
- Verify the environment variables are correctly set by checking the container startup output.
- In Host VScode you can `attach VScode`. You can also connect with container typing:
    ```bash
    docker exec -it rosbot_jazzy bash
    code .  # to open VSCode inside the container
    ```
- Clone your ws in `/root/`
- Verify in container **.bashrc** to have:
    ```bash
    source /opt/ros/jazzy/setup.bash
    source /root//my_rUBot_rosbot/install/setup.bash
    export QT_QPA_PLATFORM=xcb  # good default for RViz2 on many systems
    cd /root/my_rUBot_mecanum
    ```
You are ready to work inside the container and to connect to the robot hardware within ROS2 jazzy on Docker!

- To stop the container, open a new terminal on Host in `~/my_rUBot_mecanum/network_config/humble` and run:
    ```bash
    docker compose down
    ```
- To see the Images and Containers:
    ```bash
    docker ps -a               # containers
    docker images              # images
    ```
- To modify the `Dockerfile`, build and push to Docker Hub, you can follow the instructions:
    ```bash
    docker build -t manelpuig/ros2-humble-ub-biorob:latest .
    docker login
    docker push manelpuig/ros2-humble-ub-biorob:latest
    ```