# Racerbot Workspace

The official SFU Racerbot Workspace with solutions to the RoboRacer labs and all other learning projects.

## How to Run
If you have already built your container just run: `docker start -ai racerbot`
1. Go into the `racerbot_ws` directory. 
2. Build the image:
    ```
    docker build -f .devcontainer/Dockerfile -t racerbot .
    ```
3. Run:
    ```
    docker run -it \
    --name racerbot \
    -v $(pwd)/src:/racerbot_ws/src \
    -v racerbot_ws_build:/racerbot_ws/build \
    -v racerbot_ws_install:/racerbot_ws/install \
    -v racerbot_ws_log:/racerbot_ws/log \
    --net=host \
    racerbot
    ```
4. Update dependencies:
    ```bash
    cd /racerbot_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
5. Build the workspace and source the overlay:
    ```bash
    cd /racerbot_ws
    colcon build
    source install/setup.bash
    ```

## VSCode
If you want a smooth VSCode workflow and intellisense for ROS 2 follow these instructions:
1. Install these extensions in VSCode:
    - Dev containers
    - C/C++, C/C++ Extension Pack
    - Robotics Developer Environment
    - CMake, CMake Tools
    - Container Tools
    - Python
2. Open container in VSCode
    - Open VSCode in the `racerbot_ws` directory.
    - Press Cmd+Shift+P, type “Dev Containers: Attach to Running Container”, and select your **racerbot** container.
    - **Not recommended**: Creating a new container with “Open Folder in Container” or “Reopen in Container”. This will build a fresh container instead of using your existing one.
3. Redownload your extensions (if they didn't follow) in your VSCode container environment.
4. Make sure you have your ROS 2 packages downloaded (using `rosdep`) whenever you use a new package. This will ensure the intellisense works for it.
