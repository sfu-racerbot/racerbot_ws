# Racerbot Workspace

The official SFU Racerbot Workspace with solutions to the RoboRacer labs and all other learning projects.

## How to Run
If you have already built your container just run: `docker start -ai racerbot`
1. Clone the workspace (ideally in your racerbot folder to keep organized):
    ```
    git clone https://github.com/sfu-racerbot/racerbot_ws.git
    ```
2. Go into the `racerbot_ws` directory.
    ```
    cd racerbot_ws
    ```
3. Build the image:
    ```
    docker build -f .devcontainer/Dockerfile -t racerbot .
    ```
4. Run the container:

    If you are on Linux/MacOS:
    ```
    docker run -it \
    --name racerbot \
    -p 127.0.0.1:8765:8765 \
    -v $(pwd)/src:/racerbot_ws/src \
    -v racerbot_ws_build:/racerbot_ws/build \
    -v racerbot_ws_install:/racerbot_ws/install \
    -v racerbot_ws_log:/racerbot_ws/log \
    racerbot
    ```

    If you are on Windows:
    ```
    docker run -it `
    --name racerbot `
    -p 127.0.0.1:8765:8765 `
    -v "${PWD}/src:/racerbot_ws/src" `
    -v racerbot_ws_build:/racerbot_ws/build `
    -v racerbot_ws_install:/racerbot_ws/install `
    -v racerbot_ws_log:/racerbot_ws/log `
    racerbot
    ```
    

5. Update dependencies:
    ```bash
    cd /racerbot_ws
    sudo apt update && sudo apt upgrade -y
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
6. Build the workspace and source the overlay:
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

## F1TENTH gym environment ROS2 communication bridge
To add and run the simulator in this workspace follow these [instructions](https://github.com/Milad244/racerbot-docs/blob/main/resources/F1TENTH_GYM_ROS.md)
