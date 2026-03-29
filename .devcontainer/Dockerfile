FROM ros:humble
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y sudo tmux vim nano python3-pip \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Enable the ROS2 Testing Repo for the stupid ros-humble-foxglove-bridge package
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2-testing.list
RUN apt update

ENV SHELL=/bin/bash

RUN mkdir -p /racerbot_ws/log /racerbot_ws/build /racerbot_ws/install /racerbot_ws/src \
    && chown -R $USERNAME:$USERNAME /racerbot_ws

USER $USERNAME
WORKDIR /racerbot_ws
CMD ["/bin/bash"]
