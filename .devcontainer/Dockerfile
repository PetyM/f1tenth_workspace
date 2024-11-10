FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt upgrade -y && \
    apt install -y software-properties-common curl git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-transforms3d \
        ros-jazzy-desktop \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-ackermann-msgs

RUN git clone -b v1.0.0 https://github.com/f1tenth/f1tenth_gym.git /opt/f1tenth_gym && \
    chown -R vscode:vscode /opt/f1tenth_gym && \
    pip3 install --break-system-packages -e /opt/f1tenth_gym
