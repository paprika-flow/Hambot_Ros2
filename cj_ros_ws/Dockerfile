# Start from the official ROS 2 base image
FROM ros:humble-ros-base

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential command-line tools
RUN apt-get update && apt-get install -y \
    curl \
    git \
    htop \
    nano \
    screen \
    tmux \
    usbutils \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install communication utils
RUN apt-get update && apt-get install -y \
    gpiod \
    i2c-tools \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Install specific ROS 2 and hardware drivers
RUN apt-get update && apt-get install -y \
    ros-humble-depthai-ros-driver \
    # ros-humble-navigation2 \
    # ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace directory inside the container
WORKDIR /workspace

# Automatically source ROS 2 when opening a bash terminal in the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc