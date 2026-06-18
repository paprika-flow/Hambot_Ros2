# Start from the official ROS 2 base image
FROM ros:humble-ros-base

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Combine updates and installations to reduce image layer size
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    htop \
    nano \
    screen \
    tmux \
    usbutils \
    wget \
    gpiod \
    i2c-tools \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-depthai-ros-driver \
    && rm -rf /var/lib/apt/lists/*

# Fix potential missing dependencies for Oak-D / DepthAI cameras on Pi
RUN pip3 install --no-cache-dir opencv-python-headless 

#Install motor driver
RUN pip3 install --no-cache-dir buildhat

# Set up the workspace directory inside the container
WORKDIR /workspace

# Force entrypoint shell execution to utilize the selected DDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Automatically source ROS 2 when opening a bash terminal in the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc