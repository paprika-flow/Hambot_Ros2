# Start from Tiryoh's ARM64-supported ROS 2 Humble VNC base image
FROM tiryoh/ros2-desktop-vnc:humble

# Keep user as root during build (tiryoh creates 'ubuntu' user dynamically at runtime)
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo, Nav2, SLAM, and support packages
RUN apt-get update && apt-get install -y \
    nano \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization \
    # add new packages here

    # here
    && rm -rf /var/lib/apt/lists/*

# Install packages that are used in Voronoi program
RUN pip install scipy shapely opencv-python \
    && pip install --upgrade scipy \
    && pip install scikit-learn joblib \
    && pip install tqdm
# Scikit-learn for the classifier
# tqdm for process threading when extracting features for classifier

# Set up the workspace directory inside the container
WORKDIR /home/ubuntu/hambot_sim_ws

# Force-source your workspace environment system-wide 
# (This ensures it loads automatically for the 'ubuntu' user at runtime)
RUN echo "source /home/ubuntu/hambot_sim_ws/install/setup.bash" >> /etc/bash.bashrc

# Append a system-wide simulation reset alias
RUN echo "alias reset_sim='ign service -s /world/campus_sidewalk/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 1000 --req \"name: \\\"hambot\\\", position: { x: -4.5, y: 0.0, z: 0.1 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }\"'" >> /etc/bash.bashrc