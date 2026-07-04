# Use the official ROS 2 Humble image (ARM64 compatible)
FROM ros:humble-ros-base

# Install your specific system dependencies PERMANENTLY
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-xacro \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Install your Python mathematical and network dependencies
RUN pip3 install redis numpy control scipy

# Automatically source ROS 2 when you enter the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /workspace