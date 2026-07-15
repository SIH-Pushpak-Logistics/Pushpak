# Use the official ROS 2 Humble image (ARM64 compatible)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV USER=root

# 1. Add OSRF Repository for Gazebo Harmonic and Install Base Dependencies
RUN apt-get update && apt-get install -y curl gnupg lsb-release \
    && curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       | tee /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    python3-pip \
    redis-server \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-xacro \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    gz-harmonic \
    libgz-sim8-dev \
    rapidjson-dev \
    git \
    sudo \
    cmake \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install redis 'numpy<2' control scipy

# 2. Install ArduPilot SITL (Native ARM64, restricted to 2 cores)
WORKDIR /firmware
RUN git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git checkout Copter-4.4 \
    && git submodule update --init --recursive \
    && sed -i 's/\$EUID == 0/1 == 0/g' Tools/environment_install/install-prereqs-ubuntu.sh \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && ./waf configure --board sitl \
    && ./waf copter -j2

# 3. Build ros_gz from source targeting Harmonic (Sequential execution to prevent OOM)
WORKDIR /bridge_ws
RUN rosdep init || true && rosdep update
RUN git clone https://github.com/gazebosim/ros_gz.git -b humble src/ros_gz
RUN apt-get update && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
RUN . /opt/ros/humble/setup.sh \
    && export GZ_VERSION=harmonic \
    && export MAKEFLAGS="-j1" \
    && export CMAKE_BUILD_PARALLEL_LEVEL=1 \
    && colcon build \
        --merge-install \
        --executor sequential \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Build the ArduPilot-Gazebo Plugin
WORKDIR /bridge_ws/ardupilot_gazebo_src

# Install GStreamer dependencies right here to avoid busting the cache for ros_gz
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git . \
    && mkdir build && cd build \
    && GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j2 \
    && make install

# 5. Environment Setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /bridge_ws/install/setup.bash" >> ~/.bashrc \
    && echo "export GZ_VERSION=harmonic" >> ~/.bashrc \
    && echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc

# Set the final working directory for your host mount
WORKDIR /workspace