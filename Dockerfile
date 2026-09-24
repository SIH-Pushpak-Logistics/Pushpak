FROM ros:humble-ros-base@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138

ENV DEBIAN_FRONTEND=noninteractive \
    USER=root \
    GZ_VERSION=harmonic \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

ARG ROS_SNAPSHOT=2026-08-07
ARG TORCH_VERSION=2.13.0
ARG TORCH_CUDA=cu126
ARG ZENOH_VERSION=1.0.0

# 01 ROS apt snapshot
COPY docker/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN grep -l 'packages.ros.org' /etc/apt/sources.list.d/* 2>/dev/null | xargs -r rm -f \
    && printf 'Types: deb\nURIs: http://snapshots.ros.org/humble/%s/ubuntu\nSuites: jammy\nComponents: main\nSigned-By: /usr/share/keyrings/ros-snapshots.asc\n' "${ROS_SNAPSHOT}" \
       > /etc/apt/sources.list.d/ros2-snapshot.sources \
    && ! grep -rq 'packages.ros.org' /etc/apt/sources.list /etc/apt/sources.list.d/

# 02 System base
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    wget \
    sudo \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# 03 OSRF Gazebo repo
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list

# 04 Core build deps + Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    libgeographic-dev \
    geographiclib-tools \
    libgz-sim8-dev \
    gz-harmonic \
    protobuf-compiler \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    rapidjson-dev \
    && rm -rf /var/lib/apt/lists/*

# 05 GeographicLib datasets
RUN apt-get update && apt-get install -y bzip2 \
    && /usr/sbin/geographiclib-get-geoids egm96-5 \
    && (/usr/sbin/geographiclib-get-gravity egm96 || true) \
    && (/usr/sbin/geographiclib-get-magnetic emm2015 || true) \
    && rm -rf /var/lib/apt/lists/*

# 06 ROS binaries (snapshot)
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-message-filters \
    ros-humble-robot-localization \
    ros-humble-xacro \
    && dpkg-query -W ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-libmavconn ros-humble-mavlink \
    && rm -rf /var/lib/apt/lists/*

# 07 ArduPilot SITL
WORKDIR /firmware
RUN git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git checkout 865cffa5 \
    && git submodule update --init --recursive \
    && sed -i 's/\$EUID == 0/1 == 0/g' Tools/environment_install/install-prereqs-ubuntu.sh \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && ./waf configure --board sitl \
    && ./waf copter -j4

# 08 ros_gz (source, Harmonic)
WORKDIR /bridge_ws
RUN rosdep init || true && rosdep update
RUN mkdir -p src \
    && git clone https://github.com/gazebosim/ros_gz.git -b humble src/ros_gz \
    && git -C src/ros_gz checkout 28e586a3
RUN apt-get update && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/humble/setup.sh \
    && export MAKEFLAGS="-j4" \
    && colcon build \
        --merge-install \
        --executor sequential \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# 09 ardupilot_gazebo plugin
WORKDIR /bridge_ws/ardupilot_gazebo_src
COPY patches/ardupilot_gazebo_imu_retry.patch /tmp/ardupilot_gazebo_imu_retry.patch
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git . \
    && git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5 \
    && patch src/ArduPilotPlugin.cc < /tmp/ardupilot_gazebo_imu_retry.patch \
    && mkdir build && cd build \
    && GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4 \
    && make install

# 10 Rust toolchain + ros2_rust
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable \
    && cargo install cargo-ament-build \
    && pip3 install --no-cache-dir colcon-cargo colcon-ros-cargo
WORKDIR /ros2_rust_ws/src
RUN git clone https://github.com/ros2-rust/ros2_rust.git \
    && cd ros2_rust \
    && (git checkout humble 2>/dev/null || true)
WORKDIR /ros2_rust_ws
RUN . /opt/ros/humble/setup.sh \
    && colcon build --merge-install

# 11 PyTorch
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/${TORCH_CUDA} \
    torch==${TORCH_VERSION} \
    torchvision

# 12 Perception + telemetry Python stack
RUN pip3 install --no-cache-dir \
    ultralytics==8.4.155 \
    protobuf==3.20.3 \
    eclipse-zenoh==${ZENOH_VERSION} \
    'numpy<2'

# 13 setuptools must be apt 59.6.0
RUN for i in 1 2 3; do pip3 uninstall -y setuptools || true; done \
    && python3 -c "import setuptools; v, p = setuptools.__version__, setuptools.__file__; print('setuptools', v, p); assert v == '59.6.0' and p.startswith('/usr/lib/python3/dist-packages/'), (v, p)"

# 14 Environment
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_rust_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /bridge_ws/install/setup.bash" >> ~/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# 15 Build-time self-check
RUN . /opt/ros/humble/setup.sh \
    && . /bridge_ws/install/setup.sh \
    && ros2 pkg prefix mavros \
    && ros2 pkg prefix mavros_extras \
    && ros2 pkg prefix robot_localization \
    && ros2 pkg prefix ros_gz_bridge \
    && ros2 pkg prefix ros_gz_sim \
    && gz sim --versions \
    && rustc --version && cargo --version \
    && test -x /firmware/ardupilot/build/sitl/bin/arducopter \
    && find /usr/local/lib -name 'libArduPilotPlugin.so' | grep -q . \
    && python3 -c "import torch, torchvision, ultralytics, zenoh, numpy, cv2, setuptools; print('torch', torch.__version__, 'cuda', torch.version.cuda, 'torchvision', torchvision.__version__, 'ultralytics', ultralytics.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__, 'setuptools', setuptools.__version__); assert numpy.__version__.startswith('1.'); assert setuptools.__version__ == '59.6.0'"

WORKDIR /workspace