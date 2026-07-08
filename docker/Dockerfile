FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# -----------------------------------------------------------------------------
# Base tools
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    locales \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    git \
    vim \
    wget \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-dev \
    libpython3-dev \
    python3-numpy \
    iproute2 \
    net-tools \
    libssl-dev \
    libffi-dev \
    usbutils \
    minicom \
    nano \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

# -----------------------------------------------------------------------------
# ROS 2 apt source
# -----------------------------------------------------------------------------
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# -----------------------------------------------------------------------------
# ROS 2 Humble + ROS-side CycloneDDS
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-diagnostic-updater \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    python3-empy \
    python3-lark \
    python3-catkin-pkg \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# -----------------------------------------------------------------------------
# Intel RealSense SDK + ROS 2 wrapper (D435i)
#
# Adds Intel's apt repo with GPG key FB0B24895113F120, then installs:
#   - librealsense2-utils/dev: SDK + rs-enumerate-devices, etc.
#   - ros-humble-realsense2-camera: ROS wrapper providing /camera/* topics
#   - cv-bridge + python3-opencv: for snapshot/debug scripts
#
# Note: librealsense2-dkms is intentionally excluded (host kernel module,
# meaningless inside a container).
#
# ros-humble-image-transport-plugins provides the compressed_image_transport
# publisher. Without it realsense2_camera publishes only raw image_raw, so the
# .../color/image_raw/compressed topic that comm_bridge relays never exists.
# -----------------------------------------------------------------------------
RUN mkdir -p /etc/apt/keyrings && \
    gpg --keyserver keyserver.ubuntu.com --recv-keys FB0B24895113F120 && \
    gpg --export FB0B24895113F120 > /etc/apt/keyrings/librealsense.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        librealsense2-utils \
        librealsense2-dev \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-cv-bridge \
        ros-humble-image-transport-plugins \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Audio — reSpeaker XVF3800 ALSA capture (mic_node)
#
# portaudio19-dev pulls libjack0, replacing libjack-jackd2-0; intentional
# inside the container (GStreamer/FluidSynth JACK support is unused here).
# sounddevice + scipy are installed in the pip section below.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        libportaudio2 \
        portaudio19-dev \
        libasound2-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Python packaging
#
# setuptools==59.6.0 keeps ROS 2 Humble ament_python + --symlink-install stable.
# Do not use --no-build-isolation for Python cyclonedds later.
# -----------------------------------------------------------------------------
RUN python3 -m pip install --upgrade pip wheel && \
    python3 -m pip install --force-reinstall "setuptools==59.6.0" "numpy==1.26.4"

# -----------------------------------------------------------------------------
# Build C/C++ CycloneDDS for Python cyclonedds / Unitree SDK2PY
#
# Use 0.10.5 to match Python cyclonedds==0.10.5.
# -----------------------------------------------------------------------------
WORKDIR /opt

RUN git clone --depth 1 -b 0.10.5 https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && \
    mkdir -p build && \
    cd build && \
    cmake .. \
      -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds/install \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTING=OFF && \
    cmake --build . --target install -j"$(nproc)"

ENV CYCLONEDDS_HOME=/opt/cyclonedds/install
ENV CycloneDDS_DIR=/opt/cyclonedds/install/lib/cmake/CycloneDDS
ENV CMAKE_PREFIX_PATH=/opt/cyclonedds/install:/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/cyclonedds/install/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu
ENV PATH=/opt/cyclonedds/install/bin:${PATH}

# -----------------------------------------------------------------------------
# Python CycloneDDS binding
#
# Build isolation is intentionally enabled here so pip can use its own modern
# build environment instead of global setuptools==59.6.0.
# -----------------------------------------------------------------------------
RUN python3 -m pip install \
    --no-cache-dir \
    --no-binary cyclonedds \
    "cyclonedds==0.10.5" \
    pyserial \
    sounddevice \
    scipy

# -----------------------------------------------------------------------------
# Unitree SDK2 Python
#
# --no-deps prevents pip from replacing the cyclonedds version installed above.
# -----------------------------------------------------------------------------
WORKDIR /opt

RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git /opt/unitree_sdk2_python && \
    cd /opt/unitree_sdk2_python && \
    python3 -m pip install --no-deps .

# -----------------------------------------------------------------------------
# ROS runtime defaults
# -----------------------------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0

WORKDIR /workspace/kist-drl-g1-onboard

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export CYCLONEDDS_HOME=/opt/cyclonedds/install" >> /root/.bashrc && \
    echo "export CycloneDDS_DIR=/opt/cyclonedds/install/lib/cmake/CycloneDDS" >> /root/.bashrc && \
    echo "export CMAKE_PREFIX_PATH=/opt/cyclonedds/install:/opt/ros/humble:\$CMAKE_PREFIX_PATH" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/cyclonedds/install/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:\$LD_LIBRARY_PATH" >> /root/.bashrc && \
    echo "export PATH=/opt/cyclonedds/install/bin:\$PATH" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc && \
    echo "if [ -f /workspace/kist-drl-g1-onboard/install/setup.bash ]; then source /workspace/kist-drl-g1-onboard/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
