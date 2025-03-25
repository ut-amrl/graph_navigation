# Use NVIDIA's official Ubuntu 22.04 base image with CUDA and cuDNN preinstalled
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# Set environment variables for non-interactive installation and ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3

# Install prerequisites (non-ROS dependencies)
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    software-properties-common \
    build-essential \
    gcc \
    g++ \
    tmux \
    vim \
    unzip \
    ffmpeg \
    python-is-python3 \
    qtbase5-dev libqt5websockets5-dev libgtest-dev libgoogle-glog-dev libgflags-dev \
    git pkg-config \
    libbz2-dev libxml2-dev libzip-dev libboost-all-dev \
    lua5.2 liblua5.2-dev \
    lua5.1 liblua5.1-0-dev libtbb-dev \
    libjsoncpp-dev libeigen3-dev libcurl4-openssl-dev \
    libtins-dev libpcap-dev libglfw3-dev libglew-dev libspdlog-dev \
    libpng-dev libflatbuffers-dev \
    && rm -rf /var/lib/apt/lists/*

# Install a newer version of CMake (3.24.2)
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.2/cmake-3.24.2-linux-x86_64.sh \
    && chmod +x cmake-3.24.2-linux-x86_64.sh \
    && ./cmake-3.24.2-linux-x86_64.sh --skip-license --prefix=/usr/local \
    && rm cmake-3.24.2-linux-x86_64.sh

# Setup ROS2 Humble repository
RUN apt-get update && apt-get install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Humble Desktop and necessary build tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install LibTorch (C++)
RUN curl -LO https://download.pytorch.org/libtorch/cu117/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu117.zip \
    && unzip libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu117.zip -d /opt \
    && rm libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu117.zip

# Add LibTorch to the environment variables
ENV Torch_DIR=/opt/libtorch
ENV LD_LIBRARY_PATH="/opt/libtorch/lib:$LD_LIBRARY_PATH"

# Install GCC 11 for C++20 support
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update && apt-get install -y gcc-11 g++-11 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11

# Clone and build OSRM-backend at v5.27.1
RUN git clone --branch v5.27.1 https://github.com/Project-OSRM/osrm-backend.git /opt/osrm-backend
WORKDIR /opt/osrm-backend
RUN wget -O patch-6497-fix-tbbparallelpipeline.patch https://patch-diff.githubusercontent.com/raw/Project-OSRM/osrm-backend/pull/6493.patch \
    && patch -p1 < patch-6497-fix-tbbparallelpipeline.patch \
    && mkdir -p build && cd build \
    && cmake .. \
    && cmake --build . \
    && cmake --build . --target install

# Install additional ROS2 dependencies for Ouster SDK
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# (Optional) The Ouster SDK build steps are commented out here.
# If you need the SDK built, uncomment the following:
#
# WORKDIR /opt
# RUN git clone https://github.com/ouster-lidar/ouster-sdk.git
# WORKDIR /opt/ouster-sdk
# RUN mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=/usr/local
# RUN find /opt/ouster-sdk/build/ouster_osf -type f -exec sed -i 's/flatbuffers::IsOutRange/flatbuffers::IsOutOfRange/g' {} +
# RUN cd build && cmake --build . --target install -j$(nproc)

# Clone and build ouster-ros (ROS2 branch)
WORKDIR /opt
RUN git clone --branch ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
RUN mkdir -p /opt/ouster_ros_ws/src && mv /opt/ouster-ros /opt/ouster_ros_ws/src/ouster_ros
WORKDIR /opt/ouster_ros_ws
# Use --skip-keys to ignore keys that rosdep cannot resolve
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y --skip-keys="ament_lint_auto std_srvs"
# Source ROS2 setup before building so ament_cmake and other packages are found
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install Foxglove for ROS2
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-suite \
    ros-humble-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install python prerequisites
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Pip install python packages
RUN pip3 install tqdm

# Add ROS2 and workspace setup to the bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ouster_ros_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "export FRODO_BASELINE_WS=/frodo_autonomy" >> ~/.bashrc

WORKDIR /frodo_autonomy

# Set the entrypoint to bash
CMD ["/bin/bash"]
