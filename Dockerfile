FROM registry.hub.docker.com/library/ros:noetic

# Install apt dependencies
RUN apt-get update && \
    apt-get install -y git libgflags-dev libpopt-dev \
                       libgoogle-glog-dev liblua5.1-0-dev \
                       libboost-all-dev libqt5websockets5-dev \
                       python-is-python3 libeigen3-dev sudo tmux wget unzip

# Install ROS apt dependencies
RUN apt-get install -y ros-noetic-tf ros-noetic-angles ros-noetic-cv-bridge ros-noetic-image-transport

# Install libtorch
WORKDIR /tmp
RUN wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcpu.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-1.10.0+cpu.zip -d /opt && \
    rm libtorch-cxx11-abi-shared-with-deps-1.10.0+cpu.zip


# Create and configure the user
ARG HOST_UID
RUN useradd dev -m -s /bin/bash -u $HOST_UID -G sudo && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /home/dev && \
    chown -R dev:dev /home/dev

USER dev
WORKDIR /home/dev
RUN rosdep update

# clone deps
RUN git clone https://github.com/ut-amrl/amrl_maps.git && \
    git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodules

# set up .bashrc
RUN echo "source /opt/ros/noetic/setup.sh\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/graph_navigation\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.profile
RUN echo "source /opt/ros/noetic/setup.bash\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/graph_navigation\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc


# build deps
RUN /bin/bash -lc "cd amrl_msgs && make"
RUN /bin/bash -lc "cd ut_automata && make"

# add launcher
ENV CS393R_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
RUN chmod u+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]