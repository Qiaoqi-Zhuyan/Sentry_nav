FROM ros:humble-ros-base

# create workspace
RUN mkdir -p /sentry_nav/
WORKDIR /sentry_nav/

# clone projects
RUN git clone https://github.com/sentry_nav/ --depth=1 &&
# install dependencies and some tools
RUN sudo apt install -y ros-humble-gazebo-* && \
    sudo apt install -y ros-humble-xacro && \
    sudo apt install -y ros-humble-robot-state-publisher && \
    sudo apt install -y ros-humble-joint-state-publisher && \
    sudo apt install -y ros-humble-rviz2 &&  \
    sudo apt install -y ros-humble-nav2* && \
    sudo apt install -y ros-humble-pcl-ros && \
    sudo apt install -y ros-humble-pcl-conversions && \
    sudo apt install -y ros-humble-libpointmatcher && \
    sudo apt install -y ros-humble-tf2-geometry-msgs &&   \
    sudo apt install -y libboost-all-dev && \
    sudo apt install -y libgoogle-glog-dev && \
    sudo apt install -y  libpcl-ros-dev && \

RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install ros-humble-foxglove-bridge wget htop vim -y && \
    rm -rf /var/lib/apt/lists/*

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh && \
    rm -rf /var/lib/apt/lists/*

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/sentry_nav/install/setup.bash"' \
      /ros_entrypoint.sh
