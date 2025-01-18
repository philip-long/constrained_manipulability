FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip python3-colcon-clean git libboost-all-dev liboctomap-dev libcdd-dev

# Install libccd
RUN cd /tmp && git clone https://github.com/danfis/libccd.git && cd libccd \
    && mkdir -p build && cd build \
    && cmake -G 'Unix Makefiles' -DENABLE_DOUBLE_PRECISION=ON .. \
    && make -j$(nproc) && sudo make install

# Install FCL
RUN cd /tmp && git clone https://github.com/flexible-collision-library/fcl.git \
    && cd fcl && mkdir -p build && cd build \
    && cmake .. && make -j$(nproc) && sudo make install

# Install eigen-cddlib
RUN cd /tmp && git clone --recursive https://github.com/philip-long/eigen-cddlib \
    && cd eigen-cddlib && mkdir -p build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) && make install

# Create ROS workspace
WORKDIR /ros2_ws/src

# Clone robot_collision_checking code
RUN git clone https://github.com/philip-long/robot_collision_checking.git

# Clone octomap_filter code
RUN git clone https://github.com/mazrk7/octomap_filter.git

# Clone constrained_manipulability code
RUN git clone https://github.com/philip-long/constrained_manipulability.git

# Build packages with Colcon
WORKDIR /ros2_ws/
RUN sudo apt-get update && rosdep update && sudo rosdep install \
    --rosdistro humble --from-paths src --ignore-src -y
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Install cvxpy for IK solver utilities
USER $USERNAME
RUN pip install cvxpy