# Stage 1: Build ros packages
ARG ROS_VERSION="humble"
FROM ros:${ROS_VERSION}-ros-base as HumbleBase

SHELL ["/bin/bash", "-c"]


# Install build dependencies 
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    apt-utils \
    libspdlog-dev \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/*


# COPY File to workspace
WORKDIR /workspace
COPY . /workspace/src/
COPY shared_module /workspace/src/shared_module


# Build
RUN cd /workspace &&\
    apt-get update &&\
    rosdep install -i --from-path src --rosdistro humble -y &&\
    rm -rf /var/lib/apt/lists/* &&\
    source /opt/ros/humble/setup.bash &&\
    colcon build


# STAGE 2: Create runtime image
FROM ros:${ROS_VERSION}-ros-base

# Install runtime dependecies
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    apt-utils \
    libspdlog-dev \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY --from=HumbleBase /workspace/install /workspace/install
COPY hello_world_module/minimal_publisher/config /workspace/config/minimal_publisher
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc &&\
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]

# CMD [ "ros2 launch examples_rclpy_minimal_publisher publisher.launch.py" ]