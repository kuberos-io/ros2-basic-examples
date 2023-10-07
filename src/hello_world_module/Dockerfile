# Stage 1: Build ros packages
ARG ROS_VERSION="humble"
FROM ros:${ROS_VERSION}-ros-base as HumbleBase

SHELL ["/bin/bash", "-c"]


##############################################################
####                     ROS Packages                     ####
##############################################################
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    apt-utils \
    libspdlog-dev \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/*


##############################################################
####                     Source Code                      ####
##############################################################
WORKDIR /ws
ADD . /ws/src

# Build
RUN cd /ws &&\
    apt-get update &&\
    rosdep install -i --from-path src --rosdistro humble -y &&\
    rm -rf /var/lib/apt/lists/* &&\
    source /opt/ros/humble/setup.bash &&\
    colcon build



##############################################################
####           STAGE 2: Runtime Image                     ####
##############################################################
# Use the ros-core to optimize the image size
FROM ros:${ROS_VERSION}-ros-core

# Install runtime dependecies
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    apt-utils \
    libspdlog-dev \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY --from=HumbleBase /ws/install /ws/install

# RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc &&\
#     echo "source /ws/install/setup.bash" >> /root/.bashrc

ADD entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch minimal_publisher_extended publisher.launch.py init_topic:='CMD in Dockerfile'
