ARG BASE_IMAGE="rolling-ros-core-resolute"
ARG ARIACODA_WS=/opt/ariacoda
ARG OVERLAY_WS=/opt/overlay_ws

# Build AriaCoda
FROM ubuntu:26.04 AS ariacoda-base
ARG DEBIAN_FRONTEND=noninteractive
ARG ARIACODA_WS
RUN apt update && apt install --no-install-recommends -y \
    ca-certificates build-essential doxygen git && \
    rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/grupo-avispa/AriaCoda.git ${ARIACODA_WS}
RUN cd ${ARIACODA_WS} && make && make install

# Build pioneer_ros2
FROM ros:${BASE_IMAGE} AS pioneer-build
ARG ARIACODA_WS
ARG OVERLAY_WS
COPY --from=ariacoda-base /usr/local/lib /usr/local/lib
COPY --from=ariacoda-base /usr/local/include/Aria /usr/local/include/Aria
RUN ldconfig
WORKDIR $OVERLAY_WS
RUN mkdir -p src
COPY . ./src/pioneer_ros2

# Install ROS2 dependencies and build
RUN apt update && apt install --no-install-recommends -y \
    python3-pip \
    ros-dev-tools \
    python3-vcstool \
    python3-colcon-clean \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp
RUN rosdep init && rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN colcon clean workspace --base-select build -y

# Runtime image: only execution dependencies, the shared AriaCoda library and the install space
FROM ros:${BASE_IMAGE} AS final
ARG OVERLAY_WS
ENV OVERLAY_WS=${OVERLAY_WS}
WORKDIR $OVERLAY_WS
COPY --from=pioneer-build $OVERLAY_WS/src ./src
COPY --from=ariacoda-base /usr/local/lib/libAria.so* /usr/local/lib/
RUN ldconfig
RUN apt update && apt install --no-install-recommends -y \
    python3-rosdep \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp && \
    rosdep init && rosdep update && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
      --dependency-types=exec && \
    rm -rf /var/lib/apt/lists/* /root/.ros
COPY --from=pioneer-build $OVERLAY_WS/install ./install

WORKDIR $OVERLAY_WS/src/pioneer_ros2
COPY ./docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
