# RoboRacer autonomy stack on top of the official AutoDRIVE devkit image.
# Build context = a folder containing:
#   roboracer-rmit/   (your repo:   src/ config/ scripts/ ...)
#   roboracer_autodrive_bridge/   (the adapter package from this guide)
#
#   docker build -t roboracer-twin:dev -f roboracer.Dockerfile .
ARG BASE=autodriveecosystem/autodrive_roboracer_api:2026-iros-explore
FROM ${BASE}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ROS dependencies of your stack (hardware drivers urg_node / vesc are NOT needed)
RUN apt update && apt install -y --no-install-recommends \
        git build-essential libeigen3-dev \
        ros-humble-slam-toolbox \
        ros-humble-tf2-geometry-msgs ros-humble-tf2-ros \
        ros-humble-eigen3-cmake-module libboost-dev \
        python3-opencv python3-yaml python3-numpy python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Autoware message definitions (autoware_control_msgs, autoware_vehicle_msgs)
RUN mkdir -p /root/autoware_msgs_ws/src && cd /root/autoware_msgs_ws/src \
    && git clone --depth 1 https://github.com/autowarefoundation/autoware_msgs.git \
    && cd .. && source /opt/ros/humble/setup.bash \
    && rosdep update || true \
    && colcon build --packages-up-to autoware_control_msgs autoware_vehicle_msgs \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# Your workspace. The stack hard-codes ~/roboracer_ws/track_memory, so the
# workspace root MUST be /root/roboracer_ws.
COPY roboracer-rmit/ /root/roboracer_ws/
COPY roboracer_autodrive_bridge/ /root/roboracer_ws/src/roboracer_autodrive_bridge/
RUN cd /root/roboracer_ws && source /opt/ros/humble/setup.bash \
    && source /root/autoware_msgs_ws/install/setup.bash \
    && colcon build --symlink-install \
         --packages-skip roboracer_bringup \
    && mkdir -p /root/roboracer_ws/track_memory/active_track

RUN echo "source /root/autoware_msgs_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /root/roboracer_ws/install/setup.bash" >> ~/.bashrc
