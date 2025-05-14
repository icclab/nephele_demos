################################################################################
# ZHAW INIT
# Description:  Dockerfile to create the Base CPU Docker image
# Authors:      Mark Straub and Leonardo Militano
# Date:         2023-11-01
################################################################################

# Base image
FROM robopaas/rosdocked-humble-nephele-summit:latest AS BASE

ENV ROS_VERSION=2
USER ros
#RUN mkdir -p /root/colcon_ws/src

# Install dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y python3-colcon-bash python3-colcon-common-extensions python3-rosdep python3-vcstool ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp iproute2 nano bmon



# Install ros-foxglove-bridge
# https://github.com/foxglove/ros-foxglove-bridge
#RUN cd /root/colcon_ws/src && \
RUN cd /home/ros/colcon_ws/ && \
    sudo git clone https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove-bridge && \
    sudo apt update && \
    rosdep update && \
    rosdep install --ignore-src --default-yes --from-path src/foxglove-bridge


#RUN bash -c "cd /root/colcon_ws && \
RUN bash -c "cd /home/ros/colcon_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --event-handlers console_direct+ --symlink-install"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/ros/cyclonedds.xml



# Download the zenoh-bridge-ros2dds binary from the release page
# https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/actions
#COPY zenoh-bridge-ros2dds /zenoh-bridge-ros2dds
# install fails because of no systemd in container
#RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && \
#    sudo apt update && sudo bash -c "apt install -y zenoh-bridge-ros2dds=1.0.2 > /dev/null 2>1; echo $?"

COPY zenoh-bridge-conf.json5 /zenoh-bridge-conf.json5

ADD ./cyclonedds.xml /home/ros/cyclonedds.xml
ADD ./run_bridge.sh /run_bridge.sh

# Source ROS
# . /opt/ros/humble/setup.bash

