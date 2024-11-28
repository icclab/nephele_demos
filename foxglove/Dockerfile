# Start from Foxglove Studio base image
FROM ghcr.io/foxglove/studio:latest as foxglove

# Install ROS 2 Humble base
FROM ubuntu:22.04 as ros-base
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Setup ROS 2 repository
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble base
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install git, build tools, and other required libraries
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-image-transport-plugins \
    python3-colcon-common-extensions \
    inetutils-ping \
    libuvc-dev \
    build-essential \
    wget \
    vim \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    git \
    sudo \
    python3-rosdep \
    nlohmann-json3-dev

# Add environment and colcon utilities to .bashrc
RUN /bin/bash -c 'echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc'

# Create colcon workspace and clone projects
RUN mkdir -p ~/colcon_ws/src && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ~/colcon_ws/src && \
    git clone -b humble https://github.com/RobotnikAutomation/robotnik_common.git && \
    git clone -b humble https://github.com/icclab/summit_xl_common && \
    git clone -b humble https://github.com/icclab/icclab_summit_xl && \
    git clone -b humble-devel https://github.com/RobotnikAutomation/robotnik_sensors.git && \
    git clone https://github.com/Juancams/ros_astra_camera -b humble && \
    git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
    git clone -b ros2 https://github.com/tylerjw/serial.git && \
    git clone https://github.com/icclab/TurtleBot2-on-ROS2.git && \
    git clone https://github.com/kobuki-base/kobuki_core.git && \
    git clone https://github.com/icclab/kobuki_ros.git -b humble-test && \
    git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git && \
    git clone https://github.com/kobuki-base/velocity_smoother.git && \
    git clone https://github.com/kobuki-base/cmd_vel_mux.git && \
    git clone https://github.com/icclab/sllidar_ros2.git && \
    rm -rf TurtleBot2-on-ROS2/turtlebot2_commander "


# Update dependencies
RUN apt-get update && \
    /bin/bash -c "cd ~/colcon_ws && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi && \
    rosdep update && \
    rosdep install -ry --ignore-packages-from-source --from-paths src"

# Build the colcon workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ~/colcon_ws && colcon build --symlink-install"

# Combine the two images
FROM foxglove
COPY --from=ros-base /opt/ros /opt/ros
COPY --from=ros-base /root/colcon_ws /home/ros/colcon_ws
COPY --from=ros-base /root/colcon_ws/install /opt/ros/humble/share




# Add ROS and colcon workspace to the PATH and source environment
ENV PATH="/opt/ros/humble/bin:$PATH"
COPY entrypoint.sh /entrypoint.sh
ENV ROS_PACKAGE_PATH=/home/ros/colcon_ws/install:/opt/ros/humble/share
SHELL ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && exec bash"]


