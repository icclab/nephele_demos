# Use the official ROS 2 base image
FROM ros:humble

# Set the working directory
WORKDIR /root/ros2_ws

# Install dependencies (for ROS 2 and Git)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Clone the repository into the workspace
RUN git clone https://github.com/miltzhaw/rosbagsaver.git src/rosbagsaver

# Install Python dependencies for the ROS 2 packages (if any)
RUN pip3 install -r src/rosbagsaver/requirements.txt || echo "No requirements.txt found"
RUN pip install eclipse-zenoh

 # Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"


# Source ROS 2 and the workspace setup files
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash (for interactive use)
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run rosbagsaver my_rosbagsaver"]




ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///root/ros/cyclonedds.xml
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && \
  sudo apt update && sudo bash -c "apt install -y zenoh-bridge-ros2dds=1.0.2 > /dev/null 2>1; echo $?"
COPY zenoh-bridge-conf.json5 /zenoh-bridge-conf.json5
ADD ./cyclonedds.xml /root/ros/cyclonedds.xml
ADD ./run_rosbagsaver_tb2.sh /run_rosbagsaver_tb2.sh
ADD ./run_rosbagsaver_summit.sh /run_rosbagsaver_summit.sh
