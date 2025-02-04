
FROM robopaas/rosdocked-humble-k8s:cuda12.2.0 
USER root
# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN sudo apt-get update && sudo apt-get install -y \
        ros-humble-rmw-cyclonedds-cpp software-properties-common curl gnupg2 lsb-release

# Install required Python packages
RUN pip install pyyaml typeguard 
RUN pip install torch torchvision torchaudio 
RUN pip install opencv-python 
RUN pip install -U huggingface_hub 
ENV PATH="/home/ros/.local/bin:$PATH"
RUN huggingface-cli download --resume-download --local-dir-use-symlinks False OpenGVLab/InternVL2-8B --local-dir InternVL2-8B 
RUN pip install einops wheel timm 
RUN pip install flash_attn decord transformers accelerate sentencepiece 
RUN pip install empy==3.3.4 
RUN pip install catkin_pkg lark scikit-image 
RUN pip install numpy==1.26.4 



ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/ros/cyclonedds.xml


RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && \
sudo apt update && sudo bash -c "apt install -y zenoh-bridge-ros2dds=1.0.2 > /dev/null 2>1; echo $?"
COPY zenoh-bridge-conf.json5 /zenoh-bridge-conf.json5
ADD ./cyclonedds.xml /home/ros/cyclonedds.xml
ADD ./run_llm.sh /run_llm.sh




# create catkin ws and clone projects
RUN mkdir -p /root/ros2_ws/src && /bin/bash -c "source /opt/ros/humble/setup.bash" 

# Set the working directory
WORKDIR /root/ros2_ws/src

RUN sudo git clone https://github.com/icclab/LLM_service.git

RUN rosdep update && rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y


 # Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash &&  cd /root/ros2_ws && colcon build --symlink-install"


# Source ROS 2 and the workspace setup files
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint to ensure ROS 2 is sourced
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]
