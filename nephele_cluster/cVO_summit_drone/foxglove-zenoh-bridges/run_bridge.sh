#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ros/colcon_ws/install/setup.bash

#source /root/colcon_ws/install/setup.bash
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml asset_uri_allowlist:="['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^package://summit_xl_description/.+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']" &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
BRIDGE_PID=$!

sleep 30

#./zenoh-bridge-ros2dds -c zenoh-bridge-conf.json5 &
zenoh-bridge-ros2dds -c /zenoh-bridge-conf.json5 &
ZENOH_PID=$!

wait $BRIDGE_PID
wait $ZENOH_PID

