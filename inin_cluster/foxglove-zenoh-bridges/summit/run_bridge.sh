#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

#source /root/colcon_ws/install/setup.bash
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml asset_uri_allowlist:="['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^package://summit_xl_description/.+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']" &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml asset_uri_allowlist:="['^package://(?:[a-zA-Z]:)?(?:[\\\\/][^\\\\/:*?\"<>|]*)+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']" &
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=1000000000 & #asset_uri_allowlist:="['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^package://summit_xl_description/.+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$','^file://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^file:///home/ros/colcon_ws/src/summit_xl_common/summit_xl_description/meshes/wheels/.+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$', '^file:///home/ros/colcon_ws/src/summit_xl_common/summit_xl_description/meshes/bases.+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']" &

BRIDGE_PID=$!

sleep 30

#./zenoh-bridge-ros2dds -c zenoh-bridge-conf.json5 &
zenoh-bridge-ros2dds -c /zenoh-bridge-conf.json5 &
ZENOH_PID=$!

wait $BRIDGE_PID
wait $ZENOH_PID

