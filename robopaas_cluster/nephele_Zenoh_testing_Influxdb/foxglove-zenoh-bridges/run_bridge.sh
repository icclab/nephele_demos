#!/bin/bash
source /root/colcon_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
BRIDGE_PID=$!

sleep 30

./zenoh-bridge-ros2dds -c zenoh-bridge-conf.json5 &
ZENOH_PID=$!

wait $BRIDGE_PID
wait $ZENOH_PID

