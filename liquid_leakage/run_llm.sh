#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash


ros2 run leakage leakage_server &
LLMSERVICE_PID=$!

sleep 1

zenoh-bridge-ros2dds -c /zenoh-bridge-conf.json5 &
ZENOH_PID=$!

wait $ROSBAGSAVER_PID
wait $ZENOH_PID


