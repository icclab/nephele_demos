#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash


ros2 run rosbagsaver my_rosbagsaver process_trigger my_rosbag tcp/160.85.253.140:30447 /tb2 &
ROSBAGSAVER_PID=$!

sleep 1

zenoh-bridge-ros2dds -c /zenoh-bridge-conf.json5 &
ZENOH_PID=$!

wait $ROSBAGSAVER_PID
wait $ZENOH_PID


