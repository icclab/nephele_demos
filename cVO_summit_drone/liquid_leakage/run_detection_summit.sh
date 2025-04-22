#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

zenoh-bridge-ros2dds -c /zenoh-bridge-conf.json5 &
ZENOH_PID=$!
sleep 1s 

# ros2 launch image_pose_pub image_pose_launch.py compressed_image_topic:=/summit/summit/oak/rgb/image_raw/compressed &
# IMAGEPOSE_PID=$!
# sleep 1s 


# ros2 launch ultralytics_ros tracker.launch.xml debug:=false &
# ULTRALYTICS_PID=$!
# sleep 1s 

ros2 launch posture_detection posture_server_launch.py &
POSTURESERVICE_PID=$!

sleep 10m

ros2 launch posture_detection posture_detection_launch.py &
POSTURECLIENT_PID=$!

# wait $ULTRALYTICS_PID
wait $POSTURESERVICE_PID
# wait $IMAGEPOSE_PID
wait $POSTURECLIENT_PID
wait $ZENOH_PID


