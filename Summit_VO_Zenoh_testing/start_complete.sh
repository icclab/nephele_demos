#!/bin/bash

#Start a background screen name the session "summit":
screen -D -m -S summit &
PID=$!

#Start another window, name it "robot" and run robot startup in it
screen -S summit -X screen -t robot
screen -S summit -p robot -X exec roslaunch icclab_summit_xl summit_xl_base_bringup.launch

#In another window we have the ros1_bridge
screen -S summit -X screen -t ros1_bridge
screen -S summit -p ros1_bridge -X exec docker run --rm -it --privileged --network=host --ipc=host --pid=host --name ros1_bridge -v /home/summit:/home/summit ros:galactic-ros1-bridge bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"

#In another window we have the ros2 container that will start the robot base
screen -S summit -X screen -t ros2
screen -S summit -p ros2 -X exec docker run --rm -it --privileged --network=host --ipc=host --pid=host --name ros2 --env UID=$(id -u) --env GID=$(id -g) -v /home/summit:/home/summit -v /dev/lidar_front:/dev/lidar_front -v /dev/lidar_rear:/dev/lidar_rear -v /dev/astra_s:/dev/astra_s -v /dev/ttyUSB_gripper:/dev/ttyUSB_gripper robopaas/rosdocked-humble-nephele-summit:latest bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch icclab_summit_xl summit_xl_real.launch.py"
# wait for container to start
sleep 1s

#In the same container we will also start the vo-wot environment
screen -S summit -X screen -t vo-wot
screen -S summit -p vo-wot -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; vo-wot -t summit-td.json -f summit.yaml summit.py"

#In the same container we will also start the arm camera
screen -S summit -X screen -t arm_camera
screen -S summit -p arm_camera -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch depthai_ros_driver camera.launch.py"

#In the same container we will also start the front camera
screen -S summit -X screen -t front_camera
screen -S summit -p front_camera -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch icclab_summit_xl astra_mini.launch.py"

#In the same container we will also start nav2
screen -S summit -X screen -t nav2
screen -S summit -p nav2 -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch summit_xl_navigation nav2_bringup_launch.py use_sim_time:=false slam:=True"

#In the same container we will also start the arm
screen -S summit -X screen -t arm
screen -S summit -p arm -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch icclab_summit_xl arm_controller.launch.py kinematics_params_file:=/home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/ur5_calibrated_kinematics.yaml"


#In the same container we will also start zenoh_bridge
screen -S summit -X screen -t zenoh
screen -S summit -p zenoh -X exec docker exec -it ros2 bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/nephele/cyclonedds-zenoh.xml; source /home/ros/colcon_ws/install/setup.bash; zenoh-bridge-ros2dds -c /home/summit/nephele/zenoh-bridge-conf.json5"


#Attach to screen
#screen -r summit

#Echo help
echo ""
echo "1) Stop this script (ctrl-z) and put it in background (bg) if it's not already"
echo "2) Attach to screen with: screen -r summit / Detach with ctrl-a ctrl-d"
echo "3) Kill screen with : screen -S summit -X quit"

#Wait fo screen to end
wait $PID

#Kill docker containers
docker kill ros1_bridge
docker kill ros2
